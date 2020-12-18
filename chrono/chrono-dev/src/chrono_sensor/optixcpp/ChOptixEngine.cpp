// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// OptiX rendering engine for processing jobs for sensing. Jobs are defined on
// each sensor as a graph. Recommended to use one engine per GPU to mitigate
// OptiX blocking launch calls
//
// =============================================================================

#include "chrono_sensor/optixcpp/ChOptixEngine.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChDepthCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/optixcpp/ChOptixUtils.h"
#include "chrono_sensor/scene/lights.h"
#include "chrono_sensor/utils/ChVisualMaterialUtils.h"

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChRoundedBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

using namespace optix;

ChOptixEngine::ChOptixEngine(ChSystem* sys) : m_deviceId(0) {
    m_system = sys;
    Initialize();
}
ChOptixEngine::ChOptixEngine(ChSystem* sys, int device_id) : m_deviceId(device_id) {
    m_system = sys;
    Initialize();
}
ChOptixEngine::~ChOptixEngine() {
    if (!m_terminate) {
        Stop();  // if it hasn't been stopped yet, stop it ourselves
    }
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void ChOptixEngine::AssignSensor(std::shared_ptr<ChSensor> sensor) {
    // all optix sensor should be treated the same. Up to the sensor to specify a difference by using a unique lunch
    // kernel

    if (auto pOptixSensor = std::dynamic_pointer_cast<IRendersWithOptix>(sensor)) {
        pOptixSensor->Context() = m_context;
        m_assignedSensor.push_back(sensor);

        // initialize filters just in case they want to create any chunks of memory from the start
        for (auto f : sensor->FilterList()) {
            f->Initialize(sensor);
        }
    } else {
        std::cerr << "Error: unable to add non-optix sensor to an optix engine\n";
    }
}  // namespace sensor

void ChOptixEngine::UpdateSensor(std::shared_ptr<ChScene> scene) {
    if (!m_started) {
        Start();
    }

    bool added_sensor = false;
    std::unique_lock<std::mutex> tmp_lock;

    for (auto sensor : m_assignedSensor) {
        if (m_system->GetChTime() * sensor->GetUpdateRate() + 1e-5 > sensor->GetNumUpdates()) {
            sensor->IncrementNumUpdates();
            if (!added_sensor) {  // only add the lock once
                tmp_lock = std::unique_lock<std::mutex>(m_renderQueueMutex);
                added_sensor = true;
            }
            m_renderQueue.push_back(sensor);
        }
    }
    if (added_sensor) {
        // m_keyframes = keyframes;

        // update the scene if we want to submit any sensor
        UpdateCameraTransforms();
        UpdateBodyTransforms();
        UpdateSceneDescription(scene);

        tmp_lock.unlock();
        m_renderQueueCV.notify_all();
    }
}

void ChOptixEngine::Stop() {
    std::unique_lock<std::mutex> tmp_lock(m_renderQueueMutex);
    m_terminate = true;
    tmp_lock.unlock();
    m_renderQueueCV.notify_all();
}

void ChOptixEngine::Start() {
    if (!m_started) {
        m_thread = std::move(std::thread(&ChOptixEngine::Process, this));
        m_started = true;
    }
}

void ChOptixEngine::Process() {
    // continually loop and perform two functions: add filters from sensor to job queue, empty the job queue

    bool terminate = false;

    // keep the thread running until we submit a terminate job or equivalent
    while (!terminate) {
        // std::cout << "Engine thread creating a lock on the mutex\n";
        std::unique_lock<std::mutex> tmp_lock(m_renderQueueMutex);

        // wait for a notification from the master thread
        while (m_renderQueue.empty() && !m_terminate) {
            m_renderQueueCV.wait(tmp_lock);
        }

        terminate = m_terminate;

        if (!terminate) {
            for (auto pSensor : m_renderQueue) {
                std::shared_ptr<SensorBuffer> buffer;
                // step through the filter list, applying each filter
                for (auto filter : pSensor->FilterList()) {
                    filter->Apply(pSensor, buffer);
                }
            }
        }

        m_renderQueue.clear();  // empty list of sensor when everything is processed
        tmp_lock.unlock();      // release the lock on the job queue
    }
}

void ChOptixEngine::Initialize() {
    // initialize an optix context -> one for each render group
    m_context = Context::create();
    rtContextSetDevices(m_context->get(), 1, &m_deviceId);

    // try to enable RTX acceleration if it is available
    int rtx = 1;
    if (rtGlobalSetAttribute(RT_GLOBAL_ATTRIBUTE_ENABLE_RTX, sizeof(int), &rtx) != RT_SUCCESS)
        printf("RTX Mode Disabled on this device. \n");

    // set context-wide parameters -> make changeable by functions before initialization
    m_context->setRayTypeCount(2);
    m_context->setMaxTraceDepth(31);  // max allowable by OptiX, doesn't mean we should try to go this far though

    m_context["max_scene_distance"]->setFloat(1.e4f);
    m_context["max_depth"]->setInt(m_recursions);
    m_context["scene_epsilon"]->setFloat(1.0e-3f);
    m_context["importance_cutoff"]->setFloat(0.01f);
    m_context["ambient_light_color"]->setFloat(0.2f, 0.2f, 0.2f);

    m_root = m_context->createGroup();
    m_root->setAcceleration(m_context->createAcceleration("Trbvh"));
    // m_root->getAcceleration()->setProperty("refit", "1");
    m_context["root_node"]->set(m_root);

    optix::Program miss_program = GetRTProgram(m_context, "miss", "miss_function");
    m_context->setMissProgram(0, miss_program);
    miss_program["default_color"]->setFloat(0.5f, 0.6f, 0.7f);
    miss_program["default_depth"]->setFloat(-1.f);
    // TextureSampler tex_sampler = CreateTexture(GetChronoDataFile("sensor/textures/cloud_layers_8k.hdr"));
    TextureSampler tex_sampler = CreateTexture();
    miss_program["environment_map"]->setTextureSampler(tex_sampler);
    miss_program["has_environment_map"]->setInt(0);

    m_light_buffer = m_context->createBuffer(RT_BUFFER_INPUT);
    m_light_buffer->setFormat(RT_FORMAT_USER);

    m_light_buffer->setElementSize(sizeof(PointLight));
    m_light_buffer->setSize(0);
    // memcpy(m_light_buffer->map(), scene->GetPointLights().data(), scene->GetPointLights().size() *
    // sizeof(PointLight));
    // m_light_buffer->unmap();

    m_context["lights"]->set(m_light_buffer);
}

void ChOptixEngine::ConstructScene() {
    // update what objects are in scene
    // remove all children from root node
    while (m_root->getChildCount() > 0) {
        m_root->removeChild(0);
    }

    m_bodies.clear();
    // create the programs for each geometry -> TODO: refactor so only only one instance of a geometry or material
    // is made
    Program box_bounds = GetRTProgram(m_context, "box", "box_bounds");
    Program box_intersect = GetRTProgram(m_context, "box", "box_intersect");

    Program sphere_bounds = GetRTProgram(m_context, "sphere", "sphere_bounds");
    Program sphere_intersect = GetRTProgram(m_context, "sphere", "sphere_intersect");

    // iterate through all bodies in Chrono and add a subnode for each body in Chrono
    for (auto body : m_system->Get_bodylist()) {
        // check that the body list is not empty
        if (body->GetAssets().size() > 0) {
            // collect position and orientation of the body from Chrono
            const ChVector<double> body_pos = body->GetFrame_REF_to_abs().GetPos();
            const ChMatrix33<double> body_rot_mat = body->GetFrame_REF_to_abs().Amatrix;

            // set the transform that is applicable to the entire body
            // Transform body_transform = m_context->createTransform();
            Transform body_transform = CreateTransform(body_rot_mat, body_pos);

            bool added_asset_for_body = false;

            optix::Group asset_group = m_context->createGroup();
            asset_group->setAcceleration(m_context->createAcceleration("Trbvh"));
            body_transform->setChild(asset_group);

            // iterate through all assets in the body
            for (auto asset : body->GetAssets()) {
                // check if the asset is a ChVisualization

                if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)) {
                    // collect relative position and orientation of the asset
                    ChVector<double> asset_pos = visual_asset->Pos;
                    ChMatrix33<double> asset_rot_mat = visual_asset->Rot;

                    if (!visual_asset->IsVisible()) {
                        std::cout << "Ignoring an asset that is set to invisible\n";
                    } else if (std::shared_ptr<ChBoxShape> box_shape = std::dynamic_pointer_cast<ChBoxShape>(asset)) {
                        ChVector<float> size = box_shape->GetBoxGeometry().GetLengths();

                        // create the box geometry
                        Geometry box = m_context->createGeometry();
                        box->setPrimitiveCount(1u);
                        box->setBoundingBoxProgram(box_bounds);
                        box->setIntersectionProgram(box_intersect);
                        box["boxmin"]->setFloat(-size.x() / 2.0f, -size.y() / 2.0f, -size.z() / 2.0f);
                        box["boxmax"]->setFloat(size.x() / 2.0f, size.y() / 2.0f, size.z() / 2.0f);

                        // create a material for the box
                        Material matl;  // = m_context->createMaterial();
                        if (box_shape->material_list.size() == 0) {
                            matl = CreateMaterial();
                        } else {
                            matl = CreateMaterial(box_shape->material_list[0]);
                        }

                        // create geometry instance for the box
                        GeometryInstance gis = m_context->createGeometryInstance(box, &matl, &matl + 1);
                        GeometryGroup geometrygroup = m_context->createGeometryGroup();
                        geometrygroup->setChildCount(1);
                        geometrygroup->setChild(0, gis);  // add the box geometry to the geometry group
                        geometrygroup->setAcceleration(m_context->createAcceleration("Trbvh"));
                        // geometry_group->addChild(gis);
                        // geometrygroup->getAcceleration()->setProperty("refit", "1");

                        // create a transform that will manipulate the box
                        // Transform asset_transform = m_context->createTransform();
                        Transform asset_transform = CreateTransform(asset_rot_mat, asset_pos);

                        // add the asset transform as child of the body transform
                        asset_group->addChild(asset_transform);
                        // add the geometry group as child of the asset transform
                        asset_transform->setChild(geometrygroup);

                        added_asset_for_body = true;

                    } else if (std::shared_ptr<ChSphereShape> sphere_shape =
                                   std::dynamic_pointer_cast<ChSphereShape>(asset)) {
                        float radius = (float)sphere_shape->GetSphereGeometry().rad;
                        ChVector<double> center = sphere_shape->GetSphereGeometry().center;

                        // create the sphere geometry
                        Geometry sphere = m_context->createGeometry();
                        sphere->setPrimitiveCount(1u);
                        sphere->setBoundingBoxProgram(sphere_bounds);
                        sphere->setIntersectionProgram(sphere_intersect);
                        sphere["sphere"]->setFloat(center.x(), center.y(), center.z(), radius);

                        // create a material to do diffuse shading
                        Material matl;  // = m_context->createMaterial();
                        if (sphere_shape->material_list.size() == 0) {
                            matl = CreateMaterial();
                        } else {
                            matl = CreateMaterial(sphere_shape->material_list[0]);
                        }

                        // create geometry instance for the sphere
                        GeometryInstance gis = m_context->createGeometryInstance(sphere, &matl, &matl + 1);
                        GeometryGroup geometrygroup = m_context->createGeometryGroup();
                        geometrygroup->setChildCount(1);
                        geometrygroup->setChild(0, gis);  // add the sphere geometry to the geometry group
                        geometrygroup->setAcceleration(m_context->createAcceleration("Trbvh"));
                        // geometrygroup->getAcceleration()->setProperty("refit", "1");

                        // create a transform that will manipulate the sphere
                        // Transform asset_transform = m_context->createTransform();
                        Transform asset_transform = CreateTransform(asset_rot_mat, asset_pos);

                        // add the asset transform as child of the body transform
                        asset_group->addChild(asset_transform);
                        asset_transform->setChild(geometrygroup);

                        added_asset_for_body = true;

                    } else if (std::shared_ptr<ChTriangleMeshShape> trimesh_shape =
                                   std::dynamic_pointer_cast<ChTriangleMeshShape>(asset)) {
                        std::shared_ptr<geometry::ChTriangleMeshConnected> mesh = trimesh_shape->GetMesh();
                        // std::cout << "Optix sees mesh: " << mesh->m_filename << std::endl;
                        if (trimesh_shape->material_list.size() == 0) {
                            // add a "proper" mesh if one doesn't alrealy exist for it
                            CreateModernMeshAssets(trimesh_shape);
                        }

                        // create all the materials
                        // create a material to do diffuse shading
                        std::vector<Material> mat_list;
                        for (int m = 0; m < trimesh_shape->material_list.size(); m++) {
                            Material matl;  // = m_context->createMaterial();
                            auto vis_mat = trimesh_shape->material_list[m];
                            matl = CreateMaterial(vis_mat);

                            mat_list.push_back(matl);
                        }
                        if (mat_list.size() == 0) {
                            Material matl;  // = m_context->createMaterial();
                            matl = CreateMaterial();

                            mat_list.push_back(matl);
                        }

                        if (mesh->m_face_col_indices.size() == 0) {
                            mesh->m_face_col_indices = std::vector<ChVector<int>>(mesh->getIndicesVertexes().size());
                        }
                        // setup mesh buffers
                        optix::Buffer index_buffer = m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT3,
                                                                             mesh->getIndicesVertexes().size());
                        optix::Buffer vertex_buffer = m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3,
                                                                              mesh->getCoordsVertices().size());
                        optix::Buffer normal_buffer =
                            m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT3, mesh->getCoordsNormals().size());
                        optix::Buffer texcoord_buffer =
                            m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_FLOAT2, mesh->getCoordsUV().size());
                        optix::Buffer mat_index_buffer = m_context->createBuffer(
                            RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_INT, mesh->getIndicesColors().size());

                        std::vector<unsigned int> tmp_index_buffer =
                            std::vector<unsigned int>(3 * mesh->getIndicesVertexes().size());
                        std::vector<float> tmp_vertex_buffer = std::vector<float>(3 * mesh->getCoordsVertices().size());
                        std::vector<float> tmp_normal_buffer = std::vector<float>(3 * mesh->getCoordsNormals().size());
                        std::vector<float> tmp_texcoord_buffer = std::vector<float>(2 * mesh->getCoordsUV().size());
                        std::vector<unsigned int> tmp_mat_index_buffer =
                            std::vector<unsigned int>(mesh->getIndicesColors().size());

                        // copy over the index buffer
                        for (int i = 0; i < mesh->getIndicesVertexes().size(); i++) {
                            tmp_index_buffer[3 * i] = mesh->getIndicesVertexes().data()[i].x();
                            tmp_index_buffer[3 * i + 1] = mesh->getIndicesVertexes().data()[i].y();
                            tmp_index_buffer[3 * i + 2] = mesh->getIndicesVertexes().data()[i].z();
                        }

                        // copy over the material index buffer
                        for (int i = 0; i < mesh->getIndicesColors().size(); i++) {
                            tmp_mat_index_buffer[i] = mesh->getIndicesColors().data()[i].x();
                        }

                        // copy over the vertex buffer
                        for (int i = 0; i < mesh->getCoordsVertices().size(); i++) {
                            tmp_vertex_buffer[3 * i] = (float)mesh->getCoordsVertices().data()[i].x();
                            tmp_vertex_buffer[3 * i + 1] = (float)mesh->getCoordsVertices().data()[i].y();
                            tmp_vertex_buffer[3 * i + 2] = (float)mesh->getCoordsVertices().data()[i].z();
                        }

                        // copy over the normal buffer
                        for (int i = 0; i < mesh->getCoordsNormals().size(); i++) {
                            tmp_normal_buffer[3 * i] = (float)mesh->getCoordsNormals().data()[i].x();
                            tmp_normal_buffer[3 * i + 1] = (float)mesh->getCoordsNormals().data()[i].y();
                            tmp_normal_buffer[3 * i + 2] = (float)mesh->getCoordsNormals().data()[i].z();
                        }

                        // copy over the texcoord buffer
                        for (int i = 0; i < mesh->getCoordsUV().size(); i++) {
                            tmp_texcoord_buffer[2 * i] = (float)mesh->getCoordsUV().data()[i].x();
                            tmp_texcoord_buffer[2 * i + 1] = (float)mesh->getCoordsUV().data()[i].y();
                        }

                        memcpy(index_buffer->map(), tmp_index_buffer.data(),
                               sizeof(unsigned) * tmp_index_buffer.size());
                        index_buffer->unmap();

                        memcpy(vertex_buffer->map(), tmp_vertex_buffer.data(),
                               sizeof(float) * tmp_vertex_buffer.size());
                        vertex_buffer->unmap();

                        memcpy(normal_buffer->map(), tmp_normal_buffer.data(),
                               sizeof(float) * tmp_normal_buffer.size());
                        normal_buffer->unmap();

                        memcpy(texcoord_buffer->map(), tmp_texcoord_buffer.data(),
                               sizeof(float) * tmp_texcoord_buffer.size());
                        texcoord_buffer->unmap();

                        memcpy(mat_index_buffer->map(), tmp_mat_index_buffer.data(),
                               sizeof(unsigned) * tmp_mat_index_buffer.size());
                        mat_index_buffer->unmap();

                        // create the optix nodes for the triangle mesh
                        optix::GeometryTriangles tris = m_context->createGeometryTriangles();
                        tris->setPrimitiveCount((unsigned int)mesh->getIndicesVertexes().size());
                        tris->setTriangleIndices(index_buffer, RT_FORMAT_UNSIGNED_INT3);
                        tris->setVertices((unsigned int)mesh->getCoordsVertices().size(), vertex_buffer,
                                          RT_FORMAT_FLOAT3);
                        // tris->setMaterialCount(1);  // TODO: allow multiple materials for a single mesh
                        tris->setMaterialCount((unsigned int)mat_list.size());
                        tris->setMaterialIndices(mat_index_buffer, 0, sizeof(unsigned), RT_FORMAT_UNSIGNED_INT);
                        tris->setAttributeProgram(GetRTProgram(m_context, "triangle_mesh", "mesh_attributes"));

                        optix::GeometryInstance triangle_instance = m_context->createGeometryInstance();
                        triangle_instance->setGeometryTriangles(tris);
                        triangle_instance->setMaterialCount(
                            (unsigned int)mat_list.size());  // TODO: allow for multiple materials
                        for (int m = 0; m < mat_list.size(); m++) {
                            triangle_instance->setMaterial(m, mat_list[m]);
                        }

                        triangle_instance["vertex_buffer"]->setBuffer(vertex_buffer);
                        triangle_instance["normal_buffer"]->setBuffer(normal_buffer);
                        triangle_instance["texcoord_buffer"]->setBuffer(texcoord_buffer);
                        triangle_instance["index_buffer"]->setBuffer(index_buffer);
                        triangle_instance["material_buffer"]->setBuffer(mat_index_buffer);

                        optix::GeometryGroup triangle_group = m_context->createGeometryGroup();
                        triangle_group->setAcceleration(m_context->createAcceleration("Trbvh"));
                        // triangle_group->getAcceleration()->setProperty("refit", "1");

                        // create a transform that will manipulate the sphere
                        // Transform asset_transform = m_context->createTransform();
                        Transform asset_transform = CreateTransform(asset_rot_mat, asset_pos);

                        // add all the nodes to the graph
                        asset_group->addChild(asset_transform);
                        asset_transform->setChild(triangle_group);
                        triangle_group->addChild(triangle_instance);

                        added_asset_for_body = true;
                    }
                    // } else if (std::shared_ptr<ChEllipsoidShape> ellipsoid_shape =
                    //                std::dynamic_pointer_cast<ChEllipsoidShape>(asset)) {
                    // } else if (std::shared_ptr<ChCylinderShape>
                    // cylinder_shape =
                    //                std::dynamic_pointer_cast<ChCylinderShape>(asset)) {
                    // } else if (std::shared_ptr<ChConeShape> cone_shape =
                    //                std::dynamic_pointer_cast<ChConeShape>(asset)) {
                    // } else if (std::shared_ptr<ChRoundedBoxShape> shape =
                    //                std::dynamic_pointer_cast<ChRoundedBoxShape>(asset)) {
                    // } else if (std::shared_ptr<ChCapsuleShape> capsule_shape =
                    //                std::dynamic_pointer_cast<ChCapsuleShape>(asset)) {
                    // } else if (std::shared_ptr<ChTriangleMeshShape> trimesh_shape =
                    //                std::dynamic_pointer_cast<ChTriangleMeshShape>(asset)) {
                    // } else if (std::shared_ptr<ChPathShape> path_shape =
                    //                std::dynamic_pointer_cast<ChPathShape>(asset)) {
                    // } else if (std::shared_ptr<ChLineShape> line_shape =
                    //                std::dynamic_pointer_cast<ChLineShape>(asset)) {
                    // }
                }
                // check if the asset is a ChColorAsset
                else if (std::shared_ptr<ChColorAsset> visual_asset = std::dynamic_pointer_cast<ChColorAsset>(asset)) {
                    // std::cout << "Asset was color\n";

                }

                // check if the asset is a ChTexture
                else if (std::shared_ptr<ChTexture> visual_asset = std::dynamic_pointer_cast<ChTexture>(asset)) {
                    // std::cout << "Asset was texture\n";
                }
            }

            if (added_asset_for_body) {
                // add node to root and signify need for a rebuild
                m_root->addChild(body_transform);
                m_root->getAcceleration()->markDirty();

                // keep a handle to the body and transform pair for updating
                m_bodies.push_back(std::pair<std::shared_ptr<ChBody>, optix::Transform>(body, body_transform));
            }
        }
    }
}

void ChOptixEngine::UpdateCameraTransforms() {
    for (auto sensor : m_assignedSensor) {
        std::shared_ptr<IRendersWithOptix> pOptixSensor = std::dynamic_pointer_cast<IRendersWithOptix>(sensor);
        if (pOptixSensor) {
            Program ray_gen = pOptixSensor->RayGenProgram();
            ChFrame<double> f_offset = sensor->GetOffsetPose();
            ChFrame<double> f_body = sensor->GetParent()->GetAssetsFrame();

            ChFrame<double> global_loc = f_body * f_offset;
            //
            ray_gen["c_pos"]->setFloat((float)global_loc.GetPos().x(), (float)global_loc.GetPos().y(),
                                       (float)global_loc.GetPos().z());
            ray_gen["c_forward"]->setFloat((float)global_loc.GetA()(0), (float)global_loc.GetA()(3),
                                           (float)global_loc.GetA()(6));  // camera forward
            ray_gen["c_left"]->setFloat((float)global_loc.GetA()(1), (float)global_loc.GetA()(4),
                                        (float)global_loc.GetA()(7));  // camera left
            ray_gen["c_up"]->setFloat((float)global_loc.GetA()(2), (float)global_loc.GetA()(5),
                                      (float)global_loc.GetA()(8));  // camera up

            // ray_gen_program["hFOV"]->setFloat(CH_C_PI / 2.0);                     // camera horizontal field of
            // view ray_gen_program["vFOV"]->setFloat((CH_C_PI / 2.0) * height / width);  // camera horizontal field
            // of view
        }
    }
}

void ChOptixEngine::UpdateBodyTransforms() {
    // update the transforms for objects already in optix
    for (auto bodyPair : m_bodies) {
        const ChVector<double> body_pos = bodyPair.first->GetFrame_REF_to_abs().GetPos();
        const ChMatrix33<double> body_rot_mat = bodyPair.first->GetFrame_REF_to_abs().Amatrix;
        UpdateTransform(bodyPair.second, body_rot_mat, body_pos);
    }

    // mark the transform for rebuild
    m_root->getAcceleration()->markDirty();
}

void ChOptixEngine::UpdateSceneDescription(std::shared_ptr<ChScene> scene) {
    // get the information that we can handle from the scene and set those properties in OptiX

    // create lights -> TODO: move to own function to add more and remove them on the fly
    // PointLight lights[] = {{make_float3(-10.0f, 10.0f, 20.0f), make_float3(1.0f, 1.0f, 1.0f), 50.0, 1},
    //                        {make_float3(10.0f, 10.0f, 20.0f), make_float3(1.0f, 1.0f, 1.0f), 50.0, 1}};
    // PointLight lights[] = {{make_float3(100.0f, -100.0f, 200.0f), make_float3(2.0f, 2.0f, 2.0f), 500.0, 1}};  //,
    //                        {make_float3(10.0f, 10.0f, 20.0f), make_float3(1.0f, 1.0f, 1.0f), 50.0, 1}};

    // PointLight lights[] = {{make_float3(-100.0f, 100.0f, 100.0f), make_float3(1.0f, 1.0f, 1.0f), 300.0, 1},
    //                        {make_float3(-100.0f, -100.0f, 100.0f), make_float3(1.0f, 1.0f, 1.0f), 300.0, 1},
    //                        {make_float3(100.0f, -100.0f, 100.0f), make_float3(1.0f, 1.0f, 1.0f), 300.0, 1},
    //                        {make_float3(100.0f, 100.0f, 100.0f), make_float3(1.0f, 1.0f, 1.0f), 300.0, 1}};

    // if (!m_light_buffer) {
    //     m_light_buffer = m_context->createBuffer(RT_BUFFER_INPUT);
    //     m_light_buffer->setFormat(RT_FORMAT_USER);
    // }

    // we need to check if things have changed before we just go ahead and overwrite!!!!!!!

    m_light_buffer->setElementSize(sizeof(PointLight));
    m_light_buffer->setSize(scene->GetPointLights().size());
    memcpy(m_light_buffer->map(), scene->GetPointLights().data(), scene->GetPointLights().size() * sizeof(PointLight));
    m_light_buffer->unmap();

    m_context["lights"]->set(m_light_buffer);

    if (scene->GetBackground().has_changed) {
        // we need to check if things have changed before we just go ahead and overwrite
        optix::Program miss_program = GetRTProgram(m_context, "miss", "miss_function");
        m_context->setMissProgram(0, miss_program);
        miss_program["default_color"]->setFloat(scene->GetBackground().color.x(), scene->GetBackground().color.y(),
                                                scene->GetBackground().color.z());
        miss_program["default_depth"]->setFloat(-1.f);
        if (scene->GetBackground().has_texture) {
            TextureSampler tex_sampler = CreateTexture(GetChronoDataFile(scene->GetBackground().env_tex));
            miss_program["environment_map"]->setTextureSampler(tex_sampler);
            miss_program["has_environment_map"]->setInt(1);
        } else {
            TextureSampler tex_sampler = CreateTexture();
            miss_program["environment_map"]->setTextureSampler(tex_sampler);
            miss_program["has_environment_map"]->setInt(0);
        }

        scene->GetBackground().has_changed = false;
    }
}

///

Material ChOptixEngine::CreateMaterial() {
    optix::Material mat = m_context->createMaterial();
    Program closest_hit = GetRTProgram(m_context, "basic_shader", "reflective_shader");
    Program shadow_prog = GetRTProgram(m_context, "basic_shader", "hit_shadow");

    mat["Ka"]->setFloat(.2f, .2f, .2f);
    mat["Kd"]->setFloat(.5f, .5f, .5f);
    mat["Ks"]->setFloat(.2f, .2f, .2f);
    mat["phong_exp"]->setFloat(88.f);
    mat["fresnel_exp"]->setFloat(5.f);
    mat["fresnel_min"]->setFloat(0.f, 0.f, 0.f);
    mat["fresnel_max"]->setFloat(1.f, 1.f, 1.f);
    mat["transparency"]->setFloat(1.0);

    TextureSampler tex_sampler = CreateTexture(GetChronoDataFile("redwhite.png"));
    mat["Kd_map"]->setTextureSampler(tex_sampler);
    mat["has_texture"]->setInt(1);

    mat->setClosestHitProgram(0, closest_hit);
    mat->setAnyHitProgram(1, shadow_prog);
    return mat;
}

Material ChOptixEngine::CreateMaterial(std::shared_ptr<ChVisualMaterial> chmat) {
    optix::Material mat = m_context->createMaterial();
    Program closest_hit = GetRTProgram(m_context, "basic_shader", "reflective_shader");
    Program shadow_prog = GetRTProgram(m_context, "basic_shader", "hit_shadow");

    mat["Ka"]->setFloat(chmat->GetAmbientColor().x(), chmat->GetAmbientColor().y(), chmat->GetAmbientColor().z());
    mat["Kd"]->setFloat(chmat->GetDiffuseColor().x(), chmat->GetDiffuseColor().y(), chmat->GetDiffuseColor().z());
    mat["Ks"]->setFloat(chmat->GetSpecularColor().x(), chmat->GetSpecularColor().y(), chmat->GetSpecularColor().z());
    mat["phong_exp"]->setFloat(chmat->GetSpecularExponent());
    mat["fresnel_exp"]->setFloat(chmat->GetFresnelExp());
    mat["fresnel_min"]->setFloat(chmat->GetFresnelMin());
    mat["fresnel_max"]->setFloat(chmat->GetFresnelMax());
    mat["transparency"]->setFloat(chmat->GetTransparency());

    if (chmat->GetKdTexture() != "") {
        TextureSampler tex_sampler = CreateTexture(chmat->GetKdTexture());
        mat["Kd_map"]->setTextureSampler(tex_sampler);
        mat["has_texture"]->setInt(1);
    } else {
        TextureSampler tex_sampler = CreateTexture();
        mat["Kd_map"]->setTextureSampler(tex_sampler);
        mat["has_texture"]->setInt(0);
    }

    mat->setClosestHitProgram(0, closest_hit);
    mat->setAnyHitProgram(1, shadow_prog);
    return mat;
}

TextureSampler ChOptixEngine::CreateTexture(std::string filename) {
    TextureSampler tex_sampler = m_context->createTextureSampler();
    tex_sampler->setWrapMode(0, RT_WRAP_REPEAT);
    tex_sampler->setWrapMode(1, RT_WRAP_REPEAT);
    tex_sampler->setWrapMode(2, RT_WRAP_REPEAT);
    tex_sampler->setIndexingMode(RT_TEXTURE_INDEX_NORMALIZED_COORDINATES);
    tex_sampler->setReadMode(RT_TEXTURE_READ_NORMALIZED_FLOAT);
    tex_sampler->setMaxAnisotropy(1.0f);
    tex_sampler->setMipLevelCount(1u);
    tex_sampler->setArraySize(1u);
    tex_sampler->setFilteringModes(RT_FILTER_LINEAR, RT_FILTER_LINEAR, RT_FILTER_NONE);

    // load texture file
    ByteImageData img = LoadImage(filename);

    // std::cout << "Texture Dims: " << filename << " <" << img.w << ", " << img.h << ", " << img.c << ">\n";

    // if image loading fails, create a default failure texture
    if (img.h == 0 || img.w == 0 || img.c == 0) {
        Buffer buffer = m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4, 1u, 1u);
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        tmp_buffer[0] = 255;
        tmp_buffer[1] = 0;
        tmp_buffer[2] = 255;
        tmp_buffer[3] = 255;
        buffer->unmap();

        tex_sampler->setBuffer(0u, 0u, buffer);

        return tex_sampler;
    }

    Buffer buffer = m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4, img.w, img.h);
    if (img.c == 4) {
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                tmp_buffer[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 1];
                tmp_buffer[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 2];
                tmp_buffer[i * img.w * 4 + j * 4 + 3] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 3];
            }
        }
    } else if (img.c == 3) {
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                tmp_buffer[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w * 3 + j * 3 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w * 3 + j * 3 + 1];
                tmp_buffer[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w * 3 + j * 3 + 2];
                tmp_buffer[i * img.w * 4 + j * 4 + 3] = 255;
            }
        }
    } else if (img.c == 2) {
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                tmp_buffer[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 0];
                tmp_buffer[i * img.w * 4 + j * 4 + 3] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 1];
            }
        }
    } else if (img.c == 1) {
        unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                tmp_buffer[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w + j];
                tmp_buffer[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w + j];
                tmp_buffer[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w + j];
                tmp_buffer[i * img.w * 4 + j * 4 + 3] = 255;
            }
        }
    } else {
        std::cerr << "Error: unsupported number of channels in texture image. Channels=" << img.c << std::endl;
    }

    buffer->unmap();
    tex_sampler->setBuffer(0u, 0u, buffer);
    return tex_sampler;
}

TextureSampler ChOptixEngine::CreateTexture() {
    TextureSampler tex_sampler = m_context->createTextureSampler();
    tex_sampler->setWrapMode(0, RT_WRAP_REPEAT);
    tex_sampler->setWrapMode(1, RT_WRAP_REPEAT);
    tex_sampler->setWrapMode(2, RT_WRAP_REPEAT);
    tex_sampler->setIndexingMode(RT_TEXTURE_INDEX_NORMALIZED_COORDINATES);
    tex_sampler->setReadMode(RT_TEXTURE_READ_NORMALIZED_FLOAT);
    tex_sampler->setMaxAnisotropy(1.0f);
    tex_sampler->setMipLevelCount(1u);
    tex_sampler->setArraySize(1u);
    tex_sampler->setFilteringModes(RT_FILTER_LINEAR, RT_FILTER_LINEAR, RT_FILTER_NONE);

    // create a dummy texture
    Buffer buffer = m_context->createBuffer(RT_BUFFER_INPUT, RT_FORMAT_UNSIGNED_BYTE4, 1u, 1u);
    unsigned char* tmp_buffer = static_cast<unsigned char*>(buffer->map());
    tmp_buffer[0] = 255;
    tmp_buffer[1] = 0;
    tmp_buffer[2] = 255;
    tmp_buffer[3] = 255;
    buffer->unmap();

    tex_sampler->setBuffer(0u, 0u, buffer);

    return tex_sampler;
}

Transform ChOptixEngine::CreateTransform(ChMatrix33<double> a, ChVector<double> b) {
    optix::Transform t = m_context->createTransform();
    // const float t_mat[16] = {(float)a(0), a(1), a(2), b.x(), a(3), a(4), a(5), b.y(),
    //                          a(6),        a(7), a(8), b.z(), 0,    0,    0,    1};
    // const float inv_t_mat[16] = {a(0), a(3), a(6), -b.x(), a(1), a(4), a(7), -b.y(),
    //                              a(2), a(5), a(8), -b.z(), 0,    0,    0,    1};

    UpdateTransform(t, a, b);
    // t->setMatrix(false, t_mat, inv_t_mat);
    return t;
}

void ChOptixEngine::UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector<double> b) {
    const float t_mat[16] = {(float)a(0), (float)a(1),  (float)a(2), (float)b.x(), (float)a(3), (float)a(4),
                             (float)a(5), (float)b.y(), (float)a(6), (float)a(7),  (float)a(8), (float)b.z(),
                             0.f,         0.f,          0.f,         1.f};
    const float inv_t_mat[16] = {
        (float)a(0), (float)a(3), (float)a(6), (float)-b.x(), (float)a(1), (float)a(4), (float)a(7), (float)-b.y(),
        (float)a(2), (float)a(5), (float)a(8), (float)-b.z(), 0.f,         0.f,         0.f,         1.f};
    t->setMatrix(false, t_mat, inv_t_mat);
}

}  // namespace sensor
}  // namespace chrono

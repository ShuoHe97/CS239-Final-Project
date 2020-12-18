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
// Chrono demonstration of a camera sensor
//
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <iomanip>

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChDepthCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
// #include "chrono_sensor/filters/ChFilterGrayscale.h"
// #include "chrono_sensor/filters/ChFilterSaveXYZRGB.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

#include <random>

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

float end_time = 60.0f;
bool save_data = true;

int num_cones = 10;
int num_noncones = 0;
int num_scenes = 20;
int frames_per_scene = 100;

// struct ConePose {
//     float x;
//     float y;
//     float z;
// };

std::vector<ChVector<>> cone_poses;

int frame = 0;
bool save_prepped = false;

bool rand_primed = false;

void create_rng_once() {
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
}

std::default_random_engine generator;
std::uniform_real_distribution<double> distribution;  //(0.0, 1.0);

float rand01() {
    if (!rand_primed) {
        distribution = std::uniform_real_distribution<double>(0.0, 1.0);  //(0.0, 1.0);
        rand_primed = true;
    }
    return distribution(generator);
    // return rand() / (float)RAND_MAX;
}

float min_dist_to_cone(ChVector<> cone1) {
    float min_dist = 1000;
    for (int i = 0; i < cone_poses.size(); i++) {
        if ((cone1 - cone_poses[i]).Length() < min_dist) {
            min_dist = (cone1 - cone_poses[i]).Length();
        }
    }
    return min_dist;
}

void place_cones(ChSystem& mphysicalSystem, int num) {
    auto mmesh = std::make_shared<ChTriangleMeshConnected>();
    mmesh->LoadWavefrontMesh(GetChronoDataFile("sensor/GreenCone.obj"), false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

    auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Cone Mesh");
    trimesh_shape->SetStatic(true);
    cone_poses.clear();

    for (int i = 0; i < num; i++) {
        auto mesh_body = std::make_shared<ChBody>();
        ChVector<> cp = {6 * (rand01() - .5), 6 * (rand01() - .5), .5};
        while (min_dist_to_cone(cp) < .2) {
            cp = {4 * (rand01() - .5), 4 * (rand01() - .5), .5};
        }
        cone_poses.push_back(cp);

        mesh_body->SetPos(cp);
        // mesh_body->SetRot(Q_from_Euler123({CH_C_PI * 2 * rand01(), CH_C_PI * 2 * rand01(), CH_C_PI * 2 * rand01()}));

        mesh_body->AddAsset(trimesh_shape);
        mesh_body->SetBodyFixed(true);
        mphysicalSystem.Add(mesh_body);
    }
}

void place_noncones(ChSystem& mphysicalSystem, int num) {
    for (int i = 0; i < num; i++) {
        auto box = std::make_shared<ChBodyEasyBox>(.2 * rand01() + .001, .2 * rand01() + .001, .2 * rand01() + .001,
                                                   1000, true, true);
        box->SetPos({10 * (rand01() - .5), 10 * (rand01() - .5), rand01() + .2});
        box->SetRot(Q_from_Euler123({rand01() * CH_C_PI * 2, rand01() * CH_C_PI * 2, rand01() * CH_C_PI * 2}));
        box->SetBodyFixed(true);
        mphysicalSystem.Add(box);

        auto sphere = std::make_shared<ChBodyEasySphere>(.1 * rand01() + .001, 1000, true, true);
        sphere->SetPos({10 * (rand01() - .5), 10 * (rand01() - .5), rand01() + .2});
        sphere->SetBodyFixed(true);
        mphysicalSystem.Add(sphere);
    }
}

void save_frame(std::string data_path, int cone_number, std::vector<ChVector<>> poses, LockedXYZRGBBufferPtr& data) {
    utils::CSV_writer csv_data_writer(",");

    for (int i = 0; i < data->Height; i++) {
        for (int j = 0; j < data->Width; j++) {
            PixelXYZRGB pix = data->Buffer[i * data->Width + j];
            csv_data_writer << pix.x << pix.y << pix.z << std::to_string(pix.r) << std::to_string(pix.g)
                            << std::to_string(pix.b) << std::endl;
        }
    }
    csv_data_writer.write_to_file(data_path + "data_" + std::to_string(frame) + ".csv");

    utils::CSV_writer csv_label_writer(",");

    for (auto p : poses) {
        csv_label_writer << p.x() << p.y() << p.z() << std::endl;
    }
    csv_label_writer.write_to_file(data_path + "label_" + std::to_string(frame) + ".csv");

    // // write number of cones
    // csv_writer << "Cones" << std::endl;
    // csv_writer << cone_number << std::endl;
    //
    // // write relative position of cones
    // csv_writer << "Poses" << std::endl;
    // for (auto p : poses) {
    //     csv_writer << p.x() << p.y() << p.z() << std::endl;
    // }
    //
    // // write point cloud
    // csv_writer << "Cloud" << std::endl;
    // for (int i = 0; i < data->Height; i++) {
    //     for (int j = 0; j < data->Width; j++) {
    //         PixelXYZRGB pix = data->Buffer[i * data->Width + j];
    //         csv_writer << pix.x << pix.y << pix.z << std::to_string(pix.r) << std::to_string(pix.g)
    //                    << std::to_string(pix.b) << std::endl;
    //     }
    // }
    //
    // // PixelXYZRGB pix = data->Buffer[0];
    // // std::cout << "Point 0: " << pix.x << "," << pix.y << "," << pix.z << "," << std::to_string(pix.r) << ","
    // //           << std::to_string(pix.g) << "," << std::to_string(pix.b) << std::endl;
    //
    // csv_writer.write_to_file(data_path + "frame_" + std::to_string(frame) + ".csv");
    //
    std::cout << "Saved frame " << frame << " with " << cone_number << " cones\n";
    frame++;
}

int main(int argc, char* argv[]) {
    srand(time(NULL));
    // srand(0);

    std::cout << "Argc: " << argc << "\n";
    if (argc < 2) {
        std::cout << "Run as demo_SEN_zed <name of run>\n";
        return -1;
    }

    std::string save_path = argv[1];

    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // directory for all sensor data
    std::string sensor_data_path = "SENSOR_OUTPUT";
    if (!filesystem::create_directory(filesystem::path(sensor_data_path))) {
        std::cout << "Error creating directory " << sensor_data_path << std::endl;
        return 1;
    }

    std::string data_path = "SENSOR_OUTPUT/" + save_path + "/";
    if (!filesystem::create_directory(filesystem::path(data_path))) {
        std::cout << "Error creating directory " << data_path << std::endl;
        return 1;
    }

    for (int i = 0; i < num_scenes; i++) {
        // -----------------
        // Create the system
        // -----------------
        ChSystemNSC mphysicalSystem;

        auto ground = std::make_shared<ChBodyEasyBox>(40, 40, 1, 1000, true, true);
        ground->SetBodyFixed(true);
        ground->SetPos({0, 0, 0});
        mphysicalSystem.Add(ground);

        // auto wall1 = std::make_shared<ChBodyEasyBox>(1, 40, 40, 1000, true, true);
        // wall1->SetBodyFixed(true);
        // wall1->SetPos({-5, 0, 0});
        // wall1->SetRot(Q_from_Euler123({rand01() - .5, rand01() - .5, rand01() - .5}));
        // mphysicalSystem.Add(wall1);
        //
        // auto wall2 = std::make_shared<ChBodyEasyBox>(1, 40, 40, 1000, true, true);
        // wall2->SetBodyFixed(true);
        // wall2->SetPos({5, 0, 0});
        // wall2->SetRot(Q_from_Euler123({rand01() - .5, rand01() - .5, rand01() - .5}));
        // mphysicalSystem.Add(wall2);
        //
        // auto wall3 = std::make_shared<ChBodyEasyBox>(40, 1, 40, 1000, true, true);
        // wall3->SetBodyFixed(true);
        // wall3->SetPos({0, 5, 0});
        // wall3->SetRot(Q_from_Euler123({rand01() - .5, rand01() - .5, rand01() - .5}));
        // mphysicalSystem.Add(wall3);
        //
        // auto wall4 = std::make_shared<ChBodyEasyBox>(40, 1, 40, 1000, true, true);
        // wall4->SetBodyFixed(true);
        // wall4->SetPos({0, -5, 0});
        // wall4->SetRot(Q_from_Euler123({rand01() - .5, rand01() - .5, rand01() - .5}));
        // mphysicalSystem.Add(wall4);

        auto vis_mat = std::make_shared<ChVisualMaterial>();
        // vis_mat->SetDiffuseColor();
        ground->AddAsset(vis_mat);

        place_cones(mphysicalSystem, num_cones);
        place_noncones(mphysicalSystem, num_noncones);

        // ---------------------------------------
        // add a mesh to be visualized by a camera
        // ---------------------------------------

        // ---------------------------------------
        // create paths for saved data
        // ---------------------------------------

        // -----------------------
        // Create a sensor manager
        // -----------------------
        auto manager = std::make_shared<ChSensorManager>(&mphysicalSystem);
        manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 500);

        // ------------------------------------------------
        // Create a camera and add it to the sensor manager
        // ------------------------------------------------
        auto zed = std::make_shared<ChDepthCameraSensor>(
            ground,                                                              // body camera is attached to
            1000,                                                                // update rate in Hz
            chrono::ChFrame<double>({0, 0, .55}, Q_from_Euler123({0, .2, .5})),  // offset pose
            672,                                                                 // image width
            376,                                                                 // image height
            CH_C_PI / 2,                                                         // Horizontal field of view
            CH_C_PI / 3                                                          // vertical field of View
        );
        zed->SetName("Depth Camera Sensor");

        // --------------------------------------------------------------------
        // Create a filter graph for post-processing the images from the camera
        // --------------------------------------------------------------------

        // we want to visualize this sensor right after rendering, so add the visualize filter to the filter list.
        // zed->FilterList().push_back(std::make_shared<ChFilterVisualizePointCloud>("ZED RGB point cloud data"));
        zed->FilterList().push_back(std::make_shared<ChFilterXYZRGBAccess>());

        // add sensor to the manager
        manager->AddSensor(zed);

        // ----------------
        // Camera for debug
        // ----------------
        auto cam = std::make_shared<ChCameraSensor>(
            ground,                                                              // body camera is attached to
            15,                                                                  // update rate in Hz
            chrono::ChFrame<double>({0, 0, .55}, Q_from_Euler123({0, .2, .5})),  // offset pose
            672,                                                                 // image width
            376,                                                                 // image height
            CH_C_PI / 2,                                                         // Horizontal field of view
            CH_C_PI / 3                                                          // vertical field of View
        );
        cam->SetName("Camera Sensor");
        cam->FilterList().push_back(std::make_shared<ChFilterVisualize>("Debug Camera View"));
        // cam->FilterList().push_back(std::make_shared<ChFilterSave>());

        // add sensor to the manager
        // manager->AddSensor(cam);

        // ---------------
        // Simulate system
        // ---------------
        float orbit_radius = 10.f;
        float orbit_rate = 0.5;
        float ch_time = 0.0;

        double render_time = 0;

        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        int zedupdates = 1;
        int numzedupdates = 0;

        // manager->Update();
        // while (ch_time < end_time) {
        while (numzedupdates < frames_per_scene) {
            // zed->SetOffsetPose(chrono::ChFrame<double>(
            //     {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 1},
            //     Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

            manager->Update();
            mphysicalSystem.DoStepDynamics(0.01);

            auto start = std::chrono::high_resolution_clock::now();
            auto end = start + std::chrono::microseconds(100000);
            do {
                std::this_thread::yield();
            } while (std::chrono::high_resolution_clock::now() < end);

            LockedXYZRGBBufferPtr bufferZed = zed->GetMostRecentBuffer<LockedXYZRGBBufferPtr>();
            if (bufferZed->UpdateCount > zedupdates) {
                // std::cout << "We have a new frame of ZED data\n";
                // a new buffer has been rendered!
                zedupdates = bufferZed->UpdateCount;
                numzedupdates++;
                // std::cout << "Zed update: " << numzedupdates << "\n";
                // int num_cones = 10;
                // std::vector<ConePose> poses;
                int number_of_cones_plausibly_visible = 0;
                std::vector<ChVector<>> visible_cone_poses;

                ChFrame<> zed_frame = zed->GetOffsetPose();
                zed->SetOffsetPose(chrono::ChFrame<double>(
                    {0, 0, .2 * rand01() + .6}, Q_from_Euler123({0, .5 * rand01() + .0, 1 * rand01() * CH_C_PI * 2})));
                cam->SetOffsetPose(zed->GetOffsetPose());

                for (int c = 0; c < num_cones; c++) {
                    bool visible = true;

                    // compute relative position of cone

                    // ChFrame<> zed_frame = zed->GetOffsetPose();

                    ChVector<> rel_pos = zed_frame.TransformParentToLocal(cone_poses[c]);
                    // ChVector<> other_rel_pos = zed_frame.TransformPointParentToLocal(cone_poses[c]);
                    // std::cout << "Dist between two methods: " << (rel_pos - other_rel_pos).Length() << "\n";

                    // std::cout << "Cone Global pos: " << cone_poses[c].x() << ", " << cone_poses[c].y() << ", "
                    //           << cone_poses[c].z() << "\n";
                    // std::cout << "Zed Frame Pos: " << zed_frame.GetPos().x() << ", " << zed_frame.GetPos().y()
                    // << ",
                    // "
                    //           << zed_frame.GetPos().z() << "\n";
                    // std::cout << "Zed Frame Rot: " << Q_to_Euler123(zed_frame.GetRot()).x() << ", "
                    //           << Q_to_Euler123(zed_frame.GetRot()).y() << ", " <<
                    //           Q_to_Euler123(zed_frame.GetRot()).z()
                    //           << "\n";
                    // std::cout << "Cone Relative pos: " << rel_pos.x() << ", " << rel_pos.y() << ", " <<
                    // rel_pos.z()
                    //           << "\n";
                    // Idea is we hopefully don't have too many in edge-case scenarios:
                    // - where we have not approximated field of view well
                    // - where cone is behind another obstacle
                    // see if cone is in viewing window
                    // if (rel_pos.x() < .1 || rel_pos.x() > 10)  // near and far clip
                    //     visible = false;
                    if (abs(rel_pos.y()) > rel_pos.x())  // 90 deg horizontal
                        visible = false;
                    // if (abs(rel_pos.z()) > .577 * rel_pos.x())  // 60 deg vertical
                    //     visible = false;

                    // what do we do about cones obscured by other objects?

                    if (visible) {
                        visible_cone_poses.push_back(rel_pos);
                        number_of_cones_plausibly_visible++;
                    }
                }

                if (save_data)
                    save_frame(data_path, number_of_cones_plausibly_visible, visible_cone_poses, bufferZed);
            }

            ch_time = (float)mphysicalSystem.GetChTime();
        }
    }
    // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}

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
// utils file translating Chrono assets to higher quality assets
//
// =============================================================================

#include "chrono_sensor/utils/ChVisualMaterialUtils.h"
#include <iostream>
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {
namespace sensor {

void CreateModernMeshAssets(std::shared_ptr<ChTriangleMeshShape> mesh_shape) {
    if (mesh_shape->GetMesh()->GetFileName() == "") {
        return;
    }

    // tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    // std::vector<tinyobj::material_t> materials;

    std::string mtl_base = "";
    std::string file_name = mesh_shape->GetMesh()->GetFileName();

    // file_name = "../test_tex.obj";

    // std::cout << "Mesh to load: " << file_name << std::endl;

    // std::string err = tinyobj::LoadObj(shapes, "test_tex.obj", "");
    // std::string err = tinyobj::LoadObj(shapes, "test_monkey.obj", "");
    // std::string err = tinyobj::LoadObj(shapes, "test.obj", "");
    // std::string err = tinyobj::LoadObj(shapes, "test_sphere.obj", "");
    // std::string err = tinyobj::LoadObj(shapes, "test_cube.obj", "");

#ifdef _WIN32
    int slash_location = (int)file_name.rfind('\\');
#else
    int slash_location = file_name.rfind('/');
    // std::cout << "Slash location: " << slash_location << std::endl;
#endif
    if (std::string::npos != slash_location) {
        mtl_base = file_name.substr(0, slash_location + 1);
    }

    // std::cout << "Loading: " << mtl_base << std::endl;
    std::string err = tinyobj::LoadObj(shapes, file_name.c_str(), mtl_base.c_str());

    // std::string err = tinyobj::LoadObj(shapes, "test_tex.obj", "");
    // std::string err = tinyobj::LoadObj(shapes, "test_monkey.obj", "");
    // std::string err = tinyobj::LoadObj(shapes, "test.obj", "");
    // std::string err = tinyobj::LoadObj(shapes, "test_sphere.obj", "");
    // std::string err = tinyobj::LoadObj(shapes, "test_cube.obj", "");

    // std::cout << "Obj load result: " << err << std::endl;
    // std::cout << " # of shapes in OBJ file: " << shapes.size() << std::endl;

    // go through each shape and add the material as an asset. Also add the material id list to the ChVisualization
    // asset along with a list of triangle-to-face id list to mesh

    std::vector<std::shared_ptr<ChVisualMaterial>> material_list = std::vector<std::shared_ptr<ChVisualMaterial>>();

    std::vector<ChVector<double>> vertex_buffer = std::vector<ChVector<double>>();
    std::vector<ChVector<double>> normal_buffer = std::vector<ChVector<double>>();
    std::vector<ChVector<double>> tex_coords = std::vector<ChVector<double>>();

    std::vector<ChVector<int>> index_buffer = std::vector<ChVector<int>>();
    std::vector<ChVector<int>> material_indices = std::vector<ChVector<int>>();

    unsigned int previous_vertices = 0;

    for (int i = 0; i < shapes.size(); i++) {
        std::shared_ptr<ChVisualMaterial> mat = std::make_shared<ChVisualMaterial>();
        mat->SetAmbientColor(
            {shapes[i].material.ambient[0], shapes[i].material.ambient[1], shapes[i].material.ambient[2]});
        mat->SetDiffuseColor(
            {shapes[i].material.diffuse[0], shapes[i].material.diffuse[1], shapes[i].material.diffuse[2]});
        mat->SetSpecularColor(
            {shapes[i].material.specular[0], shapes[i].material.specular[1], shapes[i].material.specular[2]});
        mat->SetTransparency(shapes[i].material.dissolve);
        if (shapes[i].material.diffuse_texname != "") {
            mat->SetKdTexture(mtl_base + shapes[i].material.diffuse_texname);
            // std::cout << "OBJ has texture: " << shapes[i].material.diffuse_texname << std::endl;
        }

        // std::cout << "Texture name: " << shapes[i].material.diffuse_texname << std::endl;

        // std::cout << "Material directly from tinyobj: \n";
        // std::cout << "Ambient: " << shapes[i].material.ambient[0] << ", " << shapes[i].material.ambient[1] << ", "
        //           << shapes[i].material.ambient[2] << std::endl;
        // std::cout << "Diffuse: " << shapes[i].material.diffuse[0] << ", " << shapes[i].material.diffuse[1] << ", "
        //           << shapes[i].material.diffuse[2] << std::endl;
        // std::cout << "Specular: " << shapes[i].material.specular[0] << ", " << shapes[i].material.specular[1] << ", "
        //           << shapes[i].material.specular[2] << std::endl;

        // let's redo all the indices so we can gaurantee everything is consistent
        // std::cout << "Vertices: " << shapes[i].mesh.positions.size() << std::endl;
        // std::cout << "Normals: " << shapes[i].mesh.normals.size() << std::endl;
        // std::cout << "Texcoords: " << shapes[i].mesh.texcoords.size() << std::endl;
        // std::cout << "Indices: " << shapes[i].mesh.indices.size() << std::endl;

        // vertix coordinates
        for (int j = 0; j < shapes[i].mesh.positions.size() / 3; j++) {
            vertex_buffer.push_back({shapes[i].mesh.positions[3 * j + 0], shapes[i].mesh.positions[3 * j + 1],
                                     shapes[i].mesh.positions[3 * j + 2]});
        }

        // normal coorinates
        for (int j = 0; j < shapes[i].mesh.normals.size() / 3; j++) {
            normal_buffer.push_back({shapes[i].mesh.normals[3 * j + 0], shapes[i].mesh.normals[3 * j + 1],
                                     shapes[i].mesh.normals[3 * j + 2]});
        }

        // texture coordinates
        for (int j = 0; j < shapes[i].mesh.texcoords.size() / 2; j++) {
            tex_coords.push_back({shapes[i].mesh.texcoords[2 * j + 0], shapes[i].mesh.texcoords[2 * j + 1], 0});
        }

        for (int j = 0; j < shapes[i].mesh.indices.size() / 3; j++) {
            index_buffer.push_back({(int)(shapes[i].mesh.indices[3 * j + 0] + previous_vertices),
                                    (int)(shapes[i].mesh.indices[3 * j + 1] + previous_vertices),
                                    (int)(shapes[i].mesh.indices[3 * j + 2] + previous_vertices)});

            material_indices.push_back({i, 0, 0});
        }

        previous_vertices = (unsigned int)vertex_buffer.size();
        // body->AddAsset(mat);
        material_list.push_back(mat);
    }

    mesh_shape->GetMesh()->m_vertices = vertex_buffer;
    mesh_shape->GetMesh()->m_normals = normal_buffer;
    mesh_shape->GetMesh()->m_UV = tex_coords;
    mesh_shape->GetMesh()->m_face_v_indices = index_buffer;
    mesh_shape->GetMesh()->m_face_col_indices = material_indices;

    // std::cout << "Mat indices: \n";
    // for (int i = 0; i < material_indices.size(); i++) {
    //     std::cout << i << ": " << material_indices[i].x() << std::endl;
    // }

    // std::cout << "Vertices: \n";
    // for (int i = 0; i < vertex_buffer.size(); i++) {
    //     std::cout << i << ": " << vertex_buffer[i].x() << " | " << vertex_buffer[i].y() << " | " <<
    //     vertex_buffer[i].z()
    //               << std::endl;
    // }
    //
    // std::cout << "Face Indices: \n";
    // for (int i = 0; i < index_buffer.size(); i++) {
    //     std::cout << i << ": " << index_buffer[i].x() << " | " << index_buffer[i].y() << " | " <<
    //     index_buffer[i].z()
    //               << std::endl;
    // }
    //
    // std::cout << "num faces: " << index_buffer.size() << std::endl;

    mesh_shape->material_list = material_list;
}

void ConvertToModernAssets(std::shared_ptr<ChBody> body) {
    if (body->GetAssets().size() > 0) {
        // iterate through all assets in the body
        // std::cout << "Number of assets: " << body->GetAssets().size() << std::endl;
        for (auto asset : body->GetAssets()) {
            // check if the asset is a ChVisualization
            if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)) {
                // collect relative position and orientation of the asset

                if (std::shared_ptr<ChTriangleMeshShape> trimesh_shape =
                        std::dynamic_pointer_cast<ChTriangleMeshShape>(asset)) {
                    CreateModernMeshAssets(trimesh_shape);
                }

            }
            // check if the asset is a ChColorAsset
            else if (std::shared_ptr<ChColorAsset> visual_asset = std::dynamic_pointer_cast<ChColorAsset>(asset)) {
                // std::cout << "Asset was color\n";
                // CreateVisualMaterial(body, visual_asset);
            }

            // check if the asset is a ChTexture
            else if (std::shared_ptr<ChTexture> visual_asset = std::dynamic_pointer_cast<ChTexture>(asset)) {
                // std::cout << "Asset was texture\n";
            }
        }
    }
}

void ConvertToModernAssets(ChSystem* sys) {
    for (auto body : sys->Get_bodylist()) {
        ConvertToModernAssets(body);
    }
}

}  // namespace sensor
}  // namespace chrono

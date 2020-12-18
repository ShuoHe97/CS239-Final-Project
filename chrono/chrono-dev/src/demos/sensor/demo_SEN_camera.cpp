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
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

float end_time = 20.0f;
bool save_data = false;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------
    auto mmesh = std::make_shared<ChTriangleMeshConnected>();
    mmesh->LoadWavefrontMesh(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

    auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("HMMWV Chassis Mesh");
    trimesh_shape->SetStatic(true);

    auto mesh_body = std::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddAsset(trimesh_shape);
    mesh_body->SetBodyFixed(true);
    mphysicalSystem.Add(mesh_body);

    // ---------------------------------------
    // create paths for saved data
    // ---------------------------------------

    // directory for all sensor data
    std::string sensor_data_path = "SENSOR_OUTPUT";
    if (!filesystem::create_directory(filesystem::path(sensor_data_path))) {
        std::cout << "Error creating directory " << sensor_data_path << std::endl;
        return 1;
    }

    std::string color_data_path = "SENSOR_OUTPUT/cam_color/";
    if (!filesystem::create_directory(filesystem::path(color_data_path))) {
        std::cout << "Error creating directory " << color_data_path << std::endl;
        return 1;
    }

    std::string gray_data_path = "SENSOR_OUTPUT/cam_gray/";
    if (!filesystem::create_directory(filesystem::path(gray_data_path))) {
        std::cout << "Error creating directory " << gray_data_path << std::endl;
        return 1;
    }

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = std::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    auto cam = std::make_shared<ChCameraSensor>(
        mesh_body,                                                          // body camera is attached to
        50,                                                                 // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1280,                                                               // image width
        720,                                                                // image height
        CH_C_PI / 3, 9. / 16. * CH_C_PI / 3);                               // FOV
    cam->SetName("Camera Sensor");

    // --------------------------------------------------------------------
    // Create a filter graph for post-processing the images from the camera
    // --------------------------------------------------------------------

    // we want to visualize this sensor right after rendering, so add the visualize filter to the filter list.
    cam->FilterList().push_back(std::make_shared<ChFilterVisualize>("Before Grayscale Filter"));

    // we want to have access to this RGBA8 buffer on the host.
    cam->FilterList().push_back(std::make_shared<ChFilterRGBA8Access>());

    if (save_data)
        // we want to save the RGBA buffer to png
        cam->FilterList().push_back(std::make_shared<ChFilterSave>(color_data_path));

    // filter the sensor to grayscale
    cam->FilterList().push_back(std::make_shared<ChFilterGrayscale>());

    // we want to visualize this sensor after grayscale, so add the visualize filter to the filter list.
    cam->FilterList().push_back(std::make_shared<ChFilterVisualize>("Final Visualization"));

    if (save_data)
        // we want to save the grayscale buffer to png
        cam->FilterList().push_back(std::make_shared<ChFilterSave>(gray_data_path));

    // we also want to have access to this grayscale buffer on the host.
    cam->FilterList().push_back(std::make_shared<ChFilterR8Access>());

    // add sensor to the manager
    manager->AddSensor(cam);

    // ---------------
    // Simulate system
    // ---------------
    float orbit_radius = 10.f;
    float orbit_rate = 0.5;
    float ch_time = 0.0;

    double render_time = 0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        cam->SetOffsetPose(chrono::ChFrame<double>(
            {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 1},
            Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

        manager->Update();
        mphysicalSystem.DoStepDynamics(0.001);

        ch_time = (float)mphysicalSystem.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}

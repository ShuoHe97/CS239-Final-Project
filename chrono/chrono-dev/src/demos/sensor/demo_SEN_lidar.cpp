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
// Chrono demonstration of a lidar sensor
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

#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

float end_time = 100.0f;
bool save_data = false;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;

    // ----------------------------------
    // add a mesh to be sensed by a lidar
    // ----------------------------------
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

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = std::make_shared<ChSensorManager>(&mphysicalSystem);

    // ------------------------------------------------
    // Create a lidar and add it to the sensor manager
    // ------------------------------------------------
    auto lidar = std::make_shared<ChLidarSensor>(
        mesh_body,                                                          // body lidar is attached to
        10,                                                                 // scanning rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1000,                                                               // number of horizontal samples
        50,                                                                 // number of vertical channels
        CH_C_PI,                                                            // horizontal field of view
        CH_C_PI / 6.                                                        // vertical field of view
    );
    lidar->SetName("Lidar Sensor");
    manager->AddSensor(lidar);

    // -----------------------------------------------------------------
    // Create a filter graph for post-processing the data from the lidar
    // -----------------------------------------------------------------
    lidar->FilterList().push_back(std::make_shared<ChFilterVisualize>("Raw Lidar Depth Data"));
    lidar->FilterList().push_back(std::make_shared<ChFilterDIAccess>());
    lidar->FilterList().push_back(std::make_shared<ChFilterPCfromDepth>());
    lidar->FilterList().push_back(std::make_shared<ChFilterVisualizePointCloud>("Lidar Point Cloud"));
    lidar->FilterList().push_back(std::make_shared<ChFilterXYZIAccess>());

    // ---------------
    // Simulate system
    // ---------------
    float orbit_radius = 5.f;
    float orbit_rate = 0.2;
    float ch_time = 0.0;

    double render_time = 0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        lidar->SetOffsetPose(chrono::ChFrame<double>(
            {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 1},
            Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

        manager->Update();
        mphysicalSystem.DoStepDynamics(0.001);

        LockedDIBufferPtr lidar_data = std::move(lidar->GetMostRecentBuffer<LockedDIBufferPtr>());
        float min_val = 1000;
        for (int i = 0; i < lidar_data->Height; i++) {
            for (int j = 0; j < lidar_data->Width; j++) {
                if (lidar_data->Buffer[i * lidar_data->Width + j].range < min_val)
                    min_val = lidar_data->Buffer[i * lidar_data->Width + j].range;
            }
        }
        // std::cout << "Min range: " << min_val << std::endl;

        ch_time = (float)mphysicalSystem.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}

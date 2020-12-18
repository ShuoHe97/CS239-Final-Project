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
// Chrono demonstration for testing the chrono sensor module
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
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChGPSSensor.h"
#include "chrono_sensor/ChIMUSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

int num_cameras = 2 - 1;
int num_bodies = 100;
int num_groups = 1;

bool run_chrono = true;
float time_step = 0.02f;
float end_time = 60.0f;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;
    mphysicalSystem.Set_G_acc({0, 0, -9.81});

    auto floor = std::make_shared<ChBodyEasyBox>(100, 100, 1,  // x,y,z size
                                                 100,          // density
                                                 true,         // collide enable?
                                                 true);        // visualization?
    floor->SetPos({0, 0, -1.0});
    floor->SetRot(Q_from_AngZ(CH_C_PI / 2.0));
    floor->SetBodyFixed(true);

    mphysicalSystem.Add(floor);

    // add a mesh
    auto mmesh = std::make_shared<ChTriangleMeshConnected>();
    mmesh->LoadWavefrontMesh(GetChronoDataFile("shoe_view.obj"), false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    mmesh->RepairDuplicateVertexes(1e-9);

    double mmass;
    ChVector<> mcog;
    ChMatrix33<> minertia;
    double mdensity = 1000;
    mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    std::shared_ptr<ChBody> imu_parent;
    std::shared_ptr<ChBody> gps_parent;

    auto wall1 = std::make_shared<ChBodyEasyBox>(40.0, .1, 10.0, 100, true, true);
    wall1->SetPos({0, -20, 4});
    wall1->SetBodyFixed(true);
    mphysicalSystem.Add(wall1);

    auto wall2 = std::make_shared<ChBodyEasyBox>(40.0, .1, 10.0, 100, true, true);
    wall2->SetPos({0, 20, 4});
    wall2->SetBodyFixed(true);
    mphysicalSystem.Add(wall2);

    auto wall3 = std::make_shared<ChBodyEasyBox>(.1, 40.0, 10.0, 100, true, true);
    wall3->SetPos({-20, 0, 4});
    wall3->SetBodyFixed(true);
    mphysicalSystem.Add(wall3);

    auto wall4 = std::make_shared<ChBodyEasyBox>(.1, 40.0, 10.0, 100, true, true);
    wall4->SetPos({20, 0, 4});
    wall4->SetBodyFixed(true);
    mphysicalSystem.Add(wall4);

    for (int i = 0; i < num_bodies; i++) {
        // add a box
        auto box = std::make_shared<ChBodyEasyBox>((float)ChRandom() / 2.0 + 0.1, (float)ChRandom() / 2.0 + 0.1,
                                                   (float)ChRandom() / 2.0 + 0.1,  // x,y,z size
                                                   100,                            // density
                                                   true,                           // collide enable?
                                                   true);                          // visualization?
        box->SetPos({(float)ChRandom(), (float)ChRandom(), 2.0 + i});
        box->SetRot(Q_from_Euler123({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()}));
        mphysicalSystem.Add(box);

        if (!imu_parent) {
            imu_parent = box;
        }

        auto sphere = std::make_shared<ChBodyEasySphere>((float)ChRandom() / 2.0 + 0.1,  // radius
                                                         100,                            // density
                                                         true,                           // collide enable?
                                                         true);                          // visualization?
        sphere->SetPos({(float)ChRandom(), (float)ChRandom(), 2.0 + i});
        // sphere->SetRot(Q_from_Euler123({(float)ChRandom(), (float)ChRandom(), (float)rand() /
        // RAND_MAX}));
        mphysicalSystem.Add(sphere);
        if (!gps_parent) {
            gps_parent = sphere;
        }

        auto sphere_asset = sphere->GetAssets()[0];
        if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(sphere_asset)) {
            auto vis_mat = std::make_shared<ChVisualMaterial>();
            vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
            vis_mat->SetDiffuseColor({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()});
            vis_mat->SetSpecularColor({.2f, .2f, .2f});

            visual_asset->material_list.push_back(vis_mat);
        }
    }

    // directory for all sensor data
    std::string sensor_data_path = "SENSOR_OUTPUT";
    if (!filesystem::create_directory(filesystem::path(sensor_data_path))) {
        std::cout << "Error creating directory " << sensor_data_path << std::endl;
        return 1;
    }

    auto manager = std::make_shared<ChSensorManager>(&mphysicalSystem);
    // manager->Initialize();
    manager->SetAllowableGroups(num_groups);  // THIS NEEDS MORE PERFORMANCE TESTING

    // make some changes/additions to the scene
    manager->scene->AddPointLight({10, 10, 10}, {1, 1, 1}, 100);
    manager->scene->AddPointLight({1, 1, 3}, {0, 0, 1}, 100);
    std::vector<PointLight>& lights = manager->scene->GetPointLights();

    // manager->scene->AddPointLight({-10, -10, 10}, {1, 1, 1}, 50);

    auto cam = std::make_shared<ChCameraSensor>(
        floor,                                                              // body camera is attached to
        15,                                                                 // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose
        1280,                                                               // image width
        720,                                                                // image height
        CH_C_PI / 3, 9 / 16. * CH_C_PI / 3);

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

    cam->SetName("Camera Sensor 0");
    // we want to visualize this sensor right after rendering, so add the visualize filter to the filter list.
    cam->FilterList().push_back(std::make_shared<ChFilterVisualize>("Before Grayscale Filter"));
    // cam->FilterList().push_back(std::make_shared<ChFilterSave>(color_data_path));

    // we want to have access to this RGBA8 buffer on the host.
    cam->FilterList().push_back(std::make_shared<ChFilterRGBA8Access>());

    // we want to save the RGBA buffer to png
    cam->FilterList().push_back(std::make_shared<ChFilterSave>(color_data_path));

    // filter the sensor to grayscale
    cam->FilterList().push_back(std::make_shared<ChFilterGrayscale>());

    // we want to visualize this sensor after grayscale, so add the visualize filter to the filter list.
    cam->FilterList().push_back(std::make_shared<ChFilterVisualize>("Final Visualization"));

    // we want to save the grayscale buffer to png
    cam->FilterList().push_back(std::make_shared<ChFilterSave>(gray_data_path));

    // we also want to have access to this grayscale buffer on the host.
    cam->FilterList().push_back(std::make_shared<ChFilterR8Access>());

    // add sensor to the manager
    manager->AddSensor(cam);

    // add a lidar to the floor facing the falling objects
    auto lidar = std::make_shared<ChLidarSensor>(
        floor,                                                              // body to which the IMU is attached
        10,                                                                 // update rate
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose from body
        923,                                                                // horizontal samples
        24,                                                                 // vertical samples/channels
        (float)2.0f * (float)CH_C_PI / 3.0f,                                // horizontal field of view
        (float)CH_C_PI / 8.f);                                              // vertical field of view
    lidar->SetName("Lidar Sensor");
    lidar->FilterList().push_back(std::make_shared<ChFilterVisualize>("Lidar Data"));
    lidar->FilterList().push_back(std::make_shared<ChFilterPCfromDepth>());
    lidar->FilterList().push_back(std::make_shared<ChFilterVisualizePointCloud>("Lidar Point Cloud"));
    lidar->FilterList().push_back(std::make_shared<ChFilterXYZIAccess>());
    // lidar->FilterList().push_back(std::make_shared<ChFilterXYZIAccess>());
    manager->AddSensor(lidar);

    // add an IMU sensor to one of the boxes
    auto imu = std::make_shared<ChIMUSensor>(
        imu_parent,                                                         // body to which the IMU is attached
        10,                                                                 // update rate
        chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0})));  // offset pose from body
    imu->SetName("IMU");
    imu->FilterList().push_back(std::make_shared<ChFilterIMUAccess>());
    manager->AddSensor(imu);

    // add an IMU sensor to one of the boxes
    auto gps = std::make_shared<ChGPSSensor>(
        gps_parent,                                                         // body to which the GPS is attached
        10,                                                                 // update rate
        chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0})));  // offset pose from body
    gps->SetName("GPS");
    gps->FilterList().push_back(std::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps);

    // bookkeeping for testing CopyToDevice filter on all the cameras being set up below.
    std::vector<unsigned int> lastUpdateCounts(num_cameras);

    std::vector<std::shared_ptr<ChCameraSensor>> cams;
    for (int i = 0; i < num_cameras; i++) {
        auto cam1 = std::make_shared<ChCameraSensor>(
            floor,                                                              // body camera is attached to
                                                                                // 50 + 10 * (i % 2),
            10 + 10 * (i % 4),                                                  // 30 + i, // update rate in Hz
            chrono::ChFrame<double>({-3, 0, 2}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose
            1280,                                                               // image width
            720,                                                                // image height
            CH_C_PI / 3, 9 / 16. * CH_C_PI / 3);
        cams.push_back(cam1);

        std::stringstream nm;
        nm << "Camera Sensor " << i + 1;
        cam1->SetName(nm.str());
        // we want to visualize this sensor, so add the visualize filter to the filter list.
        cam1->FilterList().push_back(std::make_shared<ChFilterVisualize>("Before Grayscale Filter"));

        // at the end, we want to have access to this RGBA8 buffer on the Device (GPU)...just for testing.
        // cam1->FilterList().push_back(std::make_shared<ChFilterRGBA8CopyToDevice>());

        // filter the sensor to grayscale
        cam1->FilterList().push_back(std::make_shared<ChFilterGrayscale>());

        // we want to visualize this sensor after grayscale, so add the visualize filter to the filter list.
        cam1->FilterList().push_back(std::make_shared<ChFilterVisualize>("After Grayscale Filter"));

        // cam1->FilterList().push_back(std::make_shared<ChFilterSave>("SENSOR_OUTPUT/cam1/"));

        lastUpdateCounts[i] = 0;

        // add sensor to the manager
        manager->AddSensor(cam1);
    }
    std::cout << "Sensor manager using: " << manager->GetNumEngines() << " render groups\n";

    // ---------------
    // Simulate system
    // ---------------
    float orbit_radius = 15.f;
    float orbit_rate = 0.2;
    float ch_time = 0.0;

    double render_time = 0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    unsigned int lastCamBufferUpdate = 0;
    unsigned int lastCamGrayBufferUpdate = 0;
    unsigned int lastIMUBufferUpdate = 0;
    unsigned int lastGPSBufferUpdate = 0;

    float light_change = -.02;

    while (ch_time < end_time) {
        std::chrono::high_resolution_clock::time_point r0 = std::chrono::high_resolution_clock::now();
        cam->SetOffsetPose(chrono::ChFrame<double>(
            {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 3},
            Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

        // PointLight p;
        lights[0].pos = {-orbit_radius * cos(ch_time * orbit_rate * 2), -orbit_radius * sin(ch_time * orbit_rate * 2),
                         10};

        lights[1].color += make_float3(0, 0, light_change);
        if (lights[1].color.z < 0) {
            lights[1].color = make_float3(0, 0, 0);
            light_change = -light_change;
        }
        if (lights[1].color.z > 1) {
            lights[1].color = make_float3(0, 0, 1);
            light_change = -light_change;
        }

        // lights[0].color = {1, 1, 1};
        // lights[0].max_range = 100;
        // lights[0].casts_shadow = 1;

        manager->Update();
        std::chrono::high_resolution_clock::time_point r1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> t_render = std::chrono::duration_cast<std::chrono::duration<double>>(r1 - r0);
        render_time += t_render.count();

        mphysicalSystem.DoStepDynamics(time_step);

        // // check if our host filter for cam has been updated.
        // LockedRGBA8BufferPtr buffer = cam->GetMostRecentBuffer<LockedRGBA8BufferPtr>();
        // if (buffer->UpdateCount > lastCamBufferUpdate) {
        //     // a new buffer has been rendered!
        //     std::cout << "Host Buffer number " << buffer->UpdateCount << " for sensor '" << cam->GetName()
        //               << "' has been rendered at chrono time: " << std::fixed << std::setprecision(3) << ch_time
        //               << std::endl;
        //     lastCamBufferUpdate = buffer->UpdateCount;
        // }
        // // note that 'buffer' will go out of scope with end of the while loop iteration, unlocking the buffer,
        // // allowing
        // // the next render operation to re-fill it.
        //
        // // check if our host filter for cam has been updated.
        // LockedR8BufferPtr bufferGray = cam->GetMostRecentBuffer<LockedR8BufferPtr>();
        // if (bufferGray->UpdateCount > lastCamGrayBufferUpdate) {
        //     // a new buffer has been rendered!
        //     std::cout << "Host Gray Buffer number " << buffer->UpdateCount << " for sensor '" << cam->GetName()
        //               << "' has been rendered at chrono time: " << std::fixed << std::setprecision(3) << ch_time
        //               << std::endl;
        //     lastCamGrayBufferUpdate = bufferGray->UpdateCount;
        // }
        // // note that 'buffer' will go out of scope with end of the while loop iteration, unlocking the buffer,
        // // allowing
        // // the next render operation to re-fill it.
        //
        // // check if our host filter for imu has been updated.
        // LockedIMUBufferPtr bufferIMU = imu->GetMostRecentBuffer<LockedIMUBufferPtr>();
        // if (bufferIMU->UpdateCount > lastIMUBufferUpdate) {
        //     // a new buffer has been rendered!
        //     std::cout << "IMU Buffer number " << bufferIMU->UpdateCount << " for sensor '" << imu->GetName()
        //               << "' has been updated at chrono time: " << std::fixed << std::setprecision(3) << ch_time
        //               << std::endl;
        //     std::cout << " Acc X: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Accel[0]
        //               << ", Acc Y: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Accel[1]
        //               << ", Acc Z: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Accel[2]
        //               << ", Roll: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Roll
        //               << ", Pitch: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Pitch
        //               << ", Yaw: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Yaw << std::endl;
        //     lastIMUBufferUpdate = bufferIMU->UpdateCount;
        // }
        //
        // LockedSensorHostGPSBufferPtr bufferGPS = gps->GetMostRecentBuffer<LockedSensorHostGPSBufferPtr>();
        // if (bufferGPS->UpdateCount > lastGPSBufferUpdate) {
        //     // a new buffer has been rendered!
        //     std::cout << "GPS Buffer number " << bufferGPS->UpdateCount << " for sensor '" << gps->GetName()
        //               << "' has been updated at chrono time: " << std::fixed << std::setprecision(3) << ch_time
        //               << std::endl;
        //     std::cout << " Lat: " << std::fixed << std::setprecision(3) << bufferGPS->Buffer[0].Latitude
        //               << ", Long: " << std::fixed << std::setprecision(3) << bufferGPS->Buffer[0].Longitude
        //               << ", Alt: " << std::fixed << std::setprecision(3) << bufferGPS->Buffer[0].Altitude
        //               << ", Time: " << std::fixed << std::setprecision(3) << bufferGPS->Buffer[0].Time << std::endl;
        //     lastGPSBufferUpdate = bufferGPS->UpdateCount;
        // }

        // note that 'buffer' will go out of scope with end of the while loop iteration, unlocking the buffer, allowing
        // the next render operation to re-fill it.

        ch_time = (float)mphysicalSystem.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";
    std::cout << "Time spent in Chrono: " << wall_time.count() - render_time
              << ", extra time due to rendering: " << render_time << std::endl;

    return 0;
}

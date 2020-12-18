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
// Chrono demonstration of a GPS and an IMU sensor
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

#include "chrono_sensor/ChGPSSensor.h"
#include "chrono_sensor/ChIMUSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
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
    mphysicalSystem.Set_G_acc({0, 0, -9.81});

    // ----------------------------------
    // add double pendulum
    // ----------------------------------
    auto base = std::make_shared<ChBodyEasyBox>(.2, .2, 1, 1000, false, true);
    base->SetPos(ChVector<>(-.2, 0, .5));
    base->SetBodyFixed(true);  // the truss does not move!
    mphysicalSystem.Add(base);

    auto pendulum_leg_1 = std::make_shared<ChBodyEasyBox>(.2, .4, .2, 1000, false, true);
    pendulum_leg_1->SetPos(ChVector<>(0, .2, 1));
    mphysicalSystem.Add(pendulum_leg_1);

    auto pendulum_leg_2 = std::make_shared<ChBodyEasyBox>(.2, .4, .2, 1000, false, true);
    pendulum_leg_2->SetPos(ChVector<>(0, .6, 1));
    mphysicalSystem.Add(pendulum_leg_2);

    auto link1 = std::make_shared<ChLinkLockRevolute>();
    link1->Initialize(base, pendulum_leg_1, ChCoordsys<>({0, 0, 1}, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
    mphysicalSystem.AddLink(link1);

    auto link2 = std::make_shared<ChLinkLockRevolute>();
    link2->Initialize(pendulum_leg_1, pendulum_leg_2,
                      ChCoordsys<>({0, .4, 1}, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
    mphysicalSystem.AddLink(link2);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = std::make_shared<ChSensorManager>(&mphysicalSystem);

    // ------------------------------------------------
    // Create a GPS and add it to the sensor manager
    // ------------------------------------------------
    auto imu = std::make_shared<ChIMUSensor>(
        pendulum_leg_2,                                                     // body to which the IMU is attached
        10,                                                                 // update rate
        chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0})));  // offset pose from body
    imu->SetName("IMU");
    imu->FilterList().push_back(std::make_shared<ChFilterIMUAccess>());
    manager->AddSensor(imu);

    // add an IMU sensor to one of the boxes
    auto gps = std::make_shared<ChGPSSensor>(
        pendulum_leg_2,                                                     // body to which the GPS is attached
        10,                                                                 // update rate
        chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0})));  // offset pose from body
    gps->SetName("GPS");
    gps->FilterList().push_back(std::make_shared<ChFilterGPSAccess>());
    manager->AddSensor(gps);

    // ---------------
    // Simulate system
    // ---------------
    float orbit_radius = 10.f;
    float orbit_rate = 0.5;
    float ch_time = 0.0;

    double render_time = 0;

    unsigned int lastIMUBufferUpdate = 0;
    unsigned int lastGPSBufferUpdate = 0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        manager->Update();
        mphysicalSystem.DoStepDynamics(0.001);

        LockedIMUBufferPtr bufferIMU = imu->GetMostRecentBuffer<LockedIMUBufferPtr>();
        if (bufferIMU->UpdateCount > lastIMUBufferUpdate) {
            std::cout << "Acc X: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Accel[0]
                      << ", Acc Y: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Accel[1]
                      << ", Acc Z: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Accel[2]
                      << ", Roll: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Roll
                      << ", Pitch: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Pitch
                      << ", Yaw: " << std::fixed << std::setprecision(3) << bufferIMU->Buffer[0].Yaw << std::endl;
            lastIMUBufferUpdate = bufferIMU->UpdateCount;
        }

        LockedGPSBufferPtr bufferGPS = gps->GetMostRecentBuffer<LockedGPSBufferPtr>();
        if (bufferGPS->UpdateCount > lastGPSBufferUpdate) {
            std::cout << "Lat: " << std::fixed << std::setprecision(3) << bufferGPS->Buffer[0].Latitude
                      << ", Long: " << std::fixed << std::setprecision(3) << bufferGPS->Buffer[0].Longitude
                      << ", Alt: " << std::fixed << std::setprecision(3) << bufferGPS->Buffer[0].Altitude
                      << ", Time: " << std::fixed << std::setprecision(3) << bufferGPS->Buffer[0].Time << std::endl;
            lastGPSBufferUpdate = bufferGPS->UpdateCount;
        }

        ch_time = (float)mphysicalSystem.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}

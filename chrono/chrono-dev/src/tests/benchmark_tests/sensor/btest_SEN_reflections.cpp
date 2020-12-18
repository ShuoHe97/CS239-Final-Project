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
// Benchmark for testing changes to rendering algorimths
//
// =============================================================================

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

float end_time = 100.0f;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------

    int x_instances = 10;
    int y_instances = 10;
    int z_instances = 10;
    float x_spread = 2.f;
    float y_spread = 2.f;
    float z_spread = 2.f;

    auto color = std::make_shared<ChVisualMaterial>();
    color->SetDiffuseColor({1.f, 1.f, 1.f});
    color->SetSpecularColor({.8, .8, .8});

    // auto material = std::make_shared<ChVisualization>();
    // material->material_list.push_back(color);

    for (int i = 0; i < x_instances; i++) {
        for (int j = 0; j < y_instances; j++) {
            for (int k = 0; k < z_instances; k++) {
                auto sphere = std::make_shared<ChBodyEasySphere>(.75, 1000, false, true);
                sphere->SetPos({x_spread * (i + .5 - x_instances / 2.), y_spread * (j + .5 - y_instances / 2.),
                                z_spread * (k + .5 - z_instances / 2.)});
                sphere->SetBodyFixed(true);
                auto sphere_asset = sphere->GetAssets()[0];
                if (std::shared_ptr<ChVisualization> visual_asset =
                        std::dynamic_pointer_cast<ChVisualization>(sphere_asset)) {
                    // auto vis_mat = std::make_shared<ChVisualMaterial>();
                    // vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
                    // vis_mat->SetDiffuseColor({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()});
                    // vis_mat->SetSpecularColor({.2f, .2f, .2f});

                    visual_asset->material_list.push_back(color);
                }

                // sphere->AddAsset(material);

                mphysicalSystem.Add(sphere);
            }
        }
    }

    auto cam_body = chrono_types::make_shared<ChBodyEasyBox>(.01, .01, .01, 1000, false, false);
    cam_body->SetBodyFixed(true);
    mphysicalSystem.Add(cam_body);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = std::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 500);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    auto cam = std::make_shared<ChCameraSensor>(
        cam_body,                                                           // body camera is attached to
        100,                                                                // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1920,                                                               // image width
        1080,                                                               // image height
        CH_C_PI / 3, 9. / 16. * CH_C_PI / 3);                               // FOV
    cam->SetName("Camera Sensor");
    cam->FilterList().push_back(std::make_shared<ChFilterVisualize>());
    cam->FilterList().push_back(std::make_shared<ChFilterRGBA8Access>());

    // add sensor to the manager
    manager->AddSensor(cam);

    // ---------------
    // Simulate system
    // ---------------
    float orbit_radius = (2 * x_instances) * x_spread + 0.f;
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

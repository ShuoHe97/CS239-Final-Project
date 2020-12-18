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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Main driver function for the hmmwv full model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/utils/ChVisualMaterialUtils.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE;

// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD)
DrivelineType drive_type = DrivelineType::AWD;

SteeringType steering_type = SteeringType::PITMAN_ARM;

// Type of tire model (RIGID, RIGID_MESH, TMEASY, PACEJKA, LUGRE, FIALA, PAC89, PAC02)
TireModelType tire_model = TireModelType::PAC02;

// Rigid terrain
RigidTerrain::Type terrain_model = RigidTerrain::BOX;  // RigidTerrain::BOX,MESH;
double terrainHeight = 0;                              // terrain height (FLAT terrain only)
double terrainLength = 100.0;                          // size in X direction
double terrainWidth = 100.0;                           // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 2e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "hmmwv";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

int num_gpus_to_use = 1;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the hmmwv vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisCollisionType(chassis_collision_type);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetSteeringType(steering_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.SetVehicleStepSize(step_size);
    my_hmmwv.Initialize();

    VisualizationType tire_vis_type =
        (tire_model == TireModelType::RIGID_MESH) ? VisualizationType::MESH : VisualizationType::NONE;

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::BOX:
            patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                     ChVector<>(terrainLength, terrainWidth, 10));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::HEIGHT_MAP:
            patch = terrain.AddPatch(CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"), "test64", 128,
                                     128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::MESH:
            patch = terrain.AddPatch(CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"HMMWV Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    // Initialize output file for driver inputs
    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // Set up vehicle output
    my_hmmwv.GetVehicle().SetChassisOutput(true);
    my_hmmwv.GetVehicle().SetSuspensionOutput(0, true);
    my_hmmwv.GetVehicle().SetSteeringOutput(0, true);
    my_hmmwv.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    my_hmmwv.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // If in playback mode, attach the data file to the driver system and
    // force it to playback the driver inputs.
    if (driver_mode == PLAYBACK) {
        driver.SetInputDataFile(driver_file);
        driver.SetInputMode(ChIrrGuiDriver::DATAFILE);
    }

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        my_hmmwv.LogHardpointLocations();
    }

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << my_hmmwv.GetVehicle().GetVehicleMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int step_number = 0;
    int render_frame = 0;
    double time = 0;

    if (contact_vis) {
        app.SetSymbolscale(1e-4);
        app.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);
    }

    // create the sensor manager and a camera
    auto manager = std::make_shared<ChSensorManager>(my_hmmwv.GetSystem());

    // set lights
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);
    manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 500);

    // set environment map
    manager->scene->GetBackground().has_texture = true;
    manager->scene->GetBackground().env_tex = "sensor/textures/cloud_layers_8k.hdr";
    manager->scene->GetBackground().has_changed = true;

    auto cam1 = std::make_shared<chrono::sensor::ChCameraSensor>(
        my_hmmwv.GetChassisBody(),                                                      // body camera is attached to
        30,                                                                             // update rate in Hz
        chrono::ChFrame<double>({0, -8, 1}, Q_from_AngAxis(CH_C_PI / 2.0, {0, 0, 1})),  // offset pose
        1280,                                                                           // image width
        720,                                                                            // image height
        CH_C_PI / 3, 9 / 16. * CH_C_PI / 3);
    cam1->SetName("Camera Sensor");
    cam1->FilterList().push_back(std::make_shared<ChFilterVisualize>());
    cam1->FilterList().push_back(std::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam1);

    auto cam2 = std::make_shared<chrono::sensor::ChCameraSensor>(
        my_hmmwv.GetChassisBody(),                                            // body camera is attached to
        30,                                                                   // update rate in Hz
        chrono::ChFrame<double>({1, 0, .875}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose
        1280,                                                                 // image width
        720,                                                                  // image height
        CH_C_PI / 3, 9 / 16. * CH_C_PI / 3);
    cam2->SetName("Camera Sensor");
    cam2->FilterList().push_back(std::make_shared<ChFilterVisualize>());
    cam2->FilterList().push_back(std::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam2);

    auto lidar = std::make_shared<ChLidarSensor>(
        my_hmmwv.GetChassisBody(),                                         // body to which the IMU is attached
        10,                                                                // update rate
        chrono::ChFrame<double>({0, 0, 2}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose from body
        2000,                                                              // horizontal samples
        50,                                                                // vertical samples/channels
        (float)2.0f * CH_C_PI,                                             // horizontal field of view
        (float)CH_C_PI / 3.f);                                             // vertical field of view
    lidar->SetName("Lidar Sensor");
    lidar->FilterList().push_back(std::make_shared<ChFilterVisualize>("Lidar Data"));
    lidar->FilterList().push_back(std::make_shared<ChFilterPCfromDepth>());
    lidar->FilterList().push_back(std::make_shared<ChFilterVisualizePointCloud>("Lidar Point Cloud"));
    lidar->FilterList().push_back(std::make_shared<ChFilterXYZIAccess>());
    manager->AddSensor(lidar);

    double render_time = 0;

    int num_camera_updates = 0;

    std::cout << "setup complete." << '\n';

    while (app.GetDevice()->run()) {
        time = my_hmmwv.GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(my_hmmwv.GetSystem(), filename);
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            my_hmmwv.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }

        // Collect output data from modules (for inter-module communication)
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Driver output
        if (driver_mode == RECORD) {
            driver_csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking
                       << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        app.Advance(step);

        std::chrono::high_resolution_clock::time_point r0 = std::chrono::high_resolution_clock::now();
        manager->Update();
        std::chrono::high_resolution_clock::time_point r1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> r_dur = std::chrono::duration_cast<std::chrono::duration<double>>(r1 - r0);
        render_time += r_dur.count();

        // manager->Render();

        // // for accessing data from camera sensor
        // LockedSensorHostBufferPtr image_data = camAccessFilter->GetBuffer();
        // image_data->Buffer;         // std::unique_ptr<unsigned char[]> holding the image data
        // image_data->Width;          // int describing width of image
        // image_data->Height;         // int describing height of image
        // image_data->UpdateCount;    // int for how many times sensor has been updated
        // image_data->BytesPerPixel;  // number of bytes per pixel in image
        //
        // if (image_data->UpdateCount > num_camera_updates) {
        //     num_camera_updates = image_data->UpdateCount;
        //     std::cout << "Image " << image_data->UpdateCount << " ready. H=" << image_data->Height
        //               << ", W=" << image_data->Width << ", bytes per pixel=" << image_data->BytesPerPixel << "\n";
        // }

        // Increment frame number
        step_number++;
    }

    if (driver_mode == RECORD) {
        driver_csv.write_to_file(driver_file);
    }

    return 0;
}

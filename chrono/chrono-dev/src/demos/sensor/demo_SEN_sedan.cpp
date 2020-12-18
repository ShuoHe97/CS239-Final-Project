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
// Main driver function for the Sedan full model.
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
#include "chrono_models/vehicle/sedan/Sedan.h"

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
using namespace chrono::vehicle::sedan;
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
// PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD)
// DrivelineType drive_type = DrivelineType::FWD;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::RIGID;

// Rigid terrain
RigidTerrain::Type terrain_model = RigidTerrain::MESH;  // RigidTerrain::BOX,MESH;
double terrainHeight = 0;                               // terrain height (FLAT terrain only)
double terrainLength = 100.0;                           // size in X direction
double terrainWidth = 100.0;                            // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "Sedan";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

#define USE_IRRLICHT
bool time_code = true;
// bool overlap_rendering = true;
int num_groups = 1;
bool visualize = true;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the Sedan vehicle, set parameters, and initialize

    Sedan my_sedan;
    my_sedan.SetContactMethod(contact_method);
    my_sedan.SetChassisCollisionType(chassis_collision_type);
    my_sedan.SetChassisFixed(false);
    my_sedan.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_sedan.SetTireType(tire_model);
    my_sedan.SetTireStepSize(tire_step_size);
    my_sedan.SetVehicleStepSize(step_size);
    my_sedan.Initialize();

    VisualizationType tire_vis_type = VisualizationType::MESH;

    my_sedan.SetChassisVisualizationType(chassis_vis_type);
    my_sedan.SetSuspensionVisualizationType(suspension_vis_type);
    my_sedan.SetSteeringVisualizationType(steering_vis_type);
    my_sedan.SetWheelVisualizationType(wheel_vis_type);
    my_sedan.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_sedan.GetSystem());

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
            // patch = terrain.AddPatch(CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
            // patch =
            //     terrain.AddPatch(CSYSNORM, "~/ContinentalMapping/Processed/export_02/park_street_vis.obj",
            //     "test_mesh");
            // patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);

            // patch = terrain.AddPatch(CSYSNORM,
            // "/home/asher/ContinentalMapping/Processed/export_02/park_street_col.obj",
            //                          "test_mesh", 0, false);
            patch = terrain.AddPatch(CSYSNORM, "/home/asher/ContinentalMapping/Processed/export_02/park_street_col.obj",
                                     "test_mesh", 0, false);
            std::string vis_file =
                "/home/asher/ContinentalMapping/Processed/export_02/"
                "park_street_vis_split.obj";  //"/home/asher/ContinentalMapping/Processed/export_02/park_street_col.obj";

            auto vis_mesh = std::make_shared<geometry::ChTriangleMeshConnected>();
            vis_mesh->LoadWavefrontMesh(vis_file, true, true);

            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(vis_mesh);
            trimesh_shape->SetName("mesh_name");
            trimesh_shape->SetStatic(true);
            patch->GetGroundBody()->AddAsset(trimesh_shape);

            std::cout << "Added mesh ground\n";
            break;
    }
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    terrain.Initialize();

#ifdef USE_IRRLICHT
    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_sedan.GetVehicle(), L"Sedan Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();
#endif

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

    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1

#ifdef USE_IRRLICHT
    ChIrrGuiDriver driver(app);

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
#endif
    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        my_sedan.LogHardpointLocations();
    }

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << my_sedan.GetVehicle().GetVehicleMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int step_number = 0;
    int render_frame = 0;
    double time = 0;

#ifdef USE_IRRLICHT
    if (contact_vis) {
        app.SetSymbolscale(1e-4);
        app.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);
    }
#endif

    // create the sensor manager and a camera
    auto manager = std::make_shared<ChSensorManager>(my_sedan.GetSystem());
    manager->SetAllowableGroups(num_groups);  // THIS NEEDS MORE PERFORMANCE TESTING

    manager->scene->AddPointLight({-100, -400, 100}, {1, 1, 1}, 500);
    manager->scene->AddPointLight({-100, -200, 100}, {1, 1, 1}, 500);
    manager->scene->AddPointLight({-100, 0, 100}, {1, 1, 1}, 500);
    manager->scene->AddPointLight({-100, 200, 100}, {1, 1, 1}, 500);
    manager->scene->AddPointLight({-100, 400, 100}, {1, 1, 1}, 500);

    auto cam = std::make_shared<chrono::sensor::ChCameraSensor>(
        my_sedan.GetChassisBody(),  // body camera is attached to
        30,                         // update rate in Hz
        // true,                                                                // visualize
        chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(.2, {0, 1, 0})),  // offset pose
        1920,                                                                // image width
        1080,                                                                // image height
        CH_C_PI / 3, 9 / 16. * CH_C_PI / 3);
    cam->SetName("Camera Sensor");
    cam->FilterList().push_back(std::make_shared<ChFilterVisualize>());
    // cam->FilterList().push_back(std::make_shared<ChFilterRGBA8CopyToDevice>());
    manager->AddSensor(cam);

    auto cam2 = std::make_shared<chrono::sensor::ChCameraSensor>(
        my_sedan.GetChassisBody(),  // body camera is attached to
        30,                         // update rate in Hz
        // true,                                                                //
        chrono::ChFrame<double>({0, .35, 1}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose
        1280,                                                                // image width
        720,                                                                 // image height
        CH_C_PI / 3, 9 / 16. * CH_C_PI / 3);
    cam2->SetName("Camera Sensor");
    cam2->FilterList().push_back(std::make_shared<ChFilterVisualize>());
    // cam2->FilterList().push_back(std::make_shared<ChFilterRGBA8CopyToDevice>());
    manager->AddSensor(cam2);

    auto cam3 = std::make_shared<chrono::sensor::ChCameraSensor>(
        my_sedan.GetChassisBody(),  // body camera is attached
        25,                         // update rate in Hz
        // true,                                                                           //
        chrono::ChFrame<double>({0, 5, 1}, Q_from_AngAxis(-CH_C_PI / 2.0, {0, 0, 1})),  // offset pose
        1280,                                                                           // image width
        720,                                                                            // image height
        CH_C_PI / 3, 9 / 16. * CH_C_PI / 3);
    cam3->SetName("Camera Sensor");
    cam3->FilterList().push_back(std::make_shared<ChFilterVisualize>());
    // cam3->FilterList().push_back(std::make_shared<ChFilterRGBA8CopyToDevice>());
    manager->AddSensor(cam3);

    auto cam4 = std::make_shared<chrono::sensor::ChCameraSensor>(
        my_sedan.GetChassisBody(),  // body camera is attached to
        25,                         // update rate in Hz
        // true,                                                                           //
        chrono::ChFrame<double>({0, -5, 1}, Q_from_AngAxis(CH_C_PI / 2.0, {0, 0, 1})),  // offset pose
        1280,                                                                           // image width
        720,                                                                            // image height
        CH_C_PI / 3, 9 / 16. * CH_C_PI / 3);
    cam4->SetName("Camera Sensor");
    cam4->FilterList().push_back(std::make_shared<ChFilterVisualize>());
    // cam4->FilterList().push_back(std::make_shared<ChFilterRGBA8CopyToDevice>());
    manager->AddSensor(cam4);

    auto lidar = std::make_shared<ChLidarSensor>(
        my_sedan.GetChassisBody(),                                         // body to which the IMU is attached
        10,                                                                // update rate
        chrono::ChFrame<double>({0, 0, 2}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose from body
        2000,                                                              // horizontal samples
        48,                                                                // vertical samples/channels
        (float)2.0f * CH_C_PI,                                             // horizontal field of view
        (float)CH_C_PI / 3.f);                                             // vertical field of view
    lidar->SetName("Lidar Sensor");
    lidar->FilterList().push_back(std::make_shared<ChFilterVisualize>("Lidar Data"));
    lidar->FilterList().push_back(std::make_shared<ChFilterPCfromDepth>());
    lidar->FilterList().push_back(std::make_shared<ChFilterVisualizePointCloud>("Lidar Point Cloud"));
    lidar->FilterList().push_back(std::make_shared<ChFilterXYZIAccess>());
    manager->AddSensor(lidar);

    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

    double render_time = 0;

#ifdef USE_IRRLICHT
    while (app.GetDevice()->run()) {
#else
    while (true) {
#endif
        time = my_sedan.GetSystem()->GetChTime();

        if (time_code && step_number % 1000 == 0 && step_number != 0) {
            std::chrono::high_resolution_clock::time_point new_now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> dur =
                std::chrono::duration_cast<std::chrono::duration<double>>(new_now - now);
            std::cout << "Sim time: " << step_size * 1000 << "s, Wall Time: " << dur.count()
                      << "s, Dynamics Took: " << dur.count() - render_time << "s.\n";

            now = std::chrono::high_resolution_clock::now();
            render_time = 0;
        }

        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
#ifdef USE_IRRLICHT
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();
#endif
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(my_sedan.GetSystem(), filename);
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            my_sedan.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }

#ifdef USE_IRRLICHT
        // Collect output data from modules (for inter-module communication)
        ChDriver::Inputs driver_inputs = driver.GetInputs();

#else
        ChDriver::Inputs driver_inputs(0, 0, 0);
#endif
        // Driver output
        if (driver_mode == RECORD) {
            driver_csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking
                       << std::endl;
        }
        // Update modules (process inputs from other modules)
        //
        terrain.Synchronize(time);
        my_sedan.Synchronize(time, driver_inputs, terrain);
#ifdef USE_IRRLICHT
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs);
        driver.Synchronize(time);
#endif
        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);

        terrain.Advance(step);
        my_sedan.Advance(step);
#ifdef USE_IRRLICHT
        app.Advance(step);
        driver.Advance(step);
#endif

        if (step_number % 20 == 0) {
            std::chrono::high_resolution_clock::time_point r0 = std::chrono::high_resolution_clock::now();
            manager->Update();
            std::chrono::high_resolution_clock::time_point r1 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> r_dur = std::chrono::duration_cast<std::chrono::duration<double>>(r1 - r0);
            render_time += r_dur.count();

            // manager->Render();
        }

        // Increment frame number
        step_number++;
    }

    if (driver_mode == RECORD) {
        driver_csv.write_to_file(driver_file);
    }

    return 0;
}

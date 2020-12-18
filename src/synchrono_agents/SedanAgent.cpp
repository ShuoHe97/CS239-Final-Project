#include "SedanAgent.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

namespace synchrono {
namespace agent {

SedanAgent::SedanAgent(int id) : SynAgent(id) {}
SedanAgent::~SedanAgent() {}

void SedanAgent::Initialize(ChVector<> init_pos, ChQuaternion<> init_rot) {
    SetDataPath(CHRONO_VEHICLE_DATA_DIR);
    RigidTerrain::Type terrain_model = RigidTerrain::BOX;
    double terrainHeight = 0;      // terrain height 
    double terrainLength = 100.0;  // size in X direction
    double terrainWidth = 100.0;   // size in Y direction
    ChVector<> trackPoint(0.0, 0.0, 1.75);

    // Sedan my_sedan;
    m_sedan = std::make_shared<Sedan>();
    m_sedan->SetContactMethod(ChMaterialSurface::SMC);
    m_sedan->SetChassisCollisionType(ChassisCollisionType::NONE);
    m_sedan->SetChassisFixed(false);
    m_sedan->SetInitPosition(ChCoordsys<>(init_pos, init_rot));
    m_sedan->SetTireType(TireModelType::RIGID);
    m_sedan->SetTireStepSize(step_size);
    m_sedan->SetVehicleStepSize(step_size);
    m_sedan->Initialize();

    m_sedan->SetChassisVisualizationType(VisualizationType::MESH);
    m_sedan->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_sedan->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_sedan->SetWheelVisualizationType(VisualizationType::MESH);
    m_sedan->SetTireVisualizationType(VisualizationType::MESH);

    // RigidTerrain terrain(m_sedan->GetSystem());
    m_terrain = std::make_shared<RigidTerrain>(m_sedan->GetSystem());

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::BOX:
            patch = m_terrain->AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                        ChVector<>(terrainLength, terrainWidth, 10));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::HEIGHT_MAP:
            patch = m_terrain->AddPatch(CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"), "test64", 128,
                                        128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::MESH:
            patch = m_terrain->AddPatch(CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    m_terrain->Initialize();

    // ChWheeledVehicleIrrApp app(&m_sedan->GetVehicle(), L"Sedan Demo");
    m_app = std::make_shared<ChWheeledVehicleIrrApp>(&m_sedan->GetVehicle(), L"Sedan Demo");
    m_app->SetSkyBox();
    m_app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
                            130);
    m_app->SetChaseCamera(trackPoint, 6.0, 0.5);
    m_app->SetTimestep(step_size);
    m_app->AssetBindAll();
    m_app->AssetUpdateAll();

    // ChIrrGuiDriver driver(app);
    m_driver = std::make_shared<ChIrrGuiDriver>(*m_app.get());

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    m_driver->SetSteeringDelta(render_step_size / steering_time);
    m_driver->SetThrottleDelta(render_step_size / throttle_time);
    m_driver->SetBrakingDelta(render_step_size / braking_time);

    m_driver->Initialize();

    // Number of simulation steps between miscellaneous events
    render_steps = (int)std::ceil(render_step_size / step_size);

    int step_number = 0;
    int render_frame = 0;

    

    std::cout << "=== Sedan Agent Initialized ===\n";
}

void SedanAgent::SecondInitialize() {
    m_app->AssetBindAll();
    m_app->AssetUpdateAll();
}






void SedanAgent::Advance(double time_of_next_sync) {

    

    while (m_sedan->GetSystem()->GetChTime() < time_of_next_sync) {
        double time = m_sedan->GetSystem()->GetChTime();
        m_app->GetDevice()->run();

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            m_app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            m_app->DrawAll();
            m_app->EndScene();
        }

        ChDriver::Inputs driver_inputs = m_driver->GetInputs();

        m_driver->Synchronize(time);
        m_terrain->Synchronize(time);
        m_sedan->Synchronize(time, driver_inputs, *m_terrain);
        m_app->Synchronize(m_driver->GetInputModeAsString(), driver_inputs);

        // Advance simulation for one timestep for all modules
        m_driver->Advance(step_size);
        m_terrain->Advance(step_size);
        m_sedan->Advance(step_size);
        m_app->Advance(step_size);


        // Increment frame number
        
        step_number++;
        // std::cout << "Sedan advanced to step: " << step_number << "\n";
    }

}  // namespace agent

void SedanAgent::InitializeZombie(chrono::ChSystem* ch_system) {
    std::string chassis_vis_file = "sedan/sedan_chassis_vis.obj";
    std::string left_wheel_vis_file = "sedan/wheel_hub_left.obj";
    std::string right_wheel_vis_file = "sedan/wheel_hub_right.obj";
    std::string tire_vis_file = "sedan/tire.obj";

    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(vehicle::GetDataFile(chassis_vis_file), false, false);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(chassis_vis_file);
    trimesh_shape->SetStatic(true);

    m_zombie_body = std::make_shared<ChBodyAuxRef>();
    m_zombie_body->AddAsset(trimesh_shape);
    m_zombie_body->SetCollide(false);
    m_zombie_body->SetBodyFixed(true);
    ch_system->Add(m_zombie_body);
    m_zombie_body->SetFrame_COG_to_REF(ChFrame<>({0, 0, -0.2}, {1, 0, 0, 0}));
    // m_zombie_body->SetFrame_REF_to_abs(ChFrame<>({0, 0, 1.0}, {1, 0, 0, 0}));

    auto tire_mesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    tire_mesh->LoadWavefrontMesh(vehicle::GetDataFile(tire_vis_file), false, false);
    auto m_trimesh_tire = chrono_types::make_shared<ChTriangleMeshShape>();
    m_trimesh_tire->SetMesh(tire_mesh);
    m_trimesh_tire->SetStatic(true);
    m_trimesh_tire->SetName(tire_vis_file);

    auto wheel_left_mesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    wheel_left_mesh->LoadWavefrontMesh(vehicle::GetDataFile(left_wheel_vis_file), false, false);
    auto m_trimesh_shape_wl = chrono_types::make_shared<ChTriangleMeshShape>();
    m_trimesh_shape_wl->SetMesh(wheel_left_mesh);
    m_trimesh_shape_wl->SetStatic(true);
    m_trimesh_shape_wl->SetName(left_wheel_vis_file);

    auto wheel_right_mesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    wheel_right_mesh->LoadWavefrontMesh(vehicle::GetDataFile(right_wheel_vis_file), false, false);
    auto m_trimesh_shape_wr = chrono_types::make_shared<ChTriangleMeshShape>();
    m_trimesh_shape_wr->SetMesh(wheel_right_mesh);
    m_trimesh_shape_wr->SetStatic(true);
    m_trimesh_shape_wr->SetName(right_wheel_vis_file);

    m_w0 = std::make_shared<ChBodyAuxRef>();
    m_w0->AddAsset(m_trimesh_shape_wr);
    m_w0->AddAsset(m_trimesh_tire);
    m_w0->SetCollide(false);
    m_w0->SetBodyFixed(true);
    ch_system->Add(m_w0);

    m_w2 = std::make_shared<ChBodyAuxRef>();
    m_w2->AddAsset(m_trimesh_shape_wr);
    m_w2->AddAsset(m_trimesh_tire);
    m_w2->SetCollide(false);
    m_w2->SetBodyFixed(true);
    ch_system->Add(m_w2);

    m_w1 = std::make_shared<ChBodyAuxRef>();
    m_w1->AddAsset(m_trimesh_shape_wl);
    m_w1->AddAsset(m_trimesh_tire);
    m_w1->SetCollide(false);
    m_w1->SetBodyFixed(true);
    ch_system->Add(m_w1);

    m_w3 = std::make_shared<ChBodyAuxRef>();
    m_w3->AddAsset(m_trimesh_shape_wl);
    m_w3->AddAsset(m_trimesh_tire);
    m_w3->SetCollide(false);
    m_w3->SetBodyFixed(true);
    ch_system->Add(m_w3);
}
void SedanAgent::SynchronizeZombie(std::vector<double> state) {
    // NOTE: this -0.2 in the z direction is part of an offset somewhere in the sedan model
    m_zombie_body->SetFrame_REF_to_abs(
        ChFrame<>({state[1], state[2], state[3] - 0.2}, {state[4], state[5], state[6], state[7]}));
    m_w0->SetFrame_REF_to_abs(ChFrame<>({state[8], state[9], state[10]}, {state[11], state[12], state[13], state[14]}));
    m_w1->SetFrame_REF_to_abs(
        ChFrame<>({state[15], state[16], state[17]}, {state[18], state[19], state[20], state[21]}));
    m_w2->SetFrame_REF_to_abs(
        ChFrame<>({state[22], state[23], state[24]}, {state[25], state[26], state[27], state[28]}));
    m_w3->SetFrame_REF_to_abs(
        ChFrame<>({state[29], state[30], state[31]}, {state[32], state[33], state[34], state[35]}));
}

std::vector<double> SedanAgent::GetState() {
    ChVector<> pos = m_sedan->GetChassisBody()->GetPos();
    ChQuaternion<> rot = m_sedan->GetChassisBody()->GetRot();

    ChVector<> pos_w0 = m_sedan->GetVehicle().GetSpindlePos(0, LEFT);
    ChQuaternion<> rot_w0 = m_sedan->GetVehicle().GetSpindleRot(0, LEFT);

    ChVector<> pos_w1 = m_sedan->GetVehicle().GetSpindlePos(0, RIGHT);
    ChQuaternion<> rot_w1 = m_sedan->GetVehicle().GetSpindleRot(0, RIGHT);

    ChVector<> pos_w2 = m_sedan->GetVehicle().GetSpindlePos(1, LEFT);
    ChQuaternion<> rot_w2 = m_sedan->GetVehicle().GetSpindleRot(1, LEFT);

    ChVector<> pos_w3 = m_sedan->GetVehicle().GetSpindlePos(1, RIGHT);
    ChQuaternion<> rot_w3 = m_sedan->GetVehicle().GetSpindleRot(1, RIGHT);

    return {m_sedan->GetSystem()->GetChTime(),
            pos.x(),
            pos.y(),
            pos.z(),
            rot.e0(),
            rot.e1(),
            rot.e2(),
            rot.e3(),
            pos_w0.x(),
            pos_w0.y(),
            pos_w0.z(),
            rot_w0.e0(),
            rot_w0.e1(),
            rot_w0.e2(),
            rot_w0.e3(),  //
            pos_w1.x(),
            pos_w1.y(),
            pos_w1.z(),
            rot_w1.e0(),
            rot_w1.e1(),
            rot_w1.e2(),
            rot_w1.e3(),
            pos_w2.x(),
            pos_w2.y(),
            pos_w2.z(),
            rot_w2.e0(),
            rot_w2.e1(),
            rot_w2.e2(),
            rot_w2.e3(),
            pos_w3.x(),
            pos_w3.y(),
            pos_w3.z(),
            rot_w3.e0(),
            rot_w3.e1(),
            rot_w3.e2(),
            rot_w3.e3()};
}



// virtual void ProcessState() = 0;

chrono::ChSystem* SedanAgent::GetSystem() {
    return m_sedan->GetSystem();
}

}  // namespace agent
}  // namespace synchrono

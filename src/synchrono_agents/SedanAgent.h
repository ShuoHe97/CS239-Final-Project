
#ifndef SEDANAGENT_H
#define SEDANAGENT_H

#include "SynAgent.h"

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

namespace synchrono {
namespace agent {

class SYN_AGENT_API SedanAgent : public SynAgent {
  public:
    SedanAgent(int id);
    ~SedanAgent();
    virtual void Initialize(ChVector<> init_pos, ChQuaternion<> init_rot);
    virtual void SecondInitialize();
    virtual void Advance(double time);

    virtual void InitializeZombie(chrono::ChSystem* ch_system);
    virtual void SynchronizeZombie(std::vector<double> state);



    virtual std::vector<double> GetState();
    

    virtual chrono::ChSystem* GetSystem();

  private:
    chrono::ChSystem* m_system;
    

    // agent specific parameters
    double step_size = 0.0005;
    double render_step_size = 1.0 / 50;  // FPS = 50

    
    std::shared_ptr<ChIrrGuiDriver> m_driver;
    std::shared_ptr<RigidTerrain> m_terrain;
    std::shared_ptr<Sedan> m_sedan;
    std::shared_ptr<ChWheeledVehicleIrrApp> m_app;
    int step_number;
    int render_steps;
    int render_frame;

    // zombie
    std::shared_ptr<ChBodyAuxRef> m_zombie_body;
    std::shared_ptr<ChBodyAuxRef> m_w0;
    std::shared_ptr<ChBodyAuxRef> m_w1;
    std::shared_ptr<ChBodyAuxRef> m_w2;
    std::shared_ptr<ChBodyAuxRef> m_w3;
};
}  // namespace agent
}  // namespace synchrono
#endif

#ifndef SYNAGENT_H
#define SYNAGENT_H

#include "SynApiAgent.h"

#include <queue>
#include <string>
#include <unordered_map>

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"


#include "SynApiAgent.h"

namespace synchrono {
namespace agent {



class SYN_AGENT_API SynAgent {
  public:
    SynAgent(unsigned int id);
    ~SynAgent() {}

    unsigned int GetId() const { return m_id; }
    

    virtual void Initialize(chrono::ChVector<> init_pos, chrono::ChQuaternion<> init_rot) = 0;
    virtual void
    SecondInitialize() = 0;  
    virtual void Advance(double time_of_next_sync) = 0;

    virtual void InitializeZombie(chrono::ChSystem* ch_system) = 0;
    virtual void SynchronizeZombie(std::vector<double> state) = 0;

    virtual std::vector<double> GetState() = 0;
    
    

    

    virtual chrono::ChSystem* GetSystem() = 0;

    

  protected:
    unsigned int m_id;
    
};

} 
}  

#endif

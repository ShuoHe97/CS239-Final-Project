#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/core/ChQuaternion.h"

#include "chrono_thirdparty/filesystem/path.h"
#include <mpi.h>

#include <chrono>
#include <map>

#include "synchrono_agents/SynAgent.h"
#include "synchrono_agents/SedanAgent.h"



using namespace synchrono::agent;

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


const double T_END = 5;
const double HEARTBEAT = 0.01;
const double STEP_SIZE = 0.001;
const double SIM_EPSILON = 1e-9;






int main(int argc, char* argv[]) {
    
    SetChronoDataPath(CHRONO_DATA_DIR);

    
    // MPI Initialization
    

    std::map<int, std::shared_ptr<SynAgent>> agent_list;

    MPI_Init(&argc, &argv);

    int myRank, numRanks;
    MPI_Comm_rank(MPI_COMM_WORLD, &myRank);
    MPI_Comm_size(MPI_COMM_WORLD, &numRanks);

    int long_length = 131072;

    std::vector<double> agent_state(long_length);
    std::vector<double> temp_state;
    
    int state_length = 36;
    
    
    
    

    std::vector<double> all_states = std::vector<double>(long_length * numRanks);
    

    
    // intialization phase
    
    
    
    auto ego_agent = std::make_shared<SedanAgent>(myRank);
    ego_agent->Initialize({0, 2.0 * myRank, 0.5}, {1, 0, 0, 0});
    agent_list[myRank] = ego_agent;

    for (int i = 0; i < numRanks; i++) {
        // if the agent is not associated with our rank, then it is a zombie
        if (i != myRank) {
            auto z_agent = std::make_shared<SedanAgent>(i);
            z_agent->InitializeZombie(ego_agent->GetSystem());
            agent_list[i] = z_agent;
        }
    }

    ego_agent->SecondInitialize();

    

    double sim_time = 0;
    int step = 1;

    std::cout << std::endl;
    // everyone should be on the same page before we progress
    MPI_Barrier(MPI_COMM_WORLD);

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    // simulation loop
    while (step * HEARTBEAT < T_END) {
        // collect states from the agent we are responsible for
        temp_state = agent_list[myRank]->GetState();

        for (int i = 0; i < state_length; ++i) {
            agent_state[i] = temp_state[i];
        }
        
        
/*
        // synchronize between ranks
        if (myRank == 0) {
            MPI_Send(agent_state.data(), long_length, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD);
            for (int i = 0; i < numRanks; i++) {
                MPI_Recv(&all_states.data()[i * long_length], long_length, MPI_DOUBLE, i, 0, MPI_COMM_WORLD,
                         MPI_STATUS_IGNORE);

            }

        } else {
            MPI_Send(agent_state.data(), long_length, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD);
            
        }
*/       



/*
        MPI_Gather(agent_state.data(), long_length, MPI_DOUBLE,
               all_states.data(), long_length, MPI_DOUBLE, 0, MPI_COMM_WORLD);
*/
/*
        // send the entire set of collected states to all the ranks
        MPI_Bcast(all_states.data(), numRanks * long_length, MPI_DOUBLE, 0, MPI_COMM_WORLD);
*/




        MPI_Allgather(agent_state.data(), long_length, MPI_DOUBLE,
                  all_states.data(), long_length, MPI_DOUBLE,
                  MPI_COMM_WORLD);

        

        // update the zombie agents
        for (int i = 0; i < numRanks; i++) {
            if (i != myRank) {
                std::vector<double>::const_iterator z_start = all_states.begin() + i * long_length;
                std::vector<double>::const_iterator z_end = all_states.begin() + (i ) * long_length + state_length;
                std::vector<double> zState(z_start, z_end);

                // advance the agent to a specific time where we will synchronize
                agent_list[i]->SynchronizeZombie(zState);
            }
        }

        
        // advance the live agent
        agent_list[myRank]->Advance(step * HEARTBEAT);
        

        // increment the step
        step++;
    }

    if (myRank == 0) {
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "Wall Time: " << time_span.count() << ".\n";
    }

    // should eventually look up what sort of agent we are from an initialization file of some sort
    // but for now we are all sedans

    MPI_Finalize();

    return 0;
}

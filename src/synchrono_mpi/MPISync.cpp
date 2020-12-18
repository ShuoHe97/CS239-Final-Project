#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include <mpi.h>
#include "MPISync.h"
#include <iostream>
using namespace chrono;

void printTime(double* gTimes, int numRanks) {
    int n = sizeof(gTimes) / sizeof(gTimes[0]);
    for (int i = 1; i < numRanks; i++) {
        if (gTimes[i] == -1.0) {
            break;
        }
        std::cout << "Reporting value " << gTimes[i] << std::endl;
    }
}

struct vehicleData {
    ChVector<> loc;
    ChQuaternion<> rot;
    double time;
};

void MPISyncInfo::initSync(int* argc, char*** argv, double endTime) {
    MPI_Init(argc, argv);

    MPI_Comm_rank(MPI_COMM_WORLD, &myRank);
    MPI_Comm_size(MPI_COMM_WORLD, &numRanks);

    gTimes = (double*)malloc(numRanks * 1 * sizeof(double));
    t_end = endTime;
}

bool MPISyncInfo::doSync(double time) {
    if (time < syncTime) {
        return 0;
    } else if (time > t_end) {
        MPI_Gather(&time, 1, MPI_DOUBLE, gTimes, 1, MPI_DOUBLE, 0, MPI_COMM_WORLD);
        std::cout << "Simulation rank " << myRank << " hitting synchronization time " << time << " seconds"
                  << std::endl;
        MPI_Bcast(&syncTime, 1, MPI_DOUBLE, 0, MPI_COMM_WORLD);
        return 1;
    }
    return 0;
}

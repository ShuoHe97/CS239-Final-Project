#ifndef MPISYNC_H
#define MPISYNC_H

#include "synchrono_mpi/SynApiMPI.h"


class SYN_MPI_API MPISyncInfo {
  private:
    int myRank;
    int numRanks;

    double syncTime;
    double* gTimes;
    double endTime;
    double t_end;

  public:
    void initSync(int* argc, char*** argv, double endTime);

    int getRank() { return myRank; }

    double getSyncTime() { return syncTime; }

    bool doSync(double time);
};

SYN_MPI_API void printTime(double* gTimes, int numRanks);

struct vehicleData;

#endif

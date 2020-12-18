

#ifndef SYN_API_MPI_H
#define SYN_API_MPI_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"



#if defined(SYN_API_COMPILE_MPI)
#define SYN_MPI_API ChApiEXPORT
#else
#define SYN_MPI_API ChApiIMPORT
#endif


#endif

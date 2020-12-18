

#ifndef SYN_API_WORLD_H
#define SYN_API_WORLD_H

#include "chrono/core/ChPlatform.h"



#if defined(SYN_API_COMPILE_WORLD)
#define SYN_WORLD_API ChApiEXPORT
#else
#define SYN_WORLD_API ChApiIMPORT
#endif



#endif

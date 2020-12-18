

#ifndef SYN_API_AGENT_H
#define SYN_API_AGENT_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

#if defined(SYN_API_COMPILE_AGENT)
#define SYN_AGENT_API ChApiEXPORT
#else
#define SYN_AGENT_API ChApiIMPORT
#endif

#endif

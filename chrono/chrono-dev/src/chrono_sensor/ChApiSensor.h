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
// Author: Asher Elmquist
// =============================================================================
//
// Macro defines for exporting DLL
// =============================================================================

#ifndef CHAPISENSOR_H_
#define CHAPISENSOR_H_

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_SENSOR
// (so that the symbols with 'CH_SENSOR_API' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_SENSOR)
#define CH_SENSOR_API ChApiEXPORT
#else
#define CH_SENSOR_API ChApiIMPORT
#endif

/**
    @defgroup sensor SENSOR module
    @brief Sensor modeling and simulation

    This module provides support for modelling sensor for simulation autonomous behavior

    For additional information, see:
        - none

    @{
        @defgroup sensor_dynamics Dynamic Sensor: GPS, IMU
        @defgroup sensor_optix Optix dependent models: Camera, Lidar, Radar
    @}
*/

namespace chrono {

/// @addtogroup sensor
/// @{

/// Namespace with classes for the Sensor module.
namespace sensor {}

/// @}
}  // namespace chrono

#endif

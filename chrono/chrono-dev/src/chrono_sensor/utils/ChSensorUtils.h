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
// Authors: Asher Elmquist
// =============================================================================
//
// utils file for Chrono sensor
//
// =============================================================================

#ifndef CHSENSORUTILS_H
#define CHSENSORUTILS_H

#include "chrono_sensor/ChApiSensor.h"

#include <string>

namespace chrono {
namespace sensor {

CH_SENSOR_API void TestUtils();
CH_SENSOR_API std::string GetPTXGeneratedLocation();

}  // namespace sensor
}  // namespace chrono

#endif

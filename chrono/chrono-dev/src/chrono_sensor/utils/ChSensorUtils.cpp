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

#include "chrono_sensor/utils/ChSensorUtils.h"
#include <iostream>

namespace chrono {
namespace sensor {

void TestUtils() {
    std::cout << "Sensor utils testing" << std::endl;
}

CH_SENSOR_API std::string GetPTXGeneratedLocation() {
    return PTX_GENERATED_PATH;
}

}  // namespace sensor
}  // namespace chrono

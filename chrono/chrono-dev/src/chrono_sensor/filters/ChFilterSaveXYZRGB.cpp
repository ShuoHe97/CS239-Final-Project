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
//
// =============================================================================

#include "chrono_sensor/filters/ChFilterSaveXYZRGB.h"
#include "chrono_sensor/ChSensor.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterSaveXYZRGB::ChFilterSaveXYZRGB(std::string data_path) : ChFilter("") {
    m_path = data_path;
}

CH_SENSOR_API ChFilterSaveXYZRGB::~ChFilterSaveXYZRGB() {}

CH_SENSOR_API void ChFilterSaveXYZRGB::Apply(std::shared_ptr<ChSensor> pSensor,
                                             std::shared_ptr<SensorBuffer>& bufferInOut) {
    std::shared_ptr<SensorOptixBuffer> pOptix = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);

    pOptix->Buffer->map();

    pOptix->Buffer->unmap();
}

}  // namespace sensor
}  // namespace chrono

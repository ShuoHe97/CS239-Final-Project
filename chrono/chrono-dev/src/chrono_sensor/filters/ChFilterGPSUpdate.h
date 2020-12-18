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
// =============================================================================

#ifndef CHFILTERGPSUPDATE_H
#define CHFILTERGPSUPDATE_H

#include <memory>
#include "chrono_sensor/filters/ChFilter.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

class CH_SENSOR_API ChFilterGPSUpdate : public ChFilter {
  public:
    ChFilterGPSUpdate();
    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) {}

  private:
    std::shared_ptr<SensorHostGPSBuffer> m_buffer;
};

}  // namespace sensor
}  // namespace chrono

#endif

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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CHFILTERGRAYSCALE_H
#define CHFILTERGRAYSCALE_H

#include "chrono_sensor/filters/ChFilter.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

// a filter that, when applied to a sensor, changes the RGB buffer to grayscale (still RGBA format, though)
class CH_SENSOR_API ChFilterGrayscale : public ChFilter {
  public:
    ChFilterGrayscale(std::string name = {});

    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) {}

  private:
    std::shared_ptr<SensorDeviceR8Buffer> m_buffer;
};

}  // namespace sensor
}  // namespace chrono

#endif
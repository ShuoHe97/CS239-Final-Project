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

#ifndef CHFILTEROPTIXRENDER_H
#define CHFILTEROPTIXRENDER_H

#include <memory>
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/optixcpp/ChOptixUtils.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

class CH_SENSOR_API ChFilterOptixRender : public ChFilter {
  public:
    ChFilterOptixRender();
    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor);

  private:
    optix::Buffer AllocateBuffer(std::shared_ptr<ChSensor> pSensor);
    std::shared_ptr<ChFilterVisualize> FindOnlyVisFilter(std::shared_ptr<ChSensor> pSensor);

    std::shared_ptr<SensorOptixBuffer> m_buffer;
    optix::Program m_program;
};

}  // namespace sensor
}  // namespace chrono

#endif

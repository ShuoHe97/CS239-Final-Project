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
// Container class for an IMU sensor
//
// =============================================================================

#ifndef CHIMUSENSOR_H
#define CHIMUSENSOR_H

// #include "ChApiSensor.h"

#include "chrono_sensor/ChSensor.h"

namespace chrono {
namespace sensor {

class CH_SENSOR_API ChIMUSensor : public ChSensor {
  public:
    ChIMUSensor(std::shared_ptr<chrono::ChBody> parent,  // body to which camera is attached
                double updateRate,                       // update rate in Hz
                chrono::ChFrame<double> offsetPose       // offset of camera from Body frame
    );
    ~ChIMUSensor();

    void SetOffsetPose(chrono::ChFrame<double> pose) { m_offsetPose = pose; }
    chrono::ChFrame<double> GetOffsetPose() { return m_offsetPose; }
};
}  // namespace sensor
}  // namespace chrono

#endif

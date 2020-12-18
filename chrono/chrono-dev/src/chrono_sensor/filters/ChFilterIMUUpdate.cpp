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

#include "chrono_sensor/filters/ChFilterIMUUpdate.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/ChIMUSensor.h"
#include "chrono_sensor/ChSensor.h"

namespace chrono {
namespace sensor {

ChFilterIMUUpdate::ChFilterIMUUpdate() : ChFilter("IMU Updater") {}

CH_SENSOR_API void ChFilterIMUUpdate::Apply(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    // TODO: This sensor is not currently threadsafe, so we need to have the sensormanager do the computation directly
    // for now. Possible solution is to have the sensormanager save relavant scene information in threadsafe struct so
    // each worker thread can access as it wishes

    // TODO: have the IMU use data from ALL chrono timesteps and average to get the "ground truth" data

    // TODO: change to account for offset pose
    chrono::ChVector<> currLinAccel =
        pSensor->GetParent()->GetPos_dtdt() - pSensor->GetParent()->GetSystem()->Get_G_acc();
    currLinAccel = pSensor->GetParent()->GetRot().Rotate(currLinAccel);

    chrono::ChVector<> currAngVel = pSensor->GetParent()->GetWvel_loc();

    if (!m_buffer) {
        m_buffer = std::make_shared<SensorHostIMUBuffer>();
        m_buffer->Buffer = std::make_unique<IMUData[]>(1);
        m_buffer->Width = m_buffer->Height = 1;
    }

    // load IMU data
    m_buffer->Buffer[0].Accel[0] = currLinAccel.x();
    m_buffer->Buffer[0].Accel[1] = currLinAccel.y();
    m_buffer->Buffer[0].Accel[2] = currLinAccel.z();
    m_buffer->Buffer[0].Roll = currAngVel.x();
    m_buffer->Buffer[0].Pitch = currAngVel.y();
    m_buffer->Buffer[0].Yaw = currAngVel.z();

    bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono

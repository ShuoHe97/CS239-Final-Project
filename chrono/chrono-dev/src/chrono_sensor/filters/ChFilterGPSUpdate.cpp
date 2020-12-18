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

#include "chrono_sensor/filters/ChFilterGPSUpdate.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/ChGPSSensor.h"
#include "chrono_sensor/ChSensor.h"

namespace chrono {
namespace sensor {

ChFilterGPSUpdate::ChFilterGPSUpdate() : ChFilter("GPS Updater") {}

CH_SENSOR_API void ChFilterGPSUpdate::Apply(std::shared_ptr<ChSensor> pSensor,
                                                  std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    // TODO: This sensor is not currently threadsafe, so we need to have the sensormanager do the computation directly
    // for now. Possible solution is to have the sensormanager save relavant scene information in threadsafe struct so
    // each worker thread can access as it wishes

    // TODO: have the GPS use data from ALL chrono timesteps and average to get the "ground truth" data

    // TODO: change to account for offset pose

	if (!m_buffer) {
        m_buffer = std::make_shared<SensorHostGPSBuffer>();
		m_buffer->Buffer = std::make_unique<GPSData[]>(1);
        m_buffer->Width = m_buffer->Height = 1;
	}
    // load GPS data
    m_buffer->Buffer[0].Latitude = pSensor->GetParent()->GetPos().y();
    m_buffer->Buffer[0].Longitude = pSensor->GetParent()->GetPos().x();
    m_buffer->Buffer[0].Altitude = pSensor->GetParent()->GetPos().z();
    m_buffer->Buffer[0].Time = pSensor->GetParent()->GetSystem()->GetChTime();

	bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono

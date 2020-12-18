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
// Class for managing the all sensor updates
//
// =============================================================================

#ifndef CHSENSORMANAGER_H
#define CHSENSORMANAGER_H

// API include
#include "chrono_sensor/ChApiSensor.h"

#include "chrono/physics/ChSystem.h"

#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/optixcpp/ChOptixEngine.h"
#include "chrono_sensor/scene/ChScene.h"

#include <fstream>
#include <sstream>

namespace chrono {
namespace sensor {
class CH_SENSOR_API ChSensorManager {
  public:
    ChSensorManager(ChSystem* chrono_system);
    ~ChSensorManager();

    void Update();  // update the sensor manager -> renders and updates as it needs

    void AddSensor(std::shared_ptr<ChSensor> sensor);

    std::vector<std::shared_ptr<ChSensor>> GetSensorList() { return m_sensor_list; }

    void SetDeviceList(std::vector<unsigned int> device_ids);  // set devices that should be used
    std::vector<unsigned int> GetDeviceList();                 // get the list of devices being used

    int GetNumEngines() { return (int)m_engines.size(); }
    std::shared_ptr<ChOptixEngine> GetEngine(int context_id);

    int GetAllowableGroups() { return m_allowable_groups; }
    void SetAllowableGroups(int num_groups);

    std::shared_ptr<ChScene> scene;

  private:
    int frames = 0;

    // class variables
    ChSystem* m_system;
    std::vector<std::shared_ptr<ChOptixEngine>> m_engines;

    int m_allowable_groups = 1;

    std::vector<unsigned int> m_device_list;

    std::vector<std::shared_ptr<ChSensor>> m_sensor_list;
    std::vector<std::shared_ptr<ChSensor>> m_dynamic_sensor;
    std::vector<std::shared_ptr<ChSensor>> m_render_sensor;
};
}  // namespace sensor
}  // namespace chrono

#endif

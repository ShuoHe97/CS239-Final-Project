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
// Class for managing the Optix rendering system
//
// =============================================================================

#include "chrono_sensor/ChSensorManager.h"
#include <iomanip>
#include <iostream>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChSensorManager::ChSensorManager(ChSystem* chrono_system) {
    // save the chrono system handle
    m_system = chrono_system;
    scene = std::make_shared<ChScene>();
}

CH_SENSOR_API ChSensorManager::~ChSensorManager() {}

CH_SENSOR_API std::shared_ptr<ChOptixEngine> ChSensorManager::GetEngine(int context_id) {
    if (context_id < m_engines.size())
        return m_engines[context_id];
    std::cerr << "ERROR: index out of render group vector bounds\n";
    return NULL;
}

CH_SENSOR_API void ChSensorManager::Update() {
    // update the scene
    // scene->PackFrame(m_system);
    //
    // // have all the optix engines update their sensor
    for (auto pEngine : m_engines) {
        pEngine->UpdateSensor(scene);
    }

    // have the sensormanager update all of the non-optix sensor (IMU and GPS).
    // TODO: perhaps create a thread that takes care of this? Tradeoff since IMU should require some data from EVERY
    // step
    for (auto pSensor : m_dynamic_sensor) {
        if (m_system->GetChTime() * pSensor->GetUpdateRate() + 1e-5 > pSensor->GetNumUpdates()) {
            std::shared_ptr<SensorBuffer> buffer;
            pSensor->IncrementNumUpdates();
            // step through the filter list, applying each filter
            for (auto filter : pSensor->FilterList()) {
                filter->Apply(pSensor, buffer);
            }
        }
    }
}

CH_SENSOR_API void ChSensorManager::SetDeviceList(std::vector<unsigned int> device_ids) {
    // set the list of devices to use
}
CH_SENSOR_API std::vector<unsigned int> ChSensorManager::GetDeviceList() {
    // return the list of devices being used
    return m_device_list;
}

CH_SENSOR_API void ChSensorManager::SetAllowableGroups(int num_groups) {
    if (num_groups > 0 && num_groups < 1000) {
        m_allowable_groups = num_groups;
    }
}

CH_SENSOR_API void ChSensorManager::AddSensor(std::shared_ptr<ChSensor> sensor) {
    // check if sensor is already in sensor list
    if (std::find(m_sensor_list.begin(), m_sensor_list.end(), sensor) != m_sensor_list.end()) {
        std::cerr << "WARNING: Sensor already exists in manager. Ignoring this addition\n";
        return;
    }
    m_sensor_list.push_back(sensor);

    if (auto pOptixSensor = std::dynamic_pointer_cast<IRendersWithOptix>(sensor)) {
        m_render_sensor.push_back(sensor);
        //******** give each render group all sensor with same update rate *************//
        bool found_group = false;

        // add the sensor to an engine with sensor of similar update frequencies
        for (auto engine : m_engines) {
            if (!found_group && engine->GetSensor().size() > 0 &&
                abs(engine->GetSensor()[0]->GetUpdateRate() - sensor->GetUpdateRate()) < 0.001) {
                found_group = true;
                engine->AssignSensor(sensor);
                std::cout << "Sensor added to existing engine\n";
            }
        }

        // create new engines only when we need them
        if (!found_group) {
            if (m_engines.size() < m_allowable_groups) {
                auto engine = std::make_shared<ChOptixEngine>(
                    m_system, (int)m_engines.size() % 2);  // limits to 2 gpus, TODO: check if device supports cuda

                engine->ConstructScene();
                engine->AssignSensor(sensor);

                m_engines.push_back(engine);
                std::cout << "Created another group. Now at: " << m_engines.size() << "\n";

            } else {  // if we are not allowed to create additional groups, warn the user and polute the first group
                // std::cout << "No more allowable groups, consider allowing more groups if performace would
                // increase\n";
                m_engines[0]->AssignSensor(sensor);
                std::cout << "Couldn't find good existing engine, so adding to engine 0\n";
            }
        }
    } else {
        // add a non render-based sensor
        m_dynamic_sensor.push_back(sensor);
    }
}

}  // namespace sensor
}  // namespace chrono

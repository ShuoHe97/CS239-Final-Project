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
// OptiX rendering engine for processing jobs for sensing. Jobs are defined on
// each sensor as a graph. Recommended to use one engine per GPU to mitigate
// OptiX blocking launch calls
//
// =============================================================================

#ifndef CHOPTIXENGINE_H
#define CHOPTIXENGINE_H

#include "chrono_sensor/ChApiSensor.h"

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/scene/ChScene.h"

#include "chrono/assets/ChVisualMaterial.h"

namespace chrono {
namespace sensor {

class CH_SENSOR_API ChOptixEngine {
  public:
    ChOptixEngine(ChSystem* sys);
    ChOptixEngine(ChSystem* sys, int device_id);
    ~ChOptixEngine();

    void AssignSensor(std::shared_ptr<ChSensor> sensor);
    void UpdateSensor(std::shared_ptr<ChScene> scene);
    void Stop();
    void Start();

    void ConstructScene();

    optix::Context GetOptixContext() { return m_context; }  // return the optix context for this group
    optix::Group GetRootNode() { return m_root; }           // return the root node of this scene
    int GetDevice() { return m_deviceId; }                  // can only look at the device number, cannot change it
    int GetNumSensor() { return (int)m_assignedSensor.size(); }
    std::vector<std::shared_ptr<ChSensor>> GetSensor() { return m_assignedSensor; }

  private:
    void Initialize();
    void Process();  // function that processes sensor added to its queue
    // optix::Buffer AllocateBuffer(std::shared_ptr<ChSensor> pSensor);

    void UpdateCameraTransforms();
    void UpdateBodyTransforms();
    void UpdateSceneDescription(std::shared_ptr<ChScene> scene);
    // void UpdateScene();

    optix::Material CreateMaterial();
    optix::Material CreateMaterial(std::shared_ptr<ChVisualMaterial> chmat);
    optix::TextureSampler CreateTexture(std::string filename);
    optix::TextureSampler CreateTexture();
    optix::Transform CreateTransform(ChMatrix33<double> a, ChVector<double> b);
    void UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector<double> b);

    std::thread m_thread;                                  // thread for performing render operations
    std::vector<std::shared_ptr<ChSensor>> m_renderQueue;  // queue of sensor to process

    std::deque<Keyframe> m_keyframes;

    // mutex and condition variables
    std::mutex m_renderQueueMutex;
    std::condition_variable m_renderQueueCV;
    bool m_terminate = false;
    bool m_started = false;

    optix::Buffer m_light_buffer;

    // information that belongs to the rendering concept of this engine
    optix::Context m_context;                                  // the optix context we use for everything
    optix::Group m_root;                                       // root node of the optix scene
    std::vector<std::shared_ptr<ChSensor>> m_assignedSensor;  // list of sensor we are responsible for
    ChSystem* m_system;                                        // the chrono system we are working from
    std::vector<std::pair<std::shared_ptr<ChBody>, optix::Transform>> m_bodies;  // matching bodies to transforms
    int m_deviceId;                                                              // which device we should be using
    int m_recursions = 29;
};

}  // namespace sensor
}  // namespace chrono

#endif

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

#ifndef CHFILTERVISUALIZEPOINTCLOUD_H
#define CHFILTERVISUALIZEPOINTCLOUD_H

// #include "glad.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "chrono_sensor/filters/ChFilter.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

// a filter that, when applied to a sensor, creates a GUI window to visualize the sensor (using GLFW)
class CH_SENSOR_API ChFilterVisualizePointCloud : public ChFilter {
  public:
    ChFilterVisualizePointCloud(std::string name = {});
    virtual ~ChFilterVisualizePointCloud();

    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) {}

    void CreateGlfwWindow(std::shared_ptr<ChSensor> pSensor);
    void MakeGlContextActive();

  private:
    // helper to allow GLFWwindow to be in a unique_ptr
    struct DestroyglfwWin {
        void operator()(GLFWwindow* ptr) { glfwDestroyWindow(ptr); }
    };

    std::unique_ptr<GLFWwindow, DestroyglfwWin> m_window;
    unsigned int m_gl_tex_id = 0;

    static void OnNewWindow();
    static void OnCloseWindow();
    static int s_windowCount;
};

}  // namespace sensor
}  // namespace chrono

#endif

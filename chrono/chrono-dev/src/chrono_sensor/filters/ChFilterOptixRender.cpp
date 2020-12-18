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

#include "chrono_sensor/filters/ChFilterOptixRender.h"
#include <assert.h>
#include <algorithm>
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChDepthCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/ChSensorBuffer.h"

namespace chrono {
namespace sensor {

ChFilterOptixRender::ChFilterOptixRender() : ChFilter("OptixRenderer") {}

CH_SENSOR_API void ChFilterOptixRender::Apply(std::shared_ptr<ChSensor> pSensor,
                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    // to render, the sensor *must* inherit from IRendersWithOptix
    std::shared_ptr<IRendersWithOptix> pOptixSensor = std::dynamic_pointer_cast<IRendersWithOptix>(pSensor);
    if (!pOptixSensor) {
        throw std::runtime_error(
            "The optix render filter must be attached to a sensor that implements IRendersWithOptix");
    }

    try {
        pOptixSensor->Context()->launch(pOptixSensor->LaunchIndex(), pOptixSensor->RenderWidth(),
                                        pOptixSensor->RenderHeight(), 1);
    } catch (const optix::Exception& ex) {
        std::cout << "Error launching optix render for sensor " << pSensor->GetName() << ": " << ex.getErrorString()
                  << std::endl;
    }

    bufferInOut = std::dynamic_pointer_cast<SensorBuffer>(m_buffer);
}

CH_SENSOR_API void ChFilterOptixRender::Initialize(std::shared_ptr<ChSensor> pSensor) {
    std::shared_ptr<IRendersWithOptix> pOptixSensor = std::dynamic_pointer_cast<IRendersWithOptix>(pSensor);
    if (!pOptixSensor) {
        throw std::runtime_error(
            "The optix render filter must be attached to a sensor that implements IRendersWithOptix");
    }

    // allocate the buffer and ray generation program
    optix::Context context = pOptixSensor->Context();

    optix::Program ray_gen_program = GetRTProgram(context, pOptixSensor->RenderProgramString().file_name,
                                                  pOptixSensor->RenderProgramString().program_name);

    // get the sensor reference frames
    ChFrame<double> f_offset = pSensor->GetOffsetPose();
    ChFrame<double> f_body = pSensor->GetParent()->GetAssetsFrame();

    ChFrame<double> global_loc = f_body * f_offset;

    // set the render reference frames for OptiX
    ray_gen_program["c_pos"]->setFloat((float)global_loc.GetPos().x(), (float)global_loc.GetPos().y(),
                                       (float)global_loc.GetPos().z());
    ray_gen_program["c_forward"]->setFloat((float)global_loc.GetA()(0), (float)global_loc.GetA()(2),
                                           (float)global_loc.GetA()(5));  // camera forward
    ray_gen_program["c_left"]->setFloat((float)global_loc.GetA()(1), (float)global_loc.GetA()(3),
                                        (float)global_loc.GetA()(6));  // camera left
    ray_gen_program["c_up"]->setFloat((float)global_loc.GetA()(2), (float)global_loc.GetA()(4),
                                      (float)global_loc.GetA()(7));  // camera up

    // set the FOVs for OptiX
    ray_gen_program["c_hFOV"]->setFloat(pOptixSensor->RenderHFOV());  // camera horizontal field of view
    ray_gen_program["c_vFOV"]->setFloat(pOptixSensor->RenderVFOV());  // camera horizontal field of view

    // give the ray gen program back to the sensor (DANGEROUS THREADING ISSUES - SHOULD THE SENSOR REALLY EXPOSE
    // THIS TO THE USER?)
    m_program = ray_gen_program;
    pOptixSensor->RayGenProgram() = m_program;

    unsigned int numEntryPoints = context->getEntryPointCount();
    context->setEntryPointCount(numEntryPoints + 1);  // set the entry point count
    pOptixSensor->LaunchIndex() = numEntryPoints;

    optix::Buffer buffer = AllocateBuffer(pSensor);

    pOptixSensor->RayGenProgram()["output_buffer"]->set(buffer);
    context->setRayGenerationProgram(pOptixSensor->LaunchIndex(), pOptixSensor->RayGenProgram());
    // context->setMissProgram(pOptixSensor->LaunchIndex(), pOptixSensor->MissProgram());

    m_buffer = std::make_unique<SensorOptixBuffer>();
    m_buffer->Buffer = buffer;
    m_buffer->Width = pOptixSensor->RenderWidth();
    m_buffer->Height = pOptixSensor->RenderHeight();

    // check that the context is valid
    try {
        pOptixSensor->Context()->validate();
    } catch (const optix::Exception& ex) {
        std::string str = ex.getErrorString();
        std::cout << str;
    }
}

CH_SENSOR_API optix::Buffer ChFilterOptixRender::AllocateBuffer(std::shared_ptr<ChSensor> pSensor) {
    // to render, the sensor *must* inherit from IRendersWithOptix
    std::shared_ptr<IRendersWithOptix> pOptixSensor = std::dynamic_pointer_cast<IRendersWithOptix>(pSensor);

    unsigned int width = pOptixSensor->RenderWidth();
    unsigned int height = pOptixSensor->RenderHeight();

    optix::Buffer buffer = pOptixSensor->Context()->createBuffer(RT_BUFFER_OUTPUT | RT_BUFFER_COPY_ON_DIRTY,
                                                                 pOptixSensor->RenderBufferFormat(), width, height);
    return buffer;
}

CH_SENSOR_API std::shared_ptr<ChFilterVisualize> ChFilterOptixRender::FindOnlyVisFilter(
    std::shared_ptr<ChSensor> pSensor) {
    int cnt = (int)std::count_if(
        pSensor->FilterList().begin(), pSensor->FilterList().end(),
        [](std::shared_ptr<ChFilter> f) { return std::dynamic_pointer_cast<ChFilterVisualize>(f) != nullptr; });

    if (cnt == 1) {
        auto it = std::find_if(
            pSensor->FilterList().begin(), pSensor->FilterList().end(),
            [](std::shared_ptr<ChFilter> f) { return std::dynamic_pointer_cast<ChFilterVisualize>(f) != nullptr; });
        return std::dynamic_pointer_cast<ChFilterVisualize>(*it);
    }
    return nullptr;
}
}  // namespace sensor
}  // namespace chrono

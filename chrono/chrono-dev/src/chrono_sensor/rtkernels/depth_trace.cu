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
// RT kernels for tracing and measureing depth for a typical pinhole camera
//
// =============================================================================

#include <optixu/optixu_aabb.h>
#include "chrono_sensor/rtkernels/ray_utils.h"

using namespace optix;

rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(uint2, launch_index, rtLaunchIndex, );
rtDeclareVariable(float, scene_epsilon, , );
rtDeclareVariable(rtObject, root_node, , );
rtDeclareVariable(float, default_depth, , );
rtDeclareVariable(float3, default_color, , );

rtBuffer<float, 2> output_buffer;

// This kernel is launched once for each pixel in the image
RT_PROGRAM void depth_trace_camera() {
    size_t2 screen = output_buffer.size();

    // set the ray direction based on the proportion of image the pixel is located at
    float2 d = (make_float2(launch_index) + make_float2(0.5, 0.5)) / make_float2(screen) * 2.f - 1.f;
    // origin of the camera is  0,0,0 for now
    float3 ray_origin = {0, 0, 0};
    float3 ray_direction =
        normalize(float3{1.0, -d.x, d.y});  // flip y value to have image oriented with 0,0 in lower left

    // create a ray based on the calculated parameters
    optix::Ray ray(ray_origin, ray_direction, RADIANCE_RAY_TYPE, scene_epsilon);

    // set the ray pay load
    PerRayData_radiance prd_radiance;
    prd_radiance.importance = 1.f;
    prd_radiance.depth = 0;
    prd_radiance.type = DEPTH;

    // launch the ray
    rtTrace(root_node, ray, prd_radiance);

    // set the output buffer to be what is returned in the payload
    output_buffer[launch_index] = prd_radiance.distance;
}

RT_PROGRAM void miss_function() {
    if (prd_radiance.type == DEPTH) {
        prd_radiance.distance = default_depth;
    } else if (prd_radiance.type == LIDAR) {
    } else if (prd_radiance.type == CAMERA) {
        prd_radiance.color = default_color;
    }
}

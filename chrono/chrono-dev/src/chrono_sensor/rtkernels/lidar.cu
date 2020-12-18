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

#include <math_constants.h>
#include <optixu/optixu_aabb.h>
#include "chrono_sensor/rtkernels/ray_utils.h"

using namespace optix;

rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(uint2, launch_index, rtLaunchIndex, );
rtDeclareVariable(float, scene_epsilon, , );
rtDeclareVariable(float, max_scene_distance, , );
rtDeclareVariable(rtObject, root_node, , );

// camera parameters
rtDeclareVariable(float3, c_pos, , );      // lidar position
rtDeclareVariable(float3, c_up, , );       // lidar up vector
rtDeclareVariable(float3, c_forward, , );  // lidar forward vector
rtDeclareVariable(float3, c_left, , );     // lidar left vector
rtDeclareVariable(float, c_hFOV, , );      // lidar horizontal field of view
rtDeclareVariable(float, c_vFOV, , );      // lidar vertical field of view

rtBuffer<float2, 2> output_buffer;  // byte version

// This kernel is launched once for each pixel in the image
RT_PROGRAM void spherical() {
    size_t2 screen = output_buffer.size();

    // set the ray direction based on the proportion of image the pixel is located at
    float2 d = (make_float2(launch_index) + make_float2(0.5, 0.5)) / make_float2(screen) * 2.f - 1.f;  //[-1,1]

    float theta = d.x * c_hFOV / 2.0;
    float phi = d.y * c_vFOV / 2.0;
    float xy_proj = cos(phi);

    float z = sin(phi);
    float y = xy_proj * sin(theta);
    float x = xy_proj * cos(theta);

    // origin of the camera is  0,0,0 for now
    float3 ray_origin = c_pos;
    float3 ray_direction = normalize(c_forward * x + c_left * y + c_up * z);

    // create a ray based on the calculated parameters
    optix::Ray ray(ray_origin, ray_direction, RADIANCE_RAY_TYPE, scene_epsilon, max_scene_distance);

    // set the ray pay load
    PerRayData_radiance prd_radiance;
    prd_radiance.importance = 1.f;
    prd_radiance.depth = 0;
    prd_radiance.type = LIDAR;

    // launch the ray
    rtTrace(root_node, ray, prd_radiance);

    // set the output buffer to be what is returned in the payload
    output_buffer[launch_index] =
        make_float2(prd_radiance.color.x, prd_radiance.color.y);  // make_color(prd_radiance.color);
}

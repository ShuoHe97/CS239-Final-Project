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
// RT kernels for coloring upon ray not intersecting anything
//
// =============================================================================

#include <math_constants.h>
#include <optixu/optixu_aabb.h>
#include "chrono_sensor/rtkernels/ray_utils.h"

using namespace optix;

rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
// rtDeclareVariable(uint2, launch_index, rtLaunchIndex, );
rtDeclareVariable(float, scene_epsilon, , );
// rtDeclareVariable(float, max_scene_distance, , );
// rtDeclareVariable(rtObject, root_node, , );
rtDeclareVariable(float3, default_color, , );
rtDeclareVariable(float, default_depth, , );

// camera parameters
// rtDeclareVariable(float3, c_pos, , );      // camera position
// rtDeclareVariable(float3, c_up, , );       // camera up vector
// rtDeclareVariable(float3, c_forward, , );  // camera forward vector
// rtDeclareVariable(float3, c_left, , );     // camera left vector
// rtDeclareVariable(float, c_hFOV, , );      // camera horizontal field of view
// rtDeclareVariable(float, c_vFOV, , );      // camera vertical field of view

// environment map
rtTextureSampler<float4, 2> environment_map;
rtDeclareVariable(int, has_environment_map, , );

RT_PROGRAM void miss_function() {
    if (prd_radiance.type == DEPTH) {
        prd_radiance.distance = default_depth;
    } else if (prd_radiance.type == LIDAR) {
        prd_radiance.color = make_float3(default_depth, 0.f, 0.f);
    } else if (prd_radiance.type == CAMERA) {
        if (has_environment_map) {
            float theta = atan2f(ray.direction.x, ray.direction.y);
            float phi = asinf(ray.direction.z);
            float tex_x = theta / (2 * M_PIf);
            float tex_y = phi / (M_PIf) + 0.5;

            prd_radiance.color = make_float3(tex2D(environment_map, tex_x, tex_y));
            // prd_radiance.distance = default_depth;
            // if (prd_radiance.first_distance < 1e-3)
            //     prd_radiance.first_distance = default_depth;
            // if (prd_radiance.first_distance < 1e-3)
            //     prd_radiance.first_distance = default_depth;
        } else {
            prd_radiance.color = default_color;
            // prd_radiance.distance = default_depth;
            if (prd_radiance.first_distance < scene_epsilon)
                prd_radiance.first_distance = default_depth;
            // if (prd_radiance.first_distance < 1e-3)
            //     prd_radiance.first_distance = default_depth;
        }
    }
}

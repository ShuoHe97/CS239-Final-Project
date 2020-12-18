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
// RT kernels for box geometries
//
// =============================================================================
#include <optix.h>
#include <optixu/optixu_aabb_namespace.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h>

#include "chrono_sensor/rtkernels/ray_utils.h"
using namespace optix;

rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );
rtDeclareVariable(float3, boxmin, , );
rtDeclareVariable(float3, boxmax, , );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, t_hit, rtIntersectionDistance, );

rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
rtDeclareVariable(float3, geometric_normal, attribute geometric_normal, );
rtDeclareVariable(float2, texcoord, attribute texcoord, );
rtDeclareVariable(int, has_texture, , );

static __device__ float3 box_normal(float t, float3 t0, float3 t1) {
    float3 normal_pos = make_float3(t == t0.x ? 1 : 0, t == t0.y ? 1 : 0, t == t0.z ? 1 : 0);
    float3 normal_neg = make_float3(t == t1.x ? 1 : 0, t == t1.y ? 1 : 0, t == t1.z ? 1 : 0);
    return normal_pos - normal_neg;
}

RT_PROGRAM void box_intersect(int) {
    // calculate potential intersections with the box
    float3 t0 = (boxmin - ray.origin) / ray.direction;
    float3 t1 = (boxmax - ray.origin) / ray.direction;
    float3 near = fminf(t0, t1);
    float3 far = fmaxf(t0, t1);
    // dist_near and dist_far are the distances to the potential intsection points
    float dist_near = fmaxf(near);
    float dist_far = fminf(far);

    // check if near is less than far
    if (dist_near <= dist_far) {
        bool check_second = true;
        if (rtPotentialIntersection(dist_near)) {
            shading_normal = geometric_normal = box_normal(dist_near, t0, t1);
            float3 p = ray.origin + dist_near * ray.direction;
            float3 scaled =
                (p - boxmin) / (boxmax - boxmin) -
                make_float3(2 * abs(shading_normal.x), 2 * abs(shading_normal.y), 2 * abs(shading_normal.z));
            float u = scaled.x > -.5f ? scaled.x : scaled.y;
            float v = (scaled.x > -.5f && scaled.y > -.5f) ? scaled.y : scaled.z;
            texcoord = make_float2(u, v);
            if (rtReportIntersection(0))
                check_second = false;
        }
        if (check_second) {
            if (rtPotentialIntersection(dist_far)) {
                shading_normal = geometric_normal = box_normal(dist_far, t0, t1);
                float3 p = ray.origin + dist_far * ray.direction;
                float3 scaled = (p - boxmin) / (boxmax - boxmin) -
                                make_float3(abs(shading_normal.x), abs(shading_normal.y), abs(shading_normal.z));
                float u = scaled.x > -.5f ? abs(scaled.x) : abs(scaled.y);
                float v = scaled.x > -.5f ? abs(scaled.y) : abs(scaled.z);
                texcoord = make_float2(u, v);
                rtReportIntersection(0);
            }
        }
    }
}

RT_PROGRAM void box_bounds(int, float result[6]) {
    optix::Aabb* aabb = (optix::Aabb*)result;
    aabb->set(boxmin, boxmax);
}

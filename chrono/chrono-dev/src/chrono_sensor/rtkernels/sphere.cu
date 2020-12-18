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
// RT kernels for sphere geometries
//
// =============================================================================

#include <optix.h>
#include <optixu/optixu_aabb_namespace.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h>

#include "chrono_sensor/rtkernels/ray_utils.h"
using namespace optix;

rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );
rtDeclareVariable(float4, sphere, , );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, t_hit, rtIntersectionDistance, );

rtDeclareVariable(float3, geometric_normal, attribute geometric_normal, );
rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
rtDeclareVariable(float2, texcoord, attribute texcoord, );
rtDeclareVariable(int, has_texture, , );

RT_PROGRAM void sphere_intersect(int) {
    // calculate distances to potential intersections with a sphere (ray-sphere intersection)
    float radius = sphere.w;  // all values the same for convenience
    float3 center = make_float3(sphere.x, sphere.y, sphere.z);

    // calculate the three components of quadratic equation that defines intertersection
    float a = dot(ray.direction, ray.direction);
    float b = 2 * dot((ray.direction), (ray.origin - center));
    float c = dot(ray.origin - center, ray.origin - center) - radius * radius;

    float det = b * b - 4 * a * c;

    if (det >= 0) {
        float dist_near = (-b - sqrtf(b * b - 4 * a * c)) / (2 * a);
        float dist_far = (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);

        if (dist_near < dist_far) {
            bool check_second = true;
            if (rtPotentialIntersection(dist_near)) {
                float3 p = ray.origin + ray.direction * dist_near;
                shading_normal = geometric_normal = p - center;
                texcoord = make_float2(0.0f);
                if (rtReportIntersection(0))
                    check_second = false;
            }
            if (check_second) {
                if (rtPotentialIntersection(dist_far)) {
                    float3 p = ray.origin + ray.direction * dist_far;
                    shading_normal = geometric_normal = p - center;
                    texcoord = make_float2(0.0f);
                    rtReportIntersection(0);
                }
            }
        }
    }
}

RT_PROGRAM void sphere_bounds(int, float result[6]) {
    const float3 center = make_float3(sphere.x, sphere.y, sphere.z);
    const float3 radius = make_float3(sphere.w);

    optix::Aabb* aabb = (optix::Aabb*)result;
    aabb->set(center - radius, center + radius);
}

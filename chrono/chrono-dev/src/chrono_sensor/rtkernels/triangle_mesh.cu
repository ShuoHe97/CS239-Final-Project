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
// RT kernels for mesh geometries
//
// =============================================================================

#include <optix.h>
#include <optixu/optixu_aabb_namespace.h>
#include <optixu/optixu_math_namespace.h>
#include <optixu/optixu_matrix_namespace.h>

#include "chrono_sensor/rtkernels/ray_utils.h"
using namespace optix;

rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float, t_hit, rtIntersectionDistance, );

rtDeclareVariable(float3, geometric_normal, attribute geometric_normal, );
rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
rtDeclareVariable(float2, texcoord, attribute texcoord, );
rtDeclareVariable(int, has_texture, , );

rtBuffer<int3> index_buffer;
rtBuffer<float3> vertex_buffer;
rtBuffer<unsigned int> material_buffer;
rtBuffer<float3> normal_buffer;
rtBuffer<float2> texcoord_buffer;

RT_PROGRAM void mesh_attributes() {
    const int3 vertex_idx = index_buffer[rtGetPrimitiveIndex()];
    const float3 v0 = vertex_buffer[vertex_idx.x];
    const float3 v1 = vertex_buffer[vertex_idx.y];
    const float3 v2 = vertex_buffer[vertex_idx.z];

    geometric_normal = normalize(cross(v1 - v0, v2 - v0));

    const float2 bary_coord = rtGetTriangleBarycentrics();

    if (normal_buffer.size() == 0) {
        shading_normal = geometric_normal;
    } else {
        shading_normal =
            normalize(normal_buffer[vertex_idx.y] * bary_coord.x + normal_buffer[vertex_idx.z] * bary_coord.y +
                      normal_buffer[vertex_idx.x] * (1.0f - bary_coord.x - bary_coord.y));
    }
    if (texcoord_buffer.size() == 0) {
        texcoord = make_float2(0.0f);
    } else {
        texcoord = texcoord_buffer[vertex_idx.y] * bary_coord.x + texcoord_buffer[vertex_idx.z] * bary_coord.y +
                   texcoord_buffer[vertex_idx.x] * (1.0f - bary_coord.x - bary_coord.y);
    }
}

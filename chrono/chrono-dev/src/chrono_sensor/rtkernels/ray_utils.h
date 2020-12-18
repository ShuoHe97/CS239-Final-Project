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
// =============================================================================

#define RADIANCE_RAY_TYPE 0
#define SHADOW_RAY_TYPE 1

//
enum RayType {
    DEPTH,  // denotes ray is for simply calculating a depth for debugging
    LIDAR,  // denotes ray is for calculating lidar data
    CAMERA  // denotes ray is for calculating camera data
};

struct PerRayData_radiance {
    float3 color;
    float importance;
    int depth;
    float distance;
    RayType type;
    float first_distance;
};

static __device__ __inline__ PerRayData_radiance make_radiance_data(const float3 r_color,
                                                                    const float r_importance,
                                                                    const int r_depth,
                                                                    const float r_distance,
                                                                    const float r_first_distance,
                                                                    const RayType r_type) {
    //
    PerRayData_radiance ray_data;
    ray_data.color = r_color;
    ray_data.importance = r_importance;
    ray_data.depth = r_depth;
    ray_data.distance = r_distance;
    ray_data.first_distance = r_first_distance;
    ray_data.type = r_type;
    return ray_data;
}

struct PerRayData_shadow {
    float3 attenuation;
};

static __device__ __inline__ PerRayData_shadow make_radiance_data(const float3 r_attenuation) {
    //
    PerRayData_shadow ray_data;
    ray_data.attenuation = r_attenuation;
    return ray_data;
}

static __device__ __inline__ uchar4 make_color(const float3& c) {
    return make_uchar4(static_cast<unsigned char>(__saturatef(c.x) * 255.9999f),
                       static_cast<unsigned char>(__saturatef(c.y) * 255.9999f),
                       static_cast<unsigned char>(__saturatef(c.z) * 255.9999f), 255);
}

static __device__ __inline__ float4 make_float4(const float3& point, const uchar4& color) {
    // unsigned char ca[] = {color.w, color.x, color.y, color.z};
    float csf;
    memcpy(&csf, &color, sizeof(csf));

    return make_float4(point.x, point.y, point.z, csf);
}

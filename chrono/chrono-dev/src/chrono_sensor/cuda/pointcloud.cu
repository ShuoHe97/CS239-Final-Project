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

#include <cuda.h>
#include "pointcloud.cuh"

namespace chrono {
namespace sensor {

// Converts 32bpp ARGB imgIn pixels to 8bpp Grayscale imgOut pixels
__global__ void pointcloud_from_depth_kernel(float* imgIn, float* imgOut, int numPixels, LidarParams params) {
    int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index < numPixels) {
        int hIndex = index % params.horizontal_samples;
        int vIndex = index / params.horizontal_samples;

        float vAngle = (vIndex / (float)(params.vertical_samples)) * params.vFOV - params.vFOV / 2.;
        float hAngle = (hIndex / (float)(params.horizontal_samples)) * params.hFOV - params.hFOV / 2.;

        float range = imgIn[2 * index];

        float proj_xy = range * cos(vAngle);

        float x = proj_xy * cos(hAngle);
        float y = proj_xy * sin(hAngle);
        float z = range * sin(vAngle);
        imgOut[4 * index] = x;
        imgOut[4 * index + 1] = y;
        imgOut[4 * index + 2] = z;
        imgOut[4 * index + 3] = imgIn[2 * index + 1];
    }
}

void cuda_pointcloud_from_depth(void* bufDI, void* bufOut, int width, int height, LidarParams params) {
    int numPixels = width * height;
    const int nThreads = 512;
    int nBlocks = (numPixels + nThreads - 1) / nThreads;

    pointcloud_from_depth_kernel<<<nBlocks, nThreads>>>((float*)bufDI, (float*)bufOut, numPixels, params);
}

}  // namespace sensor
}  // namespace chrono

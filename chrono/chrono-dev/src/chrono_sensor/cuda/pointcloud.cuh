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

namespace chrono {
namespace sensor {

struct LidarParams {
    float hFOV;
    float vFOV;
    int horizontal_samples;
    int vertical_samples;
};

void cuda_pointcloud_from_depth(void* bufDI, void* bufOut, int width, int height, LidarParams params);
}  // namespace sensor
}  // namespace chrono

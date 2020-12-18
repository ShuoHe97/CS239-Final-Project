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
// utils file translating Chrono assets to higher quality assets
//
// =============================================================================

#ifndef CHVISUALMATERIALUTILS_H
#define CHVISUALMATERIALUTILS_H

#include <string>
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

void CreateModernMeshAssets(std::shared_ptr<ChTriangleMeshShape> mesh_shape);
void ConvertToModernAssets(std::shared_ptr<ChBody> body);
void ConvertToModernAssets(ChSystem* sys);

}  // namespace sensor
}  // namespace chrono

#endif

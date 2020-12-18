// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// A simplified M113 driveline.
//
// =============================================================================

#include "chrono_models/vehicle/m113a/M113a_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113a_SimpleDriveline::m_diff_maxBias = 1;  //// 3;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_SimpleDriveline::M113a_SimpleDriveline() : ChSimpleTrackDriveline("M113a_SimpleDriveline") {}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

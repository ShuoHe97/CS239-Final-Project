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
// Generic rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/generic/Generic_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Generic_RackPinion::m_steeringLinkMass = 1.889;
const ChVector<> Generic_RackPinion::m_steeringLinkInertia(.138, 0.00009, .138);
const double Generic_RackPinion::m_steeringLinkCOM = 0;
//const double Generic_RackPinion::m_steeringLinkLength = 0.896;
const double Generic_RackPinion::m_steeringLinkLength = 0.8;
const double Generic_RackPinion::m_steeringLinkRadius = 0.03;

const double Generic_RackPinion::m_pinionRadius = 0.03;

const double Generic_RackPinion::m_maxAngle = 0.1 / 0.03; //0.1m travel;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_RackPinion::Generic_RackPinion(const std::string& name) : ChRackPinion(name) {}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

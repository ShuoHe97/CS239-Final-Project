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
// HMMWV chassis subsystem.
//
// =============================================================================

#ifndef HMMWV_CHASSIS_H
#define HMMWV_CHASSIS_H

#include <string>

#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// HMMWV chassis subsystem.
class CH_MODELS_API HMMWV_Chassis : public ChRigidChassis {
  public:
    HMMWV_Chassis(const std::string& name,
                  bool fixed = false,
                  ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);
    ~HMMWV_Chassis() {}

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }
  
    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }
  
    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  protected:
    ChMatrix33<> m_inertia;

    static const double m_mass;
    static const ChVector<> m_inertiaXX;
    static const ChVector<> m_inertiaXY;
    static const ChVector<> m_COM_loc;
    static const ChCoordsys<> m_driverCsys;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif

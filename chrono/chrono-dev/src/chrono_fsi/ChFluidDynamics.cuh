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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Class for performing time integration in fluid system.
//
// =============================================================================

#ifndef CH_FLUIDDYNAMICS_H_
#define CH_FLUIDDYNAMICS_H_

#include "chrono_fsi/ChFsiForce.cuh"

#include "chrono_fsi/ChSphGeneral.cuh"

#include "chrono_fsi/ChDeviceUtils.cuh"

#include "chrono_fsi/ChFsiForceExplicitSPH.cuh"
#include "chrono_fsi/ChFsiForceIISPH.cuh"

#include "chrono_fsi/ChFsiDataManager.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief Class to represent the fluid dynamics system.
///
/// This class is used to represent a fluid system and take
/// care of the time integration of the fluid dynamics. This is
/// a class designed for base SPH simulation.
///
/// The class holds pointer to data, which is hold somewhere else.
/// It also include a forceSystem, which takes care of the
/// computation of force between markers. The forceSystem is owned
/// by the class.
class CH_FSI_API ChFluidDynamics : public ChFsiGeneral {
  public:
    /// SPH implementation
    enum class Integrator { IISPH, ExplicitSPH };

    /// Fluid dynamics class constructor.
    ///
    /// The class constructor performs the following operations:
    /// Instantiate ChFsiForce, i.e. force system;
    /// Copy the pointer to fluid data, parameters, and number of objects
    /// to member variables.
    ChFluidDynamics(ChBce* otherBceWorker,
                    ChFsiDataManager* otherFsiData,
                    SimParams* otherParamsH,
                    NumberOfObjects* otherNumObjects,
                    ChFluidDynamics::Integrator type);

    /// Class destructor. Deletes the force system.
    ~ChFluidDynamics();

    /// Integrate the fluid system in time. The underlying SPH method implementation
    /// goes inside this function.
    /// In a explicit scheme, to perform the integration, the force system
    /// calculates the forces between the particles. Then the forces are used to
    /// to update the markers position, velocity, and density in time, the latter
    /// is used to update the pressure from an equation of state. In the
    /// implicit scheme, the pressures are updated instead of density.

    virtual void IntegrateSPH(SphMarkerDataD* sphMarkersD2,
                              SphMarkerDataD* sphMarkersD1,
                              FsiBodiesDataD* fsiBodiesD,
                              FsiMeshDataD* fsiMeshD,
                              Real dT);

    /// Function to Shepard Filtering.
    /// It calculates the densities directly, not based on the derivative of
    /// the density. This function is used in addition to the density update
    /// in UpdateFluid.
    virtual void DensityReinitialization();

    /// Synchronize the copy of the data (parameters and number of objects)
    /// between device (GPU) and host (CPU).
    /// This function needs to be called once the host data are modified
    void Finalize();
    ChFluidDynamics::Integrator GetIntegratorType() { return myIntegrator; }

    ChFsiForce* GetForceSystem() { return forceSystem; }

  protected:
    ChFsiDataManager* fsiData;  ///< pointer to the fsi data. The values are maintained externally

    ChFsiForce* forceSystem;                   ///< force system object. It calculates the force between markers.
    ChFluidDynamics::Integrator myIntegrator;  ///< IISPH by default

    SimParams* paramsH;  ///< pointer to parameters. The values are mainained externally.

    NumberOfObjects* numObjectsH;  ///< pointer to number of objects (fluid markers, number of rigids, boundaries)

    /// Update SPH markers data.
    /// In an explicit formulation, the update Fluid function relies on explicit Euler
    /// Integration argorithm.
    virtual void UpdateFluid(SphMarkerDataD* sphMarkersD, Real dT);
    virtual void UpdateFluid_Implicit(SphMarkerDataD* sphMarkersD);

    /// Apply boundary to SPH markers (fluid and BCE).
    /// The function applies periodic boundary to the markers.
    virtual void ApplyBoundarySPH_Markers(SphMarkerDataD* sphMarkersD);
    virtual void ApplyModifiedBoundarySPH_Markers(SphMarkerDataD* sphMarkersD);
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif

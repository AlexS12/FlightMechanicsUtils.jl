module FlightMechanicsUtils

using LinearAlgebra
using StaticArrays


export γ_AIR, R_AIR
export gD
export RAD2DEG, DEG2RAD, M2FT, FT2M, KEL2RANK, RANK2KEL, KG2LB, LB2KG, SLUG2KG, KG2SLUG
export PSF2PA, PA2PSF, SLUGFT32KGM3, KGM32SLUGFT3
include("constants.jl")

export atmosphere_isa
include("atmosphere.jl")

export rigid_body_velocity, rigid_body_acceleration
export pqr_2_ψθϕ_dot, pqr_2_quat_dot, ψθϕ_dot_2_pqr
export uvw_to_tasαβ, uvw_dot_to_tasαβ_dot, tasαβ_dot_to_uvw_dot
export rate_of_climb_constrain_no_wind
include("kinematics.jl")

export coordinated_turn_bank, steiner_inertia, translate_forces_moments
include("mechanics.jl")

export euler_angles, quaternions, rotation_matrix_zyx
export body2horizon, body2wind
export wind2body, wind2horizon
export horizon2body, horizon2wind
include("rotations.jl")

export Ellipsoid
export Clarke1866, Clarke1880, International, Bessel, Everest, ModifiedEverest,
    AustralianNational, SouthAmerican1969, Airy, ModifiedAiry, Hough, Fischer1960SouthAsia,
    Fischer1960Mercury, Fischer1968, WGS60, WGS66, WGS72, WGS84
export llh2ecef, ecef2llh
include("ellipsoid.jl")

export qc2cas, qc2eas, qc2tas
export cas2eas, cas2tas
export eas2cas, eas2tas
export tas2cas, tas2eas
export compressible_qinf, incompressible_qinf
include("anemometry.jl")

end

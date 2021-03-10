module FlightMechanicsUtils

export γ_AIR, R_AIR, gD
include("constants.jl")

export atmosphere_isa
include("atmosphere.jl")

export rigid_body_velocity, rigid_body_acceleration
include("kinematics.jl")

export euler_angles,
    quaternions,
    body2horizon,
    horizon2body,
    wind2horizon,
    horizon2wind,
    body2wind,
    wind2body,
    rotation_matrix_zyx
include("rotations.jl")

end

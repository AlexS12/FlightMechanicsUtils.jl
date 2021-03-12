module FlightMechanicsUtils

export γ_AIR, R_AIR, gD
include("constants.jl")

export atmosphere_isa
include("atmosphere.jl")

export rigid_body_velocity,
    rigid_body_acceleration,
    body_angular_velocity_to_euler_angles_rates,
    euler_angles_rates_to_body_angular_velocity,
    body_angular_velocity_to_quaternion_rates,
    uvw_to_tasαβ,
    uvw_dot_to_tasαβ_dot,
    tasαβ_dot_to_uvw_dot
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

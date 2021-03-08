module FlightMechanicsUtils

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

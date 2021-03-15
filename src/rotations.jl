"""
    rotation_matrix_zyx(α1, α2, α3)

Calculate the rotation matrix ``R_{ab}`` that transforms a vector in b frame to a frame
(``v_{a} = R_{ab} v_{b}``) given the rotations (α1, α2, α3) (rad).

Frame b is obtained from frame a performing three intrinsic rotations of magnitude
α1, α2 and α3 in ZYX order.
"""
function rotation_matrix_zyx(α1, α2, α3)

    sα1, cα1 = sin(α1), cos(α1)
    sα2, cα2 = sin(α2), cos(α2)
    sα3, cα3 = sin(α3), cos(α3)

    R = @SMatrix [
        cα2 * cα1   (sα3 * sα2 * cα1 - cα3 * sα1)   (cα3 * sα2 * cα1 + sα3 * sα1)
        cα2 * sα1   (sα3 * sα2 * sα1 + cα3 * cα1)   (cα3 * sα2 * sα1 - sα3 * cα1)
        -sα2        sα3 * cα2                       cα3 * cα2
    ]

    return R
end


"""
    rotation_matrix_zyx(q0, q1, q2, q3)

Calculate the rotation matrix ``R_{ab}`` that transforms a vector in b frame to a frame
(``v_{a} = R_{ab} v_{b}``) given the quaternions ``q_0, q_1, q_2, q_3``.
"""
function rotation_matrix_zyx(q0, q1, q2, q3)

    q02, q12, q22, q32 = q0*q0, q1*q1, q2*q2, q3*q3

    R = @SMatrix [
        (q02+q12-q22-q32)    2*(q1*q2 - q0*q3)    2*(q1*q3 + q0*q2)
        2*(q1*q2 + q0*q3)    (q02-q12+q22-q32)    2*(q2*q3 - q0*q1)
        2*(q1*q3 - q0*q2)    2*(q2*q3 + q0*q1)    (q02-q12-q22+q32)
    ]

    return R
end


"""
    quaternions(ψ, θ, ϕ)

Calculate quaternion representation given the Euler angles (ψ, θ, ϕ) (rad).
"""
function quaternions(ψ, θ, ϕ)

    s_ψ2, c_ψ2 = sin(ψ/2), cos(ψ/2)
    s_θ2, c_θ2 = sin(θ/2), cos(θ/2)
    s_ϕ2, c_ϕ2 = sin(ϕ/2), cos(ϕ/2)

    q0 = c_ψ2*c_θ2*c_ϕ2 + s_ψ2*s_θ2*s_ϕ2
    q1 = c_ψ2*c_θ2*s_ϕ2 - s_ψ2*s_θ2*c_ϕ2
    q2 = c_ψ2*s_θ2*c_ϕ2 + s_ψ2*c_θ2*s_ϕ2
    q3 = s_ψ2*c_θ2*c_ϕ2 - c_ψ2*s_θ2*s_ϕ2

    return @SVector [q0, q1, q2, q3]
end


"""
    euler_angles(q0, q1, q2, q3)

Calculate Euler angles (ψ, θ, ϕ) (rad) given the quaternions ``q_0, q_1, q_2, q_3``.
"""
function euler_angles(q0, q1, q2, q3)

    ψ = atan(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)
    θ = asin(-2 * (q1*q3 - q0*q2))
    ϕ = atan(2 * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3)

    return @SVector [mod2pi(ψ), θ, ϕ]
end


"""
    body2horizon(x, y, z, ψ, θ, ϕ)

Transform the vector coordintes (x, y, z) given in body axis to local horizon
given the Euler angles (ψ, θ, ϕ) (rad).
"""
function body2horizon(x, y, z, ψ, θ, ϕ)
    v = @SVector [x, y, z]
    rv = rotation_matrix_zyx(ψ, θ, ϕ) * v
    return rv
end


"""
    horizon2body(x, y, z, ψ, θ, ϕ)

Transform the vector coordintes (x, y, z) given in local horizon axis to body
given the Euler angles (ψ, θ, ϕ) (rad).
"""
function horizon2body(x, y, z, ψ, θ, ϕ)
    v = @SVector [x, y, z]
    rv = transpose(rotation_matrix_zyx(ψ, θ, ϕ)) * v
    return rv
end


"""
    body2horizon(x, y, z, q0, q1, q2, q3)

Transform the vector coordintes (x, y, z) given in body axis to local horizon
given the quaternions ``q_0, q_1, q_2, q_3``.
"""
function body2horizon(x, y, z, q0, q1, q2, q3)
    v = @SVector [x, y, z]
    rv = rotation_matrix_zyx(q0, q1, q2, q3) * v
    return rv
end


"""
    horizon2body(x, y, z, q0, q1, q2, q3)

Transform the vector coordintes (x, y, z) given in local horizon axis to body
given the quaternions ``q_0, q_1, q_2, q_3``.
"""
function horizon2body(x, y, z, q0, q1, q2, q3)
    v = @SVector [x, y, z]
    rv = transpose(rotation_matrix_zyx(q0, q1, q2, q3)) * v
    return rv
end


"""
    wind2body(x, y, z, α, β)

Transform the vector coordintes (x, y, z) given in wind axis to body
given the angle of attack (α) and the angle of sideslip (β) (rad).
"""
function wind2body(x, y, z, α, β)
    v = @SVector [x, y, z]
    rv = transpose(rotation_matrix_zyx(-β, α, 0)) * v
    return rv
end


"""
    body2wind(x, y, z, α, β)

Transform the vector coordintes (x, y, z) given in body axis to wind
given the angle of attack (α) and the angle of sideslip (β) (rad).
"""
function body2wind(x, y, z, α, β)
    v = @SVector [x, y, z]
    rv = rotation_matrix_zyx(-β, α, 0) * v
    return rv
end


"""
    wind2horizon(x, y, z, χ, γ, μ)

Transform the vector coordintes (x, y, z) given in wind axis to local horizon
given the velocity angles (χ, γ, μ) (rad).
"""
function wind2horizon(x, y, z, χ, γ, μ)
    v = @SVector [x, y, z]
    rv = rotation_matrix_zyx(χ, γ, μ) * v
    return rv
end


"""
    horizon2wind(x, y, z, χ, γ, μ)

Transform the vector coordintes (x, y, z) given in local horizon axis to wind
given the velocity angles (χ, γ, μ) (rad).
"""
function horizon2wind(x, y, z, χ, γ, μ)
    v = @SVector [x, y, z]
    rv = transpose(rotation_matrix_zyx(χ, γ, μ)) * v
    return rv
end

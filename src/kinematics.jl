using LinearAlgebra


@doc raw"""
    rigid_body_velocity(vel_P, ω, r_PQ)

Calculate rigid solid velocity field.

Return velocity of a point Q of a rigid solid given the velocity of a
point P (vel_P), the rotational velocity of the solid (ω) and the relative
position of Q with respect to P.

If the reference frame 1 is attached to the solid and the velocity is
calculated with respect to reference frame 0:

``v_{10}^{Q} = v_{10}^{P} + \omega_{10} \times r^{PQ}``

being:
- ``v_{10}^{Q}`` the velocity of point Q, fixed to 1, wrt 0
- ``\omega_{10}`` the angular velocity of the solid 1 wrt 0
- ``r^{PQ}`` the position of Q wrt P (``r^{Q}-r^{P}``)

Every vector needs to be expressed in the same coordinate system.

# References

1. Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation: dynamics, controls design, and autonomous systems. John Wiley & Sons. (Section 1.3, page 26)
"""
function rigid_body_velocity(vel_P, ω, r_PQ)
    vel_Q = vel_P + ω × r_PQ
    return vel_Q
end


@doc raw"""
    rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)

Calcualte rigid body acceleration field.

Return the acceleration of a point Q of a rigid solid given the acceleration
of a point P (acc_P), the rotational velocity of the solid (ω), the rotational
acceleration of the solid (ω_dot) and the relative position of Q with respect 
to P.

``a_{10}^{Q} = a_{10}^{P} + \omega_{10} \times (\omega_{10} \times r^{PQ}) + \dot{\omega}_{10} \times r^{PQ}``

being:
- ``a_{10}^{Q}`` the acceleration of point Q, fixed to 1, wrt 0
- ``\omega_{10}`` the angular velocity of the solid 1 wrt 0
- ``\dot{\omega}_{10}`` the angular acceleration of the solid 1 wrt 0
- ``r^{PQ}`` the position of Q wrt P (``r^{Q}-r^{P}``)

# References

1. Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation: dynamics, controls design, and autonomous systems. John Wiley & Sons. (Section 1.3, Formaula 1.3-14c, page 26)
"""
function rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)
    acc_Q = acc_P + ω × (ω × r_PQ) + ω_dot × r_PQ
    return acc_Q
end


"""
    body_angular_velocity_to_euler_angles_rates(p, q, r, ψ, θ, ϕ)

Transform body angular velocity (p, q, r) [rad/s] to Euler angles rates
(ψ_dot, θ_dot, ϕ_dot) [rad/s] given the euler angles (θ, ϕ) [rad] using
kinematic angular equations.

# See also

[`euler_angles_rates_to_body_angular_velocity`](@ref), [`body_angular_velocity_to_quaternion_rates`](@ref)

# References

1. Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control and simulation: dynamics, controls design, and autonomous systems. John Wiley & Sons. Equation (1.4-4) (page 20)
"""
function body_angular_velocity_to_euler_angles_rates(p, q, r, θ, ϕ)

    sθ, cθ = sin(θ), cos(θ)
    sϕ, cϕ = sin(ϕ), cos(ϕ)

    ψ_dot = (q * sϕ + r * cϕ) / cθ
    θ_dot = q * cϕ - r * sϕ
    # ϕ_dot = p + (q * sϕ + r * cϕ) * tan(θ)
    ϕ_dot = p + ψ_dot * sθ

    return [ψ_dot, θ_dot, ϕ_dot]
end


"""
    euler_angles_rates_to_body_angular_velocity(ψ_dot, θ_dot, ϕ_dot, ψ, θ, ϕ)

Transform Euler angles rates (ψ_dot, θ_dot, ϕ_dot) [rad/s] to body angular
velocity (p, q, r) [rad/s] given the euler angles (θ, ϕ) [rad] using
kinematic angular equations.

# See also

[`body_angular_velocity_to_euler_angles_rates`](@ref)

# References

1. Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control and simulation: dynamics, controls design, and autonomous systems. John Wiley & Sons. Equation (1.4-3) (page 20)
"""
function euler_angles_rates_to_body_angular_velocity(ψ_dot, θ_dot, ϕ_dot, θ, ϕ)

    sθ, cθ = sin(θ), cos(θ)
    sϕ, cϕ = sin(ϕ), cos(ϕ)

    p = ϕ_dot - sθ * ψ_dot
    q = cϕ * θ_dot + sϕ*cθ * ψ_dot
    r = -sϕ * θ_dot + cϕ*cθ * ψ_dot

    return [p, q, r]
end


"""
    body_angular_velocity_to_quaternion_rates(p, q, r, q0, q1, q2, q3)

Transform body angular velocity (p, q, r) [rad/s] to quaternion rates [1/s].

# See also

[`body_angular_velocity_to_euler_angles_rates`](@ref)

# References

1. Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control and simulation: dynamics, controls design, and autonomous systems. John Wiley & Sons. Equation (1.8-15) (page 51)

"""
function body_angular_velocity_to_quaternion_rates(p, q, r, q0, q1, q2, q3)

    Ω = [
        0 -p -q -r;
        p  0  r -q;
        q -r  0  p;
        r  q -p  0;
    ]
    q = [q0; q1; q2; q3]
    q_dot = 0.5 * Ω * q

    return q_dot
end

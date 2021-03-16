using FlightMechanicsUtils
using Test


@testset "Rigid body vel and accel fields" begin
    # TEST rigid_body_velocity
    # Test without angular velocity
    vel_P = [10.0, 0.0, 0.0]
    ω = [0.0, 0.0, 0.0]
    r_PQ = [5.0, 5.0, 5.0]

    exp_vel_Q = [10.0, 0.0, 0.0]
    vel_Q = rigid_body_velocity(vel_P, ω, r_PQ)
    @test isapprox(vel_Q, exp_vel_Q)

    # Only angular velocity in Z axis
    vel_P = [0.0, 0.0, 0.0]
    ω = [0.0, 0.0, 1.0]
    r_PQ = [5.0, 5.0, 5.0]

    exp_vel_Q = [-sqrt(50) * sin(pi/4), sqrt(50) * cos(pi/4), 0.0]
    vel_Q = rigid_body_velocity(vel_P, ω, r_PQ)
    @test isapprox(vel_Q, exp_vel_Q)

    # X linear velocity and Z angular velocity
    vel_P = [10.0, 0.0, 0.0]
    ω = [0.0, 0.0, 1.0]
    r_PQ = [5.0, 5.0, 5.0]

    exp_vel_Q = [-sqrt(50) * sin(pi/4) + 10.0, sqrt(50) * cos(pi/4), 0.0]
    vel_Q = rigid_body_velocity(vel_P, ω, r_PQ)
    @test isapprox(vel_Q, exp_vel_Q)

    # TEST rigid_body_acceleration
    # Only X linear acceleration
    acc_P = [10.0, 0.0, 0.0]
    ω = [0.0, 0.0, 0.0]
    ω_dot = [0.0, 0.0, 0.0]
    r_PQ = [5.0, 5.0, 5.0]

    exp_accel_Q = [10.0, 0.0, 0.0]
    accel_Q = rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)
    @test isapprox(accel_Q, exp_accel_Q)

    # Only Z angular acceleration
    acc_P = [0.0, 0.0, 0.0]
    ω = [0.0, 0.0, 0.0]
    ω_dot = [0.0, 0.0, 1.0]
    r_PQ = [5.0, 5.0, 5.0]

    exp_accel_Q = [-sqrt(50) * sin(pi/4), sqrt(50) * cos(pi/4), 0.0]
    accel_Q = rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)
    @test isapprox(accel_Q, exp_accel_Q)

    # Only Z angular velocity
    acc_P = [0.0, 0.0, 0.0]
    ω = [0.0, 0.0, 2.0]
    ω_dot = [0.0, 0.0, 0.0]
    r_PQ = [5.0, 5.0, 5.0]

    exp_accel_Q = [-4.0 * sqrt(50) * sin(pi/4), -4.0 * sqrt(50) * cos(pi/4), 0.0]
    accel_Q = rigid_body_acceleration(acc_P, ω, ω_dot, r_PQ)
    @test isapprox(accel_Q, exp_accel_Q)
end


@testset "Angular kinematic equations" begin
    # Null Euler angles
    pqr = [0.5, 0.3, 0.7]
    euler_angles_rates = pqr_2_ψθϕ_dot(pqr..., 0, 0)
    @test isapprox(euler_angles_rates, pqr[end:-1:1])

    ψθϕ_dot = [0.5, 0.3, 0.7]
    pqr_ = ψθϕ_dot_2_pqr(ψθϕ_dot..., 0, 0)
    @test isapprox(pqr_, ψθϕ_dot[end:-1:1])

    # Reciprocal relationships
    θ = 0.15  # rad
    ϕ = 0.6  # rad
    pqr = [0.5, 0.3, 0.7]
    euler_angles_rates = pqr_2_ψθϕ_dot(pqr..., θ, ϕ)
    pqr_ = ψθϕ_dot_2_pqr(euler_angles_rates..., θ, ϕ)
    @test isapprox(pqr, pqr_)

    # TODO: test body_angular_velocity_to_quaternion_rates
end


@testset "Aero <--> Body velocity" begin
    uvw = [30, 3, 5]
    tas, α, β = uvw_to_tasαβ(30, 3, 5)
    u, v, w = wind2body(tas, 0, 0, α, β)
    @test isapprox([u, v, w], uvw)

    uvw = [100, 0, 0]  # m/s
    rv = uvw_to_tasαβ(uvw...)
    @test isapprox(rv, [100, 0, 0])

    uvw = 10, 0, 10  # m/s
    rv = uvw_to_tasαβ(uvw...)
    @test isapprox(rv, [sqrt(200), atan(1), 0])

    uvw = [10, 10, 0]  # m/s
    rv = uvw_to_tasαβ(uvw...)
    @test isapprox(rv, [sqrt(200), 0, asin(10/sqrt(200))])

    uvw_dot = [5, 1, 10]
    tasαβ_dot = uvw_dot_to_tasαβ_dot(uvw..., uvw_dot...)
    uvw_dot_ = tasαβ_dot_to_uvw_dot(uvw_to_tasαβ(uvw...)..., tasαβ_dot...)
    @test isapprox(uvw_dot_, uvw_dot)
end


@testset "Rate of climb constrain" begin
    # wings-level, no sideslip: θ = α + γ
    θ = rate_of_climb_constrain_no_wind(0.1, 0.05, 0, 0)
    @test isapprox(θ, 0.1 + 0.05)

    # Check case:  Stevens, B. L., Lewis, F. L., (1992).
    # Aircraft control and simulation: dynamics, controls design, and autonomous systems.
    # John Wiley & Sons. (Section 3.6, figure 3.6-2, page 192)
    θ =rate_of_climb_constrain_no_wind(0.0, deg2rad(13.7), deg2rad(0.0292), deg2rad(78.3))
    # Take into account that this check case is a full trim not just the kinematic realtionship
    @test isapprox(rad2deg(θ), 2.87, rtol=0.01)
end

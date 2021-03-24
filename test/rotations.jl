using FlightMechanicsUtils
using LinearAlgebra
using Test


@testset "quaternion <-> euler" begin
    quat = [0.8660254037844387, 0.0, 0.5, 0.0]
    euler_exp = [0.0, 1.04719755, 0.0]

    euler = euler_angles(quat...)
    @test isapprox(euler, euler_exp)

    quat_exp = [quat...]
    quat = quaternions(euler_exp...)
    @test isapprox(quat, quat_exp)

    quat = [0.5, 0.5, 0.0, 0.0]
    euler_exp = [0.0, 0.0, pi/2.0]

    euler = euler_angles(quat...)
    @test isapprox(euler, euler_exp)

    quat_exp = [quat...]
    quat = quaternions(euler_exp...)
    @test isapprox(quat, quat_exp / norm(quat_exp))

    psi, theta, phi = pi/4.0, pi/6.0, pi/12.0
    quat = quaternions(psi, theta, phi)
    euler = euler_angles(quat...)
    @test isapprox([psi, theta, phi], euler)

    quat = [0.5, 0.1, 0.2, 0.7]
    quat = quat / norm(quat)
    euler = euler_angles(quat...)
    quat3 = quaternions(euler...)
    @test isapprox(quat, quat3)
end


@testset "body <-> horizon" begin
    ones_ = [1.0, 1.0, 1.0]

    # body2horizon Euler
    # no rotation
    @test ones_ ≈ body2horizon(ones_..., 0., 0., 0.)

    angles1 = [0., 45*pi/180., 0.]
    angles2 = [0., 0., 45*pi/180]
    angles3 = [45*pi/180, 0., 0.]
    exp_b2h_1 = [2*0.70710678118654757, 1, 0]
    exp_b2h_2 = [1, 0, 2*0.70710678118654757]
    exp_b2h_3 = [0, 2*0.70710678118654757, 1]

    @test exp_b2h_1 ≈ body2horizon(ones_..., angles1...)
    @test exp_b2h_2 ≈ body2horizon(ones_..., angles2...)
    @test exp_b2h_3 ≈ body2horizon(ones_..., angles3...)

    # horizon2body Euler
    @test ones_ ≈ horizon2body(ones_..., 0., 0., 0.)
    @test ones_ ≈ horizon2body(exp_b2h_1..., angles1...)
    @test ones_ ≈ horizon2body(exp_b2h_2..., angles2...)
    @test ones_ ≈ horizon2body(exp_b2h_3..., angles3...)

    # body2horizon quaternion
    @test ones_ ≈ body2horizon(ones_..., 0., 0., 0.)

    quat1 = quaternions(angles1...)
    quat2 = quaternions(angles2...)
    quat3 = quaternions(angles3...)

    @test exp_b2h_1 ≈ body2horizon(ones_..., quat1...)
    @test exp_b2h_2 ≈ body2horizon(ones_..., quat2...)
    @test exp_b2h_3 ≈ body2horizon(ones_..., quat3...)

    # horizon2body quaternionr
    @test ones_ ≈ horizon2body(ones_..., 0., 0., 0.)
    @test ones_ ≈ horizon2body(exp_b2h_1...,  quat1...)
    @test ones_ ≈ horizon2body(exp_b2h_2...,  quat2...)
    @test ones_ ≈ horizon2body(exp_b2h_3...,  quat3...)

    # rotation matrix body to hor (euler)
    r_hb1 = rotation_matrix_zyx(angles1...)
    @test exp_b2h_1 ≈ r_hb1 * ones_
    r_hb2 = rotation_matrix_zyx(angles2...)
    @test exp_b2h_2 ≈ r_hb2 * ones_
    r_hb3 = rotation_matrix_zyx(angles3...)
    @test exp_b2h_3 ≈ r_hb3 * ones_

    # rotation matrix body <-> hor (quaternion)
    r_hb1q = rotation_matrix_zyx(quat1...)
    @test exp_b2h_1 ≈ r_hb1q * ones_
    r_hb2q = rotation_matrix_zyx(quat2...)
    @test exp_b2h_2 ≈ r_hb2q * ones_
    r_hb3q = rotation_matrix_zyx(quat3...)
    @test exp_b2h_3 ≈ r_hb3q * ones_

    # r_hb equivalent with euler and quaternion
    @test isapprox(r_hb1, r_hb1q)
    @test isapprox(r_hb2, r_hb2q)
    @test isapprox(r_hb3, r_hb3q)

    # rotation matrix hor to body (euler)
    r_bh1 = transpose(rotation_matrix_zyx(angles1...))
    @test ones_ ≈ r_bh1 * exp_b2h_1
    r_bh2 = transpose(rotation_matrix_zyx(angles2...))
    @test ones_ ≈ r_bh2 * exp_b2h_2
    r_bh3 = transpose(rotation_matrix_zyx(angles3...))
    @test ones_ ≈ r_bh3 * exp_b2h_3

    # rotation matrix horizon2body (quaternion)
    r_bh1q = transpose(rotation_matrix_zyx(quat1...))
    @test ones_ ≈ r_bh1q * exp_b2h_1
    r_bh2q = transpose(rotation_matrix_zyx(quat2...))
    @test ones_ ≈ r_bh2q * exp_b2h_2
    r_bh3q = transpose(rotation_matrix_zyx(quat3...))
    @test ones_ ≈ r_bh3q * exp_b2h_3

    # r_bh equivalent with euler and quaternion
    @test isapprox(r_bh1, r_bh1q)
    @test isapprox(r_bh2, r_bh2q)
    @test isapprox(r_bh3, r_bh3q)

    # Test that quaternion and euler produce the same transformation
    psi, theta, phi = pi/4.0, pi/6.0, pi/12.0
    quat = quaternions(psi, theta, phi)
    xh, yh, zh = 100.0, 10.0, -1.0

    xyz_b_e = horizon2body(xh, yh, zh, psi, theta, phi)
    xyz_b_q = horizon2body(xh, yh, zh, quat...)

    @test isapprox(xyz_b_e, xyz_b_q)

    xyz_h_e = body2horizon(xyz_b_e..., psi, theta, phi)
    xyz_h_q = body2horizon(xyz_b_e..., quat...)

    @test isapprox(xyz_h_e, xyz_h_q)
end


@testset "wind <-> hor/body" begin
    ones_ = [1.0, 1.0, 1.0]
    angles1 = [0., 45*pi/180., 0.]
    angles2 = [0., 0., 45*pi/180]
    angles3 = [45*pi/180, 0., 0.]
    exp_b2h_1 = [2*0.70710678118654757, 1, 0]
    exp_b2h_2 = [1, 0, 2*0.70710678118654757]
    exp_b2h_3 = [0, 2*0.70710678118654757, 1]
    #wind2horizon
    @test ones_ ≈ wind2horizon(ones_..., 0., 0., 0.)
    @test exp_b2h_1 ≈ wind2horizon(ones_..., angles1...)
    @test exp_b2h_2 ≈ wind2horizon(ones_..., angles2...)
    @test exp_b2h_3 ≈ wind2horizon(ones_..., angles3...)
    #horizon2wind
    @test ones_ ≈ horizon2wind(ones_..., 0., 0., 0.)
    @test ones_ ≈ horizon2wind(exp_b2h_1..., angles1...)
    @test ones_ ≈ horizon2wind(exp_b2h_2..., angles2...)
    @test ones_ ≈ horizon2wind(exp_b2h_3..., angles3...)
    #wind2body
    @test ones_ ≈ wind2body(ones_..., 0., 0.)
    @test [0, 1, 2*0.70710678118654757] ≈ wind2body(ones_..., 45*pi/180., 0.)
    @test exp_b2h_3 ≈ wind2body(ones_..., 0., 45*pi/180.)
    #body2wind
    @test ones_ ≈ body2wind(ones_..., 0., 0.)
    @test ones_ ≈ body2wind(0, 1, 2*0.70710678118654757, 45*pi/180., 0.)
    @test ones_ ≈ body2wind(exp_b2h_3..., 0., 45*pi/180.)
end


@testset "hor <-> ECEF" begin
    xecef, yecef, zecef = 1.0, 10.0, 100.0
    lat, lon = 0.0, 0.0
    exp_xyz_hor = [100.0, 10.0 ,-1.0]
    xyz_hor =  ecef2horizon(xecef, yecef, zecef, lat, lon)
    @test isapprox(xyz_hor, exp_xyz_hor)

    exp_xyz_ecef = [xecef, yecef, zecef]
    xyz_ecef = horizon2ecef(exp_xyz_hor..., lat, lon)
    @test isapprox(xyz_ecef, exp_xyz_ecef)

    lat, lon = pi/2.0, 0.0
    exp_xyz_hor = [-1.0, 10.0 ,-100.0]
    xyz_hor =  ecef2horizon(xecef, yecef, zecef, lat, lon)
    @test isapprox(xyz_hor, exp_xyz_hor)

    exp_xyz_ecef = [xecef, yecef, zecef]
    xyz_ecef = horizon2ecef(exp_xyz_hor..., lat, lon)
    @test isapprox(xyz_ecef, exp_xyz_ecef)

    lat, lon = 0.0, pi/2.0
    exp_xyz_hor = [100.0, -1.0 ,-10.0]

    xyz_hor =  ecef2horizon(xecef, yecef, zecef, lat, lon)
    @test isapprox(xyz_hor, exp_xyz_hor)


    exp_xyz_ecef = [xecef, yecef, zecef]
    xyz_ecef = horizon2ecef(exp_xyz_hor..., lat, lon)
    @test isapprox(xyz_ecef, exp_xyz_ecef)

    lat, lon = pi/2.0, pi/2.0
    exp_xyz_hor = [-10.0, -1.0 ,-100.0]

    xyz_hor =  ecef2horizon(xecef, yecef, zecef, lat, lon)
    @test isapprox(xyz_hor, exp_xyz_hor)

    exp_xyz_ecef = [xecef, yecef, zecef]
    xyz_ecef = horizon2ecef(exp_xyz_hor..., lat, lon)
    @test isapprox(xyz_ecef, exp_xyz_ecef)
end

using FlightMechanicsUtils
using Test


@testset "Translate forces and moments" begin
    # No traslation
    m2 = translate_forces_moments([10, 20, 30], [50, 60, 90], [1, 1, 1], [1, 1, 1])
    @test isapprox(m2, [50, 60, 90])

    # Simple translation 1
    m2 = translate_forces_moments([1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 1])
    @test isapprox(m2, [0, -1, 0])

    # Simple translation 2
    m2 = translate_forces_moments([1, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 1])
    @test isapprox(m2, [1, -1, 0])

    # Simple translation 3
    m2 = translate_forces_moments(
        [10, 20, 30],
        [50, 60, 90],
        [1, 1, 1],
        [1, 1, 1] + [0, 1, 0],
    )
    @test isapprox(m2, [50, 60, 90] + [-30, 0, 10])

    # Reciprocal
    m2 = translate_forces_moments([10, 20, 30], [50, 60, 90], [1, 1, 1], [10, 20, 30])
    m1_ = translate_forces_moments([10, 20, 30], m2, [10, 20, 30], [1, 1, 1])
    @test isapprox([50, 60, 90], m1_)
end


@testset "Steiner" begin
    # No point translation
    p1 = [0, 0, 0]
    p2 = p1
    inertia = [
        1 0 0
        0 2 0
        0 0 3
    ]
    mass = 10
    @test isapprox(steiner_inertia(p1, inertia, mass, p2), inertia)

    # Point with no inertia
    p1 = [0, 0, 0]
    p2 = [10, 0, 0]
    inertia = zeros(3, 3)
    p2 = [10, 0, 0]
    inertia = zeros(3, 3)
    mass = 10

    exp_inertia = [
        0 0 0
        0 1000 0
        0 0 1000
    ]
    @test isapprox(steiner_inertia(p1, inertia, mass, p2), exp_inertia)

    # Point with with inertia
    p1 = [0, 0, 0]
    p2 = [10, 0, 0]
    inertia = [
        1 0 0
        0 2 0
        0 0 3
    ]
    mass = 10

    exp_inertia = [
        0 0 0
        0 1000 0
        0 0 1000
    ] + inertia

    @test isapprox(steiner_inertia(p1, inertia, mass, p2), exp_inertia)

end
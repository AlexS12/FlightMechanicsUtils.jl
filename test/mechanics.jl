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


@testset "Coordinated turn bank angle constrain" begin
    # Check case:  Stevens, B. L., Lewis, F. L., (1992).
    # Aircraft control and simulation: dynamics, controls design, and autonomous systems.
    # John Wiley & Sons. (Section 3.6, figure 3.6-2, page 192)
    ϕ = coordinated_turn_bank(0.3, deg2rad(13.7), deg2rad(0.0292), 502*0.3048, 0.0)
    # Take into account that this check case is a full trim not just the kinematic realtionship
    @test isapprox(rad2deg(ϕ), 78.3, atol=0.05)

    # Opposite turn
    ϕ = coordinated_turn_bank(-0.3, deg2rad(13.7), deg2rad(0.0292), 502*0.3048, 0.0)
    @test isapprox(rad2deg(ϕ), -78.3, atol=0.05)

    # Null turn rate
    ϕ = coordinated_turn_bank(0, deg2rad(13.7), deg2rad(0.0292), 502*0.3048, 0.0)
    @test isapprox(rad2deg(ϕ), 0)

    # For regression testing (no check values found in Stevens)
    ϕ = coordinated_turn_bank(0.1, deg2rad(13.7), deg2rad(0.0292), 502*0.3048, 10)
    @test isapprox(rad2deg(ϕ), 51.16941949883969)
end

using FlightMechanicsUtils
using Test


@testset "Translate forces and moments" begin
    f1 = [10, 20, 30]
    m1 = [50, 60, 90]
    p1 = [1, 1, 1]

    # No traslation
    m2 = translate_forces_moments(f1, m1, p1, p1)
    @test isapprox(m2, m1)

    # Simple translation
    m2 = translate_forces_moments(f1, m1, p1, p1 + [0, 1, 0])
    @test isapprox(m2, m1 + [-30, 0, 10])

    # Reciprocal
    p2 = [10, 20, 30]
    m2 = translate_forces_moments(f1, m1, p1, p2)
    m1_ = translate_forces_moments(f1, m2, p2, p1)
    @test isapprox(m1, m1_)
end

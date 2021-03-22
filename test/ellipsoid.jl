using FlightMechanicsUtils
using Test


@testset "Ellipsoid" begin
    # Rogers, R. M. (2007). Applied mathematics in integrated navigation systems.
    # American Institute of Aeronautics and Astronautics. (Page 77, table 4.2)
    @test isapprox(WGS84.a, 6378137)
    @test isapprox(WGS84.b, 6356752.3142)
    @test isapprox(WGS84.finv, 298.257223563)
    @test isapprox(WGS84.e2, 0.0818191908426^2)
end

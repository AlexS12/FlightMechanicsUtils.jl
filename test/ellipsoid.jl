using FlightMechanicsUtils
using Test


@testset "Ellipsoid WGS84" begin
    # Rogers, R. M. (2007). Applied mathematics in integrated navigation systems.
    # American Institute of Aeronautics and Astronautics. (Page 77, table 4.2)
    @test isapprox(WGS84.a, 6378137)
    @test isapprox(WGS84.b, 6356752.3142)
    @test isapprox(WGS84.finv, 298.257223563)
    @test isapprox(WGS84.e2, 0.0818191908426^2)
end


@testset "llh <-> ECEF" begin
    # llh ECEF (using data from
    # - [1] Bowring, B. R. (1976). Transformation from spatial to geographical
    # coordinates. Survey review, 23(181), 323-327.)

    # From [1] Example 1 (page 325)
    x = 4114496.258  # m
    y = 0.0          # m
    z = 4870157.031  # m
    # Ellipsoid does not match any known standard
    a_bowring1976 = 6378249.145  # m
    b_bowring1976 = 6356514.870  # m
    finv_bowring1976 = a_bowring1976 / (a_bowring1976 - b_bowring1976)
    Bowring1976 = Ellipsoid(a_bowring1976, finv_bowring1976, "Bowring 1976")

    exp_llh = [deg2rad(50.0), 0, 10000.0]
    llh = ecef2llh(x, y, z; ellipsoid=Bowring1976)
    @test isapprox(llh, exp_llh)

    exp_xyz = [4114496.258, 0.0, 4870157.031]  # m
    xyz = llh2ecef(deg2rad(50.0), 0, 10000.0; ellipsoid=Bowring1976)
    @test isapprox(xyz, exp_xyz)


    # http://www.sysense.com/products/ecef_lla_converter/index.html
    x, y, z = (4114496.258, 0.0, 4870157.031)  # m
    exp_llh = [deg2rad(49.996908), 0.000000, 9907.31]
    llh = ecef2llh(x, y, z)
    # Longitude should be exact, latitude max error: 0.0018" according to [1]
    # Altitude 0.17 m
    # The implemented function must be better, but I don't know which one is using
    # this web page. The ellipsoid is assumed to be WGS84
    @test isapprox(llh[:2], exp_llh[:2], atol=deg2rad(0.0018/3600.0))
    @test isapprox(llh[3], exp_llh[3], atol=0.17)

    exp_xyz = [4114291.97, 0.00, 4870449.48]  # m
    xyz = llh2ecef(deg2rad(50.0), 0, 10000.0)
    @test isapprox(xyz, exp_xyz)

    # non-zero longitude
    x, y, z = (3814496.258, 1514496.258, 4870157.031)  # m
    exp_llh = [deg2rad(50.068095), deg2rad(21.654910), 3263.97]
    llh = ecef2llh(x, y, z)
    # Longitude should be exact, latitude max error: 0.0018" according to [1]
    # Altitude 0.17 m
    # The implemented function must be better, but I don't know which one is using
    # this web page. The ellipsoid is assumed to be WGS84
    @test isapprox(llh[:2], exp_llh[:2], atol=deg2rad(0.0018/3600.0))
    @test isapprox(llh[3], exp_llh[3], atol=0.17)

    exp_xyz = [3814496.258, 1514496.258, 4870157.031]  # m
    xyz = llh2ecef(deg2rad(50.068095), deg2rad(21.654910), 3263.97)
    @test isapprox(xyz, exp_xyz)
end

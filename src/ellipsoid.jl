@doc raw"""
    Ellipsoid(a, b, f, finv, e2, ϵ2, name)

Earth ellipsoid model.

# Fields

- `a::Real`: semi-major axis (m).
- `b::Real`: semi-minor axis (m).
- `f::Real`: flattening.
- `finv::Real`: inverse of flattening (``f^{-1}``).
- `e2::Real`: eccentricity squared.
- `ϵ2::Real`: second eccentricity squared.
- `name::String`: ellipsoid name.

# Notes

``a`` is the semi-major axis, ``b`` is the semi-minor axis. Ellipsoids are normally defined
by ``a`` and ``f^{-1}``. The rest of parameters are derived.

Flattening, ``f``:
``f = \frac{a-b}{a} = a - \frac{b}{a}``

Eccentricity (first eccentricity), ``e``:
``e^2 = \frac{a^2 - b^2}{a^2} = f (2 - f)``

Second eccentricity, ``ϵ```:
``ϵ^2 = \frac{a^2 - b^2}{b^2} = \frac{f (2 - f)}{(1 - f)^2}``

# See also

[`Ellipsoid(a, finv, name)`](@ref)

# References

1. Stevens, B. L., Lewis, F. L., (1992). Aircraft control and simulation: dynamics, controls design, and autonomous systems. John Wiley & Sons. Section 1.6 (page 23)
2. Rogers, R. M. (2007). Applied mathematics in integrated navigation systems. American Institute of Aeronautics and Astronautics. Chapter 4 (Nomenclature differs from 1).
3. Bowring, B. R. (1976). Transformation from spatial to geographical coordinates. Survey review, 23(181), 323-327.
"""
struct Ellipsoid
    "Semi-major axis (m)."
    a::Real
    "Semi-minor axis (m)."
    b::Real
    "Flattening ``f = \frac{a-b}{a} = a - \frac{b}{a}``."
    f::Real
    "Inverse of flattening."
    finv::Real
    "Eccentricity. ``e^2 = \frac{a^2 - b^2}{a^2} = f (2 - f)``"
    e2::Real
    "Second eccentricity. ``ϵ^2 = \frac{a^2 - b^2}{b^2} = \frac{f (2 - f)}{(1 - f)^2}``."
    ϵ2::Real
    "Ellipsoid name."
    name::String
end


"""
    Ellipsoid(a, finv, name)

Earth ellipsoid model from semi-major axis, a (m), and inverse of falttening, f.

"""
function Ellipsoid(a, finv, name)
    f = 1 / finv
    b = a * (1 - f)
    e2 = f * (2 - f)
    ϵ2 = e2 / (1 - f)^2
    return Ellipsoid(a, b, f, finv, e2, ϵ2, name)
end


# Rogers, R. M. (2007). Applied mathematics in integrated navigation systems.
# American Institute of Aeronautics and Astronautics. (Page 76, table 4.1)
const Clarke1866 = Ellipsoid(6378206.4, 294.9786982, "Clarke1866")
const Clarke1880 = Ellipsoid(6378249.145, 294.465, "Clarke1880")
const International = Ellipsoid(6378388.0, 297.0, "International")
const Bessel = Ellipsoid(6377397.155, 299.1528128, "Bessel")
const Everest = Ellipsoid(6377276.345, 300.8017, "Everest")
const ModifiedEverest = Ellipsoid(6377304.063, 300.8017, "ModifiedEverest")
const AustralianNational = Ellipsoid(6378160.0, 298.25, "AustralianNational")
const SouthAmerican1969 = Ellipsoid(6378160.0, 298.25, "SouthAmerican1969")
const Airy = Ellipsoid(6377564.396, 299.3249646, "Airy")
const ModifiedAiry = Ellipsoid(6377340.189, 299.3249646, "ModifiedAiry")
const Hough = Ellipsoid(6378270.0, 297.0, "Hough")
const Fischer1960SouthAsia = Ellipsoid(6378155.0, 298.3, "Fischer1960SouthAsia")
const Fischer1960Mercury = Ellipsoid(6378166.0, 298.3, "Fischer1960Mercury")
const Fischer1968 = Ellipsoid(6378150.0, 298.3, "Fischer1968")
const WGS60 = Ellipsoid(6378165.0, 298.3, "WGS60")
const WGS66 = Ellipsoid(6378145.0, 298.25, "WGS66")
const WGS72 = Ellipsoid(6378135.0, 298.26, "WGS72")
const WGS84 = Ellipsoid(6378137.0, 298.257223563, "WGS84")


"""
    llh2ecef(lat, lon, height; ellipsoid=WGS84)

Transform geodetic latitude, longitude (rad) and ellipsoidal height (m) to ECEF for the
given ellipsoid (default ellipsoid is WGS84).

# References

1. Rogers, R. M. (2007). Applied mathematics in integrated navigation systems. American Institute of Aeronautics and Astronautics. (Page 75, equations 4.20, 4.21, 4.22)
"""
function llh2ecef(lat, lon, height; ellipsoid=WGS84)
    f = ellipsoid.f
    a = ellipsoid.a
    e2 = ellipsoid.e2

    var = a / sqrt(1.0 - e2 * sin(lat)*sin(lat))

    px = (var + height) * cos(lat)*cos(lon)
    py = (var + height) * cos(lat)*sin(lon)
    pz = (var * (1.0 - e2) + height)* sin(lat)

    return [px, py, pz]
end


"""
    ecef2llh(xecef, yecef, zecef; ellipsoid=WGS84)

Transform ECEF coordinates to geodetic latitude, longitude (rad) and ellipsoidal height (m)
for the given ellipsoid (default ellipsoid is WGS84).

# Notes

* The transformation is direct without iterations as [1] introduced the need to iterate for
near Earth positions.
* [2] is an updated of increased accuracy of [1]. The former is used in this implementation
although the latter implementation is commented in the code.
* Model becomes unstable if latitude is close to 90º. An alternative equation
 can be found in [2] equation (16) but has not been implemented.

 # References

1. Bowring, B. R. (1976). Transformation from spatial to geographical coordinates. Survey review, 23(181), 323-327.
2. Bowring, B. R. (1985). The accuracy of geodetic latitude and height equations. Survey Review, 28(218), 202-206.
"""
function ecef2llh(xecef, yecef, zecef; ellipsoid=WGS84)

    x, y, z = xecef, yecef, zecef
    e = sqrt(ellipsoid.e2);
    e2 = ellipsoid.e2
    ϵ2 = ellipsoid.ϵ2
    a = ellipsoid.a
    b = ellipsoid.b

    p = sqrt(x*x + y*y)
    R = sqrt(p*p + z*z)
    θ = atan(z, p)
    # [1] equation (1) does not change in [2]
    lon = atan(y, x)
    # u -> geographical latitude
    # [1] below equation (4) and [2] equation (6)
    # This lead to errors in latitud with maximum value 0.0018"
    #u = atan(a/b * z/x)

    # [2] equation (17) If the latitude is also required to be very accurate
    # for outer-space positions then the value of u for (6) should be obtained
    # from:
    u = atan(b*z / (a*p) * (1 + ϵ2 * b / R))
    # [1] equation (4)
    #lat = atan((z + ϵ2 * b * sin(u)^3) / (x - e2 * a * cos(u)^3))
    # [2] equation (18)
    lat = atan((z + ϵ2 * b * sin(u)^3) / (p - e2 * a * cos(u)^3))
    # [2] after equation (1)
    v = a / sqrt(1.0 - e2*sin(lat)^2)
    # [2] equation (7)
    # height = p*cos(lat) + z*sin(lat) - a*a / v
    # equivalent to [2] equation (8)
    height = R * cos(lat - θ) - a*a / v
    # which is insensitive to the error in latitude calculation (The influence
    # is of order 2 [2] equation (12))
    return [lat, lon, height]
end

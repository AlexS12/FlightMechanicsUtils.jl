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

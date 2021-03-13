@doc raw"""
    translate_forces_moments(forces_1, moments_1, p1, p2)

Calculate equivalent moment at point 2 (p2) given the forces (forces_1)
 and moments (moments_1) at point 1 (p1).

``M_{2} = M_{1} + r \times F_{1}``
"""
function translate_forces_moments(forces_1, moments_1, p1, p2)
    r21 = p1 - p2
    moments_2 = moments_1 + r21 × forces_1
    return moments_2
end

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


@doc raw"""
    steiner_inertia(cg, inertia_g, mass, p2)

Calculate the inertia tensor of a rigid solid at point 2 (p2),
given the inertia tensor (inertia_g) at the center of gravity (cg)
and the mass of the system.

`` r = p_2 - cg ``

``I_{2} = I_{cg} + m (r^T · r I - r · r^T) ``
"""
function steiner_inertia(cg, inertia1, mass, p2)
    r = p2 - cg
    inertia2 = inertia1 + mass * ((r' * r) * I - r*r')
    return inertia2
end

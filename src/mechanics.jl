@doc raw"""
    translate_forces_moments(forces_1, moments_1, p1, p2)

Calculate equivalent moment at point 2 (p2) given the forces (forces_1)
 and moments (moments_1) at point 1 (p1).

`` r = p_1 - p_2 ``

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


"""
    coordinated_turn_bank(ψ_dot, α, β, tas, γ)

Calculate roll angle (ϕ) for a given turn rate (ψ_dot), angle of attack (α),
angle of sideslie (β), tas and flight path angle (γ) in the absence of wind.

Imposes sum of forces along y body axis equal to zero.
"""
function coordinated_turn_bank(ψ_dot, α, β, tas, γ)
    G = ψ_dot * tas / gD

    if abs(γ) < 1e-8
        ϕ = G * cos(β) / (cos(α) - G * sin(α) * sin(β))
        ϕ = atan(ϕ)
    else
        a = 1 - G * tan(α) * sin(β)
        b = sin(γ) / cos(β)
        c = 1 + G^2 * cos(β)^2

        sq = sqrt(c * (1 - b^2) + G^2 * sin(β)^2)

        num = (a - b^2) + b * tan(α) * sq
        den = a ^ 2 - b^2 * (1 + c * tan(α)^2)

        ϕ = atan(G * cos(β) / cos(α) * num / den)
    end
    return ϕ
end

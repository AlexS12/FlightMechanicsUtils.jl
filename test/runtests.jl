using SafeTestsets

@safetestset "Rotations" begin include("rotations.jl") end
@safetestset "Atmosphere" begin include("atmosphere.jl") end

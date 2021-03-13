using SafeTestsets

@safetestset "Rotations" begin include("rotations.jl") end
@safetestset "Atmosphere" begin include("atmosphere.jl") end
@safetestset "Kinematics" begin include("kinematics.jl") end
@safetestset "Mechanics" begin include("mechanics.jl") end
@safetestset "Anemometry" begin include("anemometry.jl") end

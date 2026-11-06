using BigArm
using Test

@testset "BigArm.jl" begin
    include("primitives.jl")
    include("graph.jl")
    include("geometry.jl")
    include("ghd.jl")
end

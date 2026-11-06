using CoordinateTransformations: Translation
using GeometryBasics: Mesh, coordinates, faces
using StaticArrays: SVector

include(joinpath(@__DIR__, "..", "projects", "ghd", "upright.jl"))

@testset "SM-2.5 upright end-to-end" begin
    spec = UprightSpec()
    upright = build_upright(spec)

    @test upright isa RectangularTube
    @test upright.width === 3.0
    @test upright.height === 3.0
    @test upright.length === 103.0
    @test upright.thickness === 0.120
    @test !isstock(upright)
    @test length(holes(upright)) == spec.hole_count

    g = ManufacturingGraph(upright)
    @test length(g.nodes) == 2 + spec.hole_count
    @test isstock(first(g.nodes))
    @test first(g.nodes).length === spec.stock_length
    @test g.nodes[2].length === spec.length
    @test last(g.nodes) === upright

    ops = operations(upright)
    @test length(ops) == 1 + spec.hole_count
    @test first(ops) isa Saw
    @test all(op -> op isa Drill && op.penetration isa ThroughAll, ops[2:end])

    # Invalid oversized hole on the upright fails immediately
    front = datum(upright, SVector(0, -1, 0))
    @test_throws ArgumentError drill(
        upright,
        Circle(4.0),
        Datum(front.transform ∘ Translation(1.5, 51.0, 0.0)),
        ThroughAll(),
    )

    dot = to_dot(g)
    @test startswith(dot, "digraph ManufacturingGraph {\n")
    @test occursin("Saw(103.0)", dot)
    @test occursin("Drill(Circle(1.0), (1.5, 0.0, 2.0), ThroughAll)", dot)
    @test occursin("Drill(Circle(1.0), (1.5, 0.0, 100.0), ThroughAll)", dot)

    mesh = derive_mesh(upright)
    @test mesh isa Mesh
    lo, hi = mesh_bbox(mesh)
    @test lo ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
    @test hi ≈ SVector(3.0, 3.0, 103.0) atol = 1e-12

    stats = check_manifold_and_volume(mesh)
    @test stats.euler == -4 * spec.hole_count
end

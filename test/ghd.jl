using CoordinateTransformations: Translation
using GeometryBasics: Mesh, coordinates, faces
using LinearAlgebra: cross, dot, norm
using MeshCat: MeshCat, Visualizer
using StaticArrays: SVector

include(joinpath(@__DIR__, "..", "projects", "ghd", "upright.jl"))
include(joinpath(@__DIR__, "..", "projects", "ghd", "attachment.jl"))

function world_hole_datum(transform::RigidTransform, op::Drill)
    return Datum(transform ∘ op.datum.transform)
end

function are_coaxial(d1::Datum, d2::Datum; atol::Real=1e-9)
    u1 = zaxis(d1)
    u2 = zaxis(d2)
    parallel = abs(abs(dot(u1, u2)) - 1.0) <= atol
    collinear = norm(cross(origin(d2) - origin(d1), u1)) <= atol
    return parallel && collinear
end

@testset "SM-2.5 upright end-to-end" begin
    spec = UprightSpec()
    upright = build_upright(spec)

    @test upright isa RectangularTube
    @test upright.width === 3.0
    @test upright.height === 3.0
    @test upright.length === 103.0
    @test upright.thickness === 0.120
    @test !isstock(upright)
    @test length(operations_of_type(Drill, upright)) == spec.hole_count

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
    @test occursin("Saw((0.0, 0.0, 103.0), (0.0, 0.0, 1.0))", dot)
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

@testset "GHD attachment and upright assembly" begin
    plate_spec = MountingPlateSpec()
    arm_spec = ArmSpec()
    plate = build_mounting_plate(plate_spec)
    arm = build_arm(arm_spec)

    @test plate isa Plate
    @test plate.width === 12.0
    @test plate.height === 8.0
    @test plate.thickness === 0.1875
    @test !isstock(plate)
    @test length(operations_of_type(Drill, plate)) == 4
    @test length(ManufacturingGraph(plate).nodes) == 5

    plate_mesh = derive_mesh(plate)
    lo_p, hi_p = mesh_bbox(plate_mesh)
    @test lo_p ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
    @test hi_p ≈ SVector(12.0, 8.0, 0.1875) atol = 1e-12
    plate_stats = check_manifold_and_volume(plate_mesh)
    @test plate_stats.euler == 2 - 2 * length(operations_of_type(Drill, plate))

    @test arm isa RectangularTube
    @test arm.width === 3.0
    @test arm.height === 3.0
    @test arm.length === 24.0
    @test arm.thickness === 0.120
    @test !isstock(arm)
    @test length(operations_of_type(Drill, arm)) == 2
    @test length(ManufacturingGraph(arm).nodes) == 4

    arm_mesh = derive_mesh(arm)
    lo_a, hi_a = mesh_bbox(arm_mesh)
    @test lo_a ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
    @test hi_a ≈ SVector(3.0, 3.0, 24.0) atol = 1e-12
    arm_stats = check_manifold_and_volume(arm_mesh)
    @test arm_stats.euler == -4 * length(operations_of_type(Drill, arm))

    mount_hole_index = 19
    asm = build_ghd_assembly(;
        plate_spec=plate_spec,
        arm_spec=arm_spec,
        mount_hole_index=mount_hole_index,
    )
    @test asm isa Assembly
    @test length(asm) == 4

    upright_part, t_upright = asm[1]
    left_plate_part, t_left = asm[2]
    right_plate_part, t_right = asm[3]
    arm_part, t_arm = asm[4]

    upright_holes = operations_of_type(Drill, upright_part)
    left_holes = operations_of_type(Drill, left_plate_part)
    right_holes = operations_of_type(Drill, right_plate_part)
    arm_holes = operations_of_type(Drill, arm_part)

    # Left and right plates align coaxially at all 4 holes
    for k in 1:4
        dl = world_hole_datum(t_left, left_holes[k])
        dr = world_hole_datum(t_right, right_holes[k])
        @test are_coaxial(dl, dr)
        @test left_holes[k].profile.diameter === right_holes[k].profile.diameter
    end

    # Plate upright-mounting holes (holes 1 and 2) align coaxially with upright holes #19 and #21
    for (plate_idx, upright_idx) in ((1, mount_hole_index), (2, mount_hole_index + 2))
        du = world_hole_datum(t_upright, upright_holes[upright_idx])
        dl = world_hole_datum(t_left, left_holes[plate_idx])
        dr = world_hole_datum(t_right, right_holes[plate_idx])
        @test are_coaxial(dl, du)
        @test are_coaxial(dr, du)
        @test left_holes[plate_idx].profile.diameter === upright_holes[upright_idx].profile.diameter
    end

    # Plate arm-mounting holes (holes 3 and 4) align coaxially with horizontal arm holes 1 and 2
    for (plate_idx, arm_idx) in ((3, 1), (4, 2))
        da = world_hole_datum(t_arm, arm_holes[arm_idx])
        dl = world_hole_datum(t_left, left_holes[plate_idx])
        dr = world_hole_datum(t_right, right_holes[plate_idx])
        @test are_coaxial(dl, da)
        @test are_coaxial(dr, da)
        @test left_holes[plate_idx].profile.diameter === arm_holes[arm_idx].profile.diameter
    end

    vis = Visualizer()
    @test visualize(asm; vis=vis, open_browser=false) === vis
    MeshCat.close_server!(vis.core)
end

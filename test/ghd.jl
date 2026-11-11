using CoordinateTransformations: Translation
using GeometryBasics: Mesh, coordinates, faces
using LinearAlgebra: cross, dot, norm
using MeshCat: MeshCat, Visualizer
using StaticArrays: SVector

include(joinpath(@__DIR__, "..", "projects", "ghd", "upright.jl"))
include(joinpath(@__DIR__, "..", "projects", "ghd", "bolted_arm.jl"))
include(joinpath(@__DIR__, "..", "projects", "ghd", "echo_ghd.jl"))

function world_hole_datum(transform::RigidTransform, op::Drill)
    return Datum(transform ∘ op.datum.transform)
end

function world_tube_axis_datum(transform::RigidTransform)
    return Datum(transform)
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

@testset "Design B welded Echo GHD attachment and assembly" begin
    spec = EchoGHDAttachmentSpec()
    footplate = build_footplate(spec)
    ear = build_side_ear(spec)
    web = build_standoff_web(spec)
    axle = build_roller_axle(spec)
    roller = build_foam_roller(spec)
    frame = build_echo_ghd_weldment(spec)

    # Footplate: 24 × 14 × 3/16 in plate with 9.0 × 6.5 in rectangular cutout window
    @test footplate isa Plate
    @test footplate.width === 24.0
    @test footplate.height === 14.0
    @test footplate.thickness === 0.1875
    @test !isstock(footplate)
    @test length(operations_of_type(Cut, footplate)) == 1
    fp_mesh = derive_mesh(footplate)
    lo_fp, hi_fp = mesh_bbox(fp_mesh)
    @test lo_fp ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
    @test hi_fp ≈ SVector(24.0, 14.0, 0.1875) atol = 1e-12
    fp_stats = check_manifold_and_volume(fp_mesh)
    @test fp_stats.euler == 0
    @test fp_stats.volume ≈ (24.0 * 14.0 - 9.0 * 6.5) * 0.1875 atol = 1e-9

    # Side ear: 2.5 × 8.5 × 3/16 in plate with 4 × 1.0 in through-holes on 2.0 in centers
    @test ear isa Plate
    @test ear.width === 2.5
    @test ear.height === 8.5
    @test ear.thickness === 0.1875
    @test length(operations_of_type(Drill, ear)) == 4
    ear_mesh = derive_mesh(ear)
    ear_stats = check_manifold_and_volume(ear_mesh)
    @test ear_stats.euler == 2 - 2 * 4

    # Central roller standoff web: 6.0 × 10.5 × 3/16 in plate with 2 × 1.0 in axle holes
    @test web isa Plate
    @test web.width === 6.0
    @test web.height === 10.5
    @test web.thickness === 0.1875
    @test length(operations_of_type(Drill, web)) == 2
    web_mesh = derive_mesh(web)
    web_stats = check_manifold_and_volume(web_mesh)
    @test web_stats.euler == 2 - 2 * 2

    # Roller axle shaft: 1.0 in OD × 20.0 in long round steel tube (0.120 in wall)
    @test axle isa RoundTube
    @test axle.outer_diameter === 1.0
    @test axle.length === 20.0
    @test axle.thickness === 0.120
    axle_stats = check_manifold_and_volume(derive_mesh(axle))
    @test axle_stats.euler == 0

    # Foam roller: 4.5 in OD × 8.0 in long cylindrical pad (1.75 in wall -> 1.0 in ID)
    @test roller isa RoundTube
    @test roller.outer_diameter === 4.5
    @test roller.length === 8.0
    @test roller.thickness === 1.75
    roller_stats = check_manifold_and_volume(derive_mesh(roller))
    @test roller_stats.euler == 0

    # Welded steel frame DAG and mesh
    @test frame isa Weldment
    @test operation(frame) isa Weld
    @test operation(frame).profile === LineSegment(2.0)
    frame_g = ManufacturingGraph(frame)
    @test frame_g.nodes[end] === frame
    @test count(isstock, frame_g.nodes) == 4
    @test length(operations_of_type(Cut, frame)) == 1
    @test length(operations_of_type(Drill, frame)) == 6
    @test length(operations_of_type(Saw, frame)) == 1
    weld_ops = operations_of_type(Weld, frame)
    @test length(weld_ops) == 5
    @test [op.profile for op in weld_ops] == [
        LineSegment(1.0),
        LineSegment(1.0),
        Circle(1.0),
        Circle(1.0),
        LineSegment(2.0),
    ]
    frame_dot = to_dot(frame_g)
    @test occursin("part Weldment", frame_dot)
    @test occursin("Weld(LineSegment(1.0), (10.3125, 2.75, 0.0))", frame_dot)
    @test occursin("Weld(LineSegment(1.0), (13.5, 2.75, 0.0))", frame_dot)
    @test occursin("Weld(Circle(1.0), (5.0, 1.0, 0.1875))", frame_dot)
    @test occursin("Weld(Circle(1.0), (5.0, 9.5, 0.1875))", frame_dot)
    @test occursin("Weld(LineSegment(2.0), (12.09375, 1.75, 0.0))", frame_dot)
    @test count(line -> occursin("[label=\"Weld(", line), split(chomp(frame_dot), '\n')) == 10

    for node in frame_g.nodes
        check_manifold_and_volume(derive_mesh(node))
    end
    frame_mesh = derive_mesh(frame)
    lo_f, hi_f = mesh_bbox(frame_mesh)
    @test lo_f ≈ SVector(0.0, 0.0, -2.5) atol = 1e-12
    @test hi_f ≈ SVector(24.0, 14.0, 6.0) atol = 1e-12
    frame_stats = check_manifold_and_volume(frame_mesh)
    @test frame_stats.volume ≈
          fp_stats.volume + 2 * ear_stats.volume + web_stats.volume + 2 * axle_stats.volume atol = 1e-9

    # Full assembly on SM-2.5 upright
    mount_hole_index = 19
    asm = build_echo_ghd_assembly(;
        attachment_spec=spec,
        mount_hole_index=mount_hole_index,
    )
    @test asm isa Assembly
    @test length(asm) == 6

    upright_part, t_upright = asm[1]
    frame_part, t_frame = asm[2]
    ll_roller, t_ll = asm[3]
    lr_roller, t_lr = asm[4]
    ul_roller, t_ul = asm[5]
    ur_roller, t_ur = asm[6]
    @test frame_part isa Weldment
    @test all(r -> r isa RoundTube, (ll_roller, lr_roller, ul_roller, ur_roller))

    # Extract relative transforms from the weld tree (`frame = ((footplate + left_ear) + right_ear) + ((web + lower_axle) + upper_axle)`)
    w_ears, web_assembly = parent(frame_part)
    w_left, right_ear_part = parent(w_ears)
    _, left_ear_part = parent(w_left)
    w_web_lower, _ = parent(web_assembly)
    web_part, _ = parent(w_web_lower)

    t_left_ear = BigArm._weld_transform(operation(w_left))
    t_right_ear = BigArm._weld_transform(operation(w_ears))
    t_web = BigArm._weld_transform(operation(frame_part))
    t_lower_axle = t_web ∘ BigArm._weld_transform(operation(w_web_lower))
    t_upper_axle = t_web ∘ BigArm._weld_transform(operation(web_assembly))

    upright_holes = operations_of_type(Drill, upright_part)
    left_ear_holes = operations_of_type(Drill, left_ear_part)
    right_ear_holes = operations_of_type(Drill, right_ear_part)
    web_holes = operations_of_type(Drill, web_part)

    # All 4 holes of left and right side ears align coaxially in world coordinates
    for k in 1:4
        dl = world_hole_datum(t_frame ∘ t_left_ear, left_ear_holes[k])
        dr = world_hole_datum(t_frame ∘ t_right_ear, right_ear_holes[k])
        @test are_coaxial(dl, dr)
    end

    # All 4 ear holes align coaxially with 4 consecutive upright holes (18, 19, 20, 21)
    for (ear_idx, upright_idx) in (
        (1, mount_hole_index - 1),
        (2, mount_hole_index),
        (3, mount_hole_index + 1),
        (4, mount_hole_index + 2),
    )
        du = world_hole_datum(t_upright, upright_holes[upright_idx])
        dl = world_hole_datum(t_frame ∘ t_left_ear, left_ear_holes[ear_idx])
        dr = world_hole_datum(t_frame ∘ t_right_ear, right_ear_holes[ear_idx])
        @test are_coaxial(dl, du)
        @test are_coaxial(dr, du)
    end

    # Lower and upper axles align coaxially with standoff web holes and all 4 foam rollers
    d_web_lower = world_hole_datum(t_frame ∘ t_web, web_holes[1])
    d_web_upper = world_hole_datum(t_frame ∘ t_web, web_holes[2])
    d_axle_lower = world_tube_axis_datum(t_frame ∘ t_lower_axle)
    d_axle_upper = world_tube_axis_datum(t_frame ∘ t_upper_axle)
    @test are_coaxial(d_axle_lower, d_web_lower)
    @test are_coaxial(d_axle_upper, d_web_upper)
    @test are_coaxial(world_tube_axis_datum(t_ll), d_axle_lower)
    @test are_coaxial(world_tube_axis_datum(t_lr), d_axle_lower)
    @test are_coaxial(world_tube_axis_datum(t_ul), d_axle_upper)
    @test are_coaxial(world_tube_axis_datum(t_ur), d_axle_upper)

    vis = Visualizer()
    @test visualize(asm; vis=vis, open_browser=false) === vis
    MeshCat.close_server!(vis.core)
end

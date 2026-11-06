using CoordinateTransformations: LinearMap, Translation
using GeometryBasics: GeometryBasics, Mesh, Point2d, coordinates, faces
using LinearAlgebra: cross, dot, norm
using MeshCat: MeshCat, Visualizer
using Rotations: RotX
using StaticArrays: SVector

function mesh_bbox(mesh::Mesh)
    pts = coordinates(mesh)
    xs = [p[1] for p in pts]
    ys = [p[2] for p in pts]
    zs = [p[3] for p in pts]
    return (
        SVector(minimum(xs), minimum(ys), minimum(zs)),
        SVector(maximum(xs), maximum(ys), maximum(zs)),
    )
end

function check_manifold_and_volume(mesh::Mesh)
    pts = coordinates(mesh)
    fcs = faces(mesh)
    nverts = length(pts)
    nfaces = length(fcs)
    half_edges = Dict{Tuple{Int,Int},Int}()
    vol6 = 0.0
    valid_indices = true
    nondegenerate = true

    for f in fcs
        i1, i2, i3 = Int(f[1]), Int(f[2]), Int(f[3])
        if !(1 <= i1 <= nverts && 1 <= i2 <= nverts && 1 <= i3 <= nverts && i1 != i2 && i2 != i3 && i3 != i1)
            valid_indices = false
            break
        end

        p1 = SVector(pts[i1][1], pts[i1][2], pts[i1][3])
        p2 = SVector(pts[i2][1], pts[i2][2], pts[i2][3])
        p3 = SVector(pts[i3][1], pts[i3][2], pts[i3][3])
        norm(cross(p2 - p1, p3 - p1)) > 1e-12 || (nondegenerate = false)

        for (u, v) in ((i1, i2), (i2, i3), (i3, i1))
            half_edges[(u, v)] = get(half_edges, (u, v), 0) + 1
        end
        vol6 += dot(p1, cross(p2, p3))
    end

    @test valid_indices
    @test nondegenerate

    # Watertight orientable 2-manifold: every directed half-edge appears once,
    # and its reverse half-edge appears once.
    @test all(==(1), values(half_edges))
    @test all(((u, v),) -> get(half_edges, (v, u), 0) == 1, keys(half_edges))

    nedges = length(half_edges) ÷ 2
    euler = nverts - nedges + nfaces
    return (; nverts, nfaces, nedges, euler, volume=vol6 / 6.0)
end

@testset "Derived 2D/3D geometry" begin
    stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
    blank = saw(stick, 103.0)
    front = datum(blank, SVector(0, -1, 0))
    right = datum(blank, SVector(1, 0, 0))

    one_wall = drill(
        blank,
        Circle(1.0),
        Datum(front.transform ∘ Translation(1.5, 10.0, 0.0)),
        ThroughWall(),
    )
    one_hole = drill(
        blank,
        Circle(1.0),
        Datum(front.transform ∘ Translation(1.0, 10.0, 0.0)),
        ThroughAll(),
    )
    multi_hole = drill(
        drill(
            one_hole,
            Circle(1.0),
            Datum(front.transform ∘ Translation(1.5, 20.0, 0.0)),
            ThroughAll(),
        ),
        Circle(0.5),
        Datum(right.transform ∘ Translation(1.5, 30.0, 0.0)),
        ThroughAll(),
    )

    @testset "Raw stick and sawn blank meshes" begin
        stick_mesh = derive_mesh(stick)
        @test stick_mesh isa Mesh
        lo, hi = mesh_bbox(stick_mesh)
        @test lo ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test hi ≈ SVector(3.0, 3.0, 288.0) atol = 1e-12

        stick_stats = check_manifold_and_volume(stick_mesh)
        @test stick_stats.nverts == 16
        @test stick_stats.nfaces == 32
        @test stick_stats.euler == 0
        expected_area = 3.0 * 3.0 - (3.0 - 2 * 0.120)^2
        @test stick_stats.volume ≈ expected_area * 288.0 atol = 1e-9

        blank_mesh = derive_mesh(blank)
        @test blank_mesh isa Mesh
        lo_b, hi_b = mesh_bbox(blank_mesh)
        @test lo_b ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test hi_b ≈ SVector(3.0, 3.0, 103.0) atol = 1e-12

        blank_stats = check_manifold_and_volume(blank_mesh)
        @test blank_stats.nverts == 16
        @test blank_stats.nfaces == 32
        @test blank_stats.euler == 0
        @test blank_stats.volume ≈ expected_area * 103.0 atol = 1e-9
    end

    @testset "Intermediate 1-hole and multi-hole part meshes" begin
        blank_stats = check_manifold_and_volume(derive_mesh(blank))

        wall_mesh = derive_mesh(one_wall)
        lo_w, hi_w = mesh_bbox(wall_mesh)
        @test lo_w ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test hi_w ≈ SVector(3.0, 3.0, 103.0) atol = 1e-12
        wall_stats = check_manifold_and_volume(wall_mesh)
        @test wall_stats.euler == -2
        @test wall_stats.volume < blank_stats.volume

        # ThroughWall pierces front wall (Y = 0 and Y = t), not back wall (Y = 3.0)
        wall_pts = coordinates(wall_mesh)
        @test count(p -> isapprox(p[2], 0.0; atol=1e-9) && isapprox(hypot(p[1] - 1.5, p[3] - 10.0), 0.5; atol=1e-9), wall_pts) == BigArm._CIRCLE_SEGMENTS
        @test count(p -> isapprox(p[2], 0.120; atol=1e-9) && isapprox(hypot(p[1] - 1.5, p[3] - 10.0), 0.5; atol=1e-9), wall_pts) == BigArm._CIRCLE_SEGMENTS
        @test count(p -> isapprox(p[2], 3.0; atol=1e-9) && isapprox(hypot(p[1] - 1.5, p[3] - 10.0), 0.5; atol=1e-9), wall_pts) == 0

        one_mesh = derive_mesh(one_hole)
        lo_1, hi_1 = mesh_bbox(one_mesh)
        @test lo_1 ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test hi_1 ≈ SVector(3.0, 3.0, 103.0) atol = 1e-12
        one_stats = check_manifold_and_volume(one_mesh)
        @test one_stats.euler == -4
        @test (blank_stats.volume - one_stats.volume) ≈ 2.0 * (blank_stats.volume - wall_stats.volume) atol = 1e-9
        @test isapprox(blank_stats.volume - one_stats.volume, 2.0 * π * 0.5^2 * 0.120; rtol=0.01)

        # Off-center ThroughAll hole at X = 1.0, Z = 10.0 aligns on front and back walls in 3D
        one_pts = coordinates(one_mesh)
        for y_plane in (0.0, 0.120, 3.0 - 0.120, 3.0)
            @test count(
                p -> isapprox(p[2], y_plane; atol=1e-9) && isapprox(hypot(p[1] - 1.0, p[3] - 10.0), 0.5; atol=1e-9),
                one_pts,
            ) == BigArm._CIRCLE_SEGMENTS
        end

        multi_mesh = derive_mesh(multi_hole)
        lo_m, hi_m = mesh_bbox(multi_mesh)
        @test lo_m ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test hi_m ≈ SVector(3.0, 3.0, 103.0) atol = 1e-12
        multi_stats = check_manifold_and_volume(multi_mesh)
        @test multi_stats.euler == -12
        @test multi_stats.volume < one_stats.volume

        # Every node in the manufacturing graph meshes cleanly
        g = ManufacturingGraph(multi_hole)
        for node in g.nodes
            m = derive_mesh(node)
            @test m isa Mesh
            check_manifold_and_volume(m)
        end
    end

    @testset "2.5D slab generality and dispatch" begin
        b = BigArm._MeshBuilder()
        plate_datum = Datum(Translation(0.0, 0.0, 0.0) ∘ LinearMap(RotX(0.0)))
        outer = BigArm._rect_loop(0.0, 0.0, 6.0, 4.0)
        hole_loop = Point2d[Point2d(p[1] + 3.0, p[2] + 2.0) for p in BigArm._contour(Circle(1.0))]
        BigArm._add_slab!(
            b,
            plate_datum,
            outer,
            outer,
            [hole_loop],
            0.25;
            close_perimeter=true,
        )
        plate_mesh = Mesh(b.points, b.faces)
        lo, hi = mesh_bbox(plate_mesh)
        @test lo ≈ SVector(0.0, 0.0, -0.25) atol = 1e-12
        @test hi ≈ SVector(6.0, 4.0, 0.0) atol = 1e-12
        plate_stats = check_manifold_and_volume(plate_mesh)
        @test plate_stats.euler == 0
        @test isapprox(plate_stats.volume, (6.0 * 4.0 - π * 0.5^2) * 0.25; rtol=0.01)

        struct UnmeshedPart <: AbstractPart end
        @test_throws MethodError derive_mesh(UnmeshedPart())
    end

    @testset "Plate meshes" begin
        raw_plate = Plate(6.0, 4.0, 0.25)
        raw_mesh = derive_mesh(raw_plate)
        @test raw_mesh isa Mesh
        lo_p, hi_p = mesh_bbox(raw_mesh)
        @test lo_p ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test hi_p ≈ SVector(6.0, 4.0, 0.25) atol = 1e-12
        raw_stats = check_manifold_and_volume(raw_mesh)
        @test raw_stats.nverts == 8
        @test raw_stats.nfaces == 12
        @test raw_stats.euler == 2
        @test raw_stats.volume ≈ 6.0 * 4.0 * 0.25 atol = 1e-9

        top = datum(raw_plate, SVector(0, 0, 1))
        bottom = datum(raw_plate, SVector(0, 0, -1))

        # Single through-hole from top (+Z)
        p1 = drill(raw_plate, Circle(1.0), Datum(top.transform ∘ Translation(2.0, 2.0, 0.0)), ThroughAll())
        m1 = derive_mesh(p1)
        lo_1, hi_1 = mesh_bbox(m1)
        @test lo_1 ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test hi_1 ≈ SVector(6.0, 4.0, 0.25) atol = 1e-12
        s1 = check_manifold_and_volume(m1)
        @test s1.euler == 0
        @test isapprox(raw_stats.volume - s1.volume, π * 0.5^2 * 0.25; rtol=0.01)

        pts1 = coordinates(m1)
        for z_plane in (0.0, 0.25)
            @test count(
                p -> isapprox(p[3], z_plane; atol=1e-9) && isapprox(hypot(p[1] - 2.0, p[2] - 2.0), 0.5; atol=1e-9),
                pts1,
            ) == BigArm._CIRCLE_SEGMENTS
        end

        # Second through-hole from bottom (-Z) with ThroughWall (at bottom x=1.5 -> 3D X=4.5)
        p2 = drill(p1, Circle(1.0), Datum(bottom.transform ∘ Translation(1.5, 2.0, 0.0)), ThroughWall())
        m2 = derive_mesh(p2)
        s2 = check_manifold_and_volume(m2)
        @test s2.euler == -2
        @test (raw_stats.volume - s2.volume) ≈ 2.0 * (raw_stats.volume - s1.volume) atol = 1e-9

        pts2 = coordinates(m2)
        for z_plane in (0.0, 0.25)
            @test count(
                p -> isapprox(p[3], z_plane; atol=1e-9) && isapprox(hypot(p[1] - 4.5, p[2] - 2.0), 0.5; atol=1e-9),
                pts2,
            ) == BigArm._CIRCLE_SEGMENTS
        end

        # Blind holes from top and bottom surfaces
        p3 = drill(p2, Circle(0.5), Datum(top.transform ∘ Translation(3.25, 1.0, 0.0)), Blind(0.1))
        p4 = drill(p3, Circle(0.5), Datum(bottom.transform ∘ Translation(2.75, 3.0, 0.0)), Blind(0.1))
        for node in ManufacturingGraph(p4).nodes
            mn = derive_mesh(node)
            @test mn isa Mesh
            check_manifold_and_volume(mn)
        end
        s4 = check_manifold_and_volume(derive_mesh(p4))
        @test s4.euler == -2
        @test isapprox(s2.volume - s4.volume, 2.0 * π * 0.25^2 * 0.1; rtol=0.01)
    end

    @testset "Geometry and graph visualization" begin
        vis = Visualizer()
        plate = drill(
            Plate(6.0, 4.0, 0.25),
            Circle(1.0),
            Datum(Translation(2.0, 2.0, 0.25) ∘ LinearMap(RotX(0.0))),
            ThroughAll(),
        )
        for p in (stick, blank, one_hole, multi_hole, plate)
            returned = visualize(p; vis=vis, open_browser=false)
            @test returned === vis
        end
        html = MeshCat.static_html(vis)
        @test occursin("BufferGeometry", html)
        MeshCat.close_server!(vis.core)

        g = ManufacturingGraph(multi_hole)
        dot_path = tempname() * ".dot"
        dot_out = visualize(g; path=dot_path, open_viewer=false)
        @test dot_out == to_dot(g)
        @test read(dot_path, String) == to_dot(g)
        rm(dot_path; force=true)
    end
end

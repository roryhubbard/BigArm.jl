using CoordinateTransformations: LinearMap, Translation
using GeometryBasics: GeometryBasics, Mesh, Point2d, coordinates, faces
using LinearAlgebra: cross, dot, norm
using MeshCat: MeshCat, Visualizer
using Rotations: RotX, RotY, RotZ
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
    blank = saw(stick, Datum(Translation(0.0, 0.0, 103.0)))
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

        # Miter-sawn RectangularTube (45° top miter centered at Z = 50.0, 45° bottom miter centered at Z = 10.0)
        miter_top = saw(blank, Datum(Translation(1.5, 1.5, 50.0) ∘ LinearMap(RotY(-π / 4))))
        miter_top_mesh = derive_mesh(miter_top)
        lo_mt, hi_mt = mesh_bbox(miter_top_mesh)
        @test lo_mt ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test hi_mt ≈ SVector(3.0, 3.0, 51.5) atol = 1e-12
        mt_stats = check_manifold_and_volume(miter_top_mesh)
        @test mt_stats.nverts == 16
        @test mt_stats.nfaces == 32
        @test mt_stats.euler == 0
        @test mt_stats.volume ≈ expected_area * 50.0 atol = 1e-9

        miter_both = saw(miter_top, Datum(Translation(1.5, 1.5, 10.0) ∘ LinearMap(RotX(3π / 4))))
        miter_both_mesh = derive_mesh(miter_both)
        mb_stats = check_manifold_and_volume(miter_both_mesh)
        @test mb_stats.nverts == 16
        @test mb_stats.nfaces == 32
        @test mb_stats.euler == 0
        @test mb_stats.volume ≈ expected_area * (50.0 - 10.0) atol = 1e-9
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

        # Polygonal Cut on RectangularTube (ThroughWall and ThroughAll)
        tube_slot = cut(
            blank,
            Rectangle(1.0, 2.0),
            Datum(front.transform ∘ Translation(1.5, 10.0, 0.0)),
            ThroughAll(),
        )
        m_slot = derive_mesh(tube_slot)
        s_slot = check_manifold_and_volume(m_slot)
        @test s_slot.euler == -4
        @test (blank_stats.volume - s_slot.volume) ≈ 2.0 * (1.0 * 2.0) * 0.120 atol = 1e-9
    end

    @testset "2.5D slab generality and dispatch" begin
        b = BigArm._MeshBuilder()
        plate_datum = Datum(IdentityRigidTransform)
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

        # Rectangular through-window cutout in a Plate (e.g. Echo GHD footplate)
        fp_stock = Plate(24.0, 14.0, 0.1875)
        fp_top = datum(fp_stock, SVector(0, 0, 1))
        fp_bottom = datum(fp_stock, SVector(0, 0, -1))
        fp_window = cut(
            fp_stock,
            Rectangle(9.0, 8.5),
            Datum(fp_top.transform ∘ Translation(12.0, 7.0, 0.0)),
        )
        m_fp = derive_mesh(fp_window)
        lo_fp, hi_fp = mesh_bbox(m_fp)
        @test lo_fp ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test hi_fp ≈ SVector(24.0, 14.0, 0.1875) atol = 1e-12
        s_fp = check_manifold_and_volume(m_fp)
        @test s_fp.nverts == 16
        @test s_fp.nfaces == 32
        @test s_fp.euler == 0
        @test s_fp.volume ≈ (24.0 * 14.0 - 9.0 * 8.5) * 0.1875 atol = 1e-9

        pts_fp = coordinates(m_fp)
        for z_plane in (0.0, 0.1875), x_corner in (7.5, 16.5), y_corner in (2.75, 11.25)
            @test count(
                p -> isapprox(p[1], x_corner; atol=1e-9) &&
                     isapprox(p[2], y_corner; atol=1e-9) &&
                     isapprox(p[3], z_plane; atol=1e-9),
                pts_fp,
            ) == 1
        end

        # Combining rectangular cutouts (top and bottom) with circular holes
        fp_combo = drill(
            cut(
                fp_window,
                Rectangle(2.0, 1.5),
                Datum(fp_bottom.transform ∘ Translation(3.0, 7.0, 0.0)),
            ),
            Circle(1.0),
            Datum(fp_top.transform ∘ Translation(3.0, 7.0, 0.0)),
            ThroughAll(),
        )
        s_combo = check_manifold_and_volume(derive_mesh(fp_combo))
        @test s_combo.euler == -4
        @test isapprox(
            s_fp.volume - s_combo.volume,
            (2.0 * 1.5 + π * 0.5^2) * 0.1875;
            rtol=0.01,
        )

        # Bandsawn Plate: 45° corner chamfer removing a 2×2 right triangle (area = 2.0)
        chamfered_plate = saw(
            raw_plate,
            Datum(Translation(4.0, 4.0, 0.0) ∘ LinearMap(RotZ(π / 4) * RotY(π / 2))),
        )
        m_chamfer = derive_mesh(chamfered_plate)
        s_chamfer = check_manifold_and_volume(m_chamfer)
        @test s_chamfer.nverts == 10
        @test s_chamfer.euler == 2
        @test s_chamfer.volume ≈ (6.0 * 4.0 - 0.5 * 2.0 * 2.0) * 0.25 atol = 1e-9
    end

    @testset "RoundTube meshes" begin
        axle_stock = RoundTube(1.0, 240.0, 0.120)
        axle = saw(axle_stock, Datum(Translation(0.0, 0.0, 20.0)))
        roller = saw(RoundTube(4.5, 96.0, 1.75), Datum(Translation(0.0, 0.0, 8.0)))

        for (tube, od, len, t) in (
            (axle_stock, 1.0, 240.0, 0.120),
            (axle, 1.0, 20.0, 0.120),
            (roller, 4.5, 8.0, 1.75),
        )
            m = derive_mesh(tube)
            @test m isa Mesh
            r_out = od / 2.0
            r_in = r_out - t
            lo, hi = mesh_bbox(m)
            @test lo ≈ SVector(-r_out, -r_out, 0.0) atol = 1e-12
            @test hi ≈ SVector(r_out, r_out, len) atol = 1e-12
            stats = check_manifold_and_volume(m)
            @test stats.nverts == 4 * BigArm._CIRCLE_SEGMENTS
            @test stats.euler == 0
            @test isapprox(stats.volume, π * (r_out^2 - r_in^2) * len; rtol=0.01)
        end

        # 45° miter-sawn RoundTube centered at Z = 15.0
        miter_axle = saw(axle, Datum(Translation(0.0, 0.0, 15.0) ∘ LinearMap(RotX(π / 4))))
        m_miter_axle = derive_mesh(miter_axle)
        lo_ma, hi_ma = mesh_bbox(m_miter_axle)
        @test lo_ma ≈ SVector(-0.5, -0.5, 0.0) atol = 1e-12
        @test hi_ma ≈ SVector(0.5, 0.5, 15.5) atol = 1e-12
        stats_ma = check_manifold_and_volume(m_miter_axle)
        @test stats_ma.nverts == 4 * BigArm._CIRCLE_SEGMENTS
        @test stats_ma.euler == 0
        @test isapprox(stats_ma.volume, π * (0.5^2 - (0.5 - 0.120)^2) * 15.0; rtol=0.01)
    end

    @testset "Geometry and graph visualization" begin
        vis = Visualizer()
        plate = cut(
            drill(
                Plate(6.0, 4.0, 0.25),
                Circle(1.0),
                Datum(Translation(1.5, 2.0, 0.25)),
                ThroughAll(),
            ),
            Rectangle(1.5, 1.5),
            Datum(Translation(4.0, 2.0, 0.25)),
        )
        axle = saw(RoundTube(1.0, 240.0, 0.120), Datum(Translation(0.0, 0.0, 20.0)))
        for p in (stick, blank, one_hole, multi_hole, plate, axle)
            returned = visualize(p; vis=vis, open_browser=false)
            @test returned === vis
        end
        asm = Assembly([
            one_hole => IdentityRigidTransform,
            plate => Translation(-1.5, 0.0, 10.0) ∘ LinearMap(RotX(π / 2)),
        ])
        returned_asm = visualize(asm; vis=vis, open_browser=false)
        @test returned_asm === vis
        root_node = vis.core.tree.children["meshcat"]
        @test haskey(root_node.children, "part_1")
        @test haskey(root_node.children, "part_2")
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

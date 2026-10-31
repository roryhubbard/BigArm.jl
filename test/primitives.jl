using CoordinateTransformations: AffineMap, LinearMap, Translation
using Rotations: RotX, RotZ
using StaticArrays: SVector

@testset "Primitives and manufacturability rules" begin
    @testset "Type hierarchy, Datum SE(3) transforms, and valid construction" begin
        stick = RectangularTube(3, 3, 288, 0.120)
        @test stick isa RectangularTube
        @test stick isa AbstractTube
        @test stick isa AbstractPart
        @test stick.width === 3.0
        @test stick.height === 3.0
        @test stick.length === 288.0
        @test stick.wall === 0.120
        @test stick.parent === nothing
        @test stick.op === nothing
        @test isstock(stick)
        @test length(datums(stick)) == 4
        @test isempty(holes(stick))

        c = Circle(1)
        @test c isa Circle
        @test c isa AbstractProfile
        @test c.diameter === 1.0

        # Front wall datum frame and CoordinateTransformations AffineMap
        f = front(stick)
        @test f isa Datum
        @test f isa AbstractDatum
        @test origin(f) ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test x_axis(f) ≈ SVector(1.0, 0.0, 0.0) atol = 1e-12
        @test y_axis(f) ≈ SVector(0.0, 0.0, 1.0) atol = 1e-12
        @test z_axis(f) ≈ SVector(0.0, -1.0, 0.0) atol = 1e-12

        # Direct construction from Translation and LinearMap
        @test origin(Datum(Translation(1.0, 2.0, 3.0))) === SVector(1.0, 2.0, 3.0)
        @test z_axis(Datum(LinearMap(RotX(π / 2)))) ≈ SVector(0.0, -1.0, 0.0) atol = 1e-12

        d = translate(f, 1.5, 2.0)
        @test d === front(stick, 1.5, 2.0)
        @test origin(d) ≈ SVector(1.5, 0.0, 2.0) atol = 1e-12
        @test local_coords(f, d) ≈ SVector(1.5, 2.0, 0.0) atol = 1e-12
        @test d(0.0, 0.0, 0.0) ≈ SVector(1.5, 0.0, 2.0) atol = 1e-12
        @test f(1.5, 2.0, 0.0) ≈ SVector(1.5, 0.0, 2.0) atol = 1e-12

        # Right, back, left wall transforms map local (x, y) to 3D automatically
        @test origin(right(stick, 0.5, 10.0)) ≈ SVector(3.0, 0.5, 10.0) atol = 1e-12
        @test origin(back(stick, 0.8, 10.0)) ≈ SVector(2.2, 3.0, 10.0) atol = 1e-12
        @test origin(left(stick, 0.5, 10.0)) ≈ SVector(0.0, 2.5, 10.0) atol = 1e-12

        @test ThroughAll() === ThroughAll()
        @test ThroughAll() isa AbstractPenetration
        @test ThroughWall() === ThroughWall()
        @test ThroughWall() isa AbstractPenetration
        @test Blind(0.5) === Blind(0.5)
        @test Blind(0.5).depth === 0.5

        s_op = Saw(103)
        @test s_op isa Saw
        @test s_op isa AbstractOperation
        @test s_op.length === 103.0

        dr_op = Drill(c, d, ThroughAll())
        @test dr_op isa Drill
        @test dr_op isa AbstractOperation
        @test dr_op.profile === c
        @test dr_op.datum === d
        @test dr_op.penetration === ThroughAll()
        pw = pierced_walls(stick, dr_op)
        @test length(pw) == 2
        @test pw[1][1] == front(stick)
        @test pw[1][2] ≈ SVector(1.5, 0.0, 2.0) atol = 1e-12
        @test pw[2][1] == back(stick)
        @test pw[2][2] ≈ SVector(1.5, 3.0, 2.0) atol = 1e-12
        pw_wall = pierced_walls(stick, Drill(c, d, ThroughWall()))
        @test length(pw_wall) == 1
        @test pw_wall[1][1] == front(stick)
        @test pw_wall[1][2] ≈ SVector(1.5, 0.0, 2.0) atol = 1e-12
    end

    @testset "Invalid stock dimensions" begin
        # Non-positive dimensions
        @test_throws DomainError RectangularTube(0.0, 3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(-3.0, 3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 0.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, -3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, 0.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, -100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, 100.0, 0.0)
        @test_throws DomainError RectangularTube(3.0, 3.0, 100.0, -0.120)

        # Non-finite dimensions
        @test_throws DomainError RectangularTube(NaN, 3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(Inf, 3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, NaN, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, Inf, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, 100.0, Inf)

        # Wall thickness >= min(width, height) / 2
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 100.0, 1.5)
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 100.0, 1.6)
        @test_throws ArgumentError RectangularTube(3.0, 2.0, 100.0, 1.0)
        @test_throws ArgumentError RectangularTube(2.0, 3.0, 100.0, 1.1)

        # Mismatched parent and op
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 103.0, 0.120, stick, nothing)
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 103.0, 0.120, nothing, Saw(103.0))
    end

    @testset "Invalid Circle, Datum, Blind, and Saw construction" begin
        @test_throws DomainError Circle(0.0)
        @test_throws DomainError Circle(-1.0)
        @test_throws DomainError Circle(NaN)
        @test_throws DomainError Circle(Inf)

        # Non-finite Datum coordinates or linear map
        @test_throws DomainError Datum(Translation(NaN, 0.0, 0.0))
        @test_throws DomainError Datum(LinearMap([Inf 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0]))
        # Non-rotation (scaling or reflection) linear map rejected by Rotations.isrotation
        @test_throws ArgumentError Datum(LinearMap([2.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0]))
        @test_throws ArgumentError Datum(LinearMap([1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 -1.0]))

        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        @test_throws DomainError translate(front(stick), NaN, 5.0)
        @test_throws DomainError translate(front(stick), Inf, 5.0)
        @test_throws DomainError translate(front(stick), 1.5, NaN)
        @test_throws DomainError translate(front(stick), 1.5, Inf)

        @test_throws DomainError Blind(0.0)
        @test_throws DomainError Blind(-0.5)
        @test_throws DomainError Blind(NaN)
        @test_throws DomainError Blind(Inf)

        @test_throws DomainError Saw(0.0)
        @test_throws DomainError Saw(-10.0)
        @test_throws DomainError Saw(NaN)
        @test_throws DomainError Saw(Inf)
    end

    @testset "Valid operation chains and immutability" begin
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        blank = saw(stick, 103.0)

        @test blank isa RectangularTube
        @test !isstock(blank)
        @test blank.parent === stick
        @test blank.op === Saw(103.0)
        @test blank.length === 103.0
        @test isempty(holes(blank))

        # Sawing to exact same length is permitted (length <= current length)
        same_len = saw(blank, 103.0)
        @test same_len.length === 103.0
        @test same_len.parent === blank

        # Sequential ThroughAll drilling (default penetration), centered at x = 1.5 on a 3″ face
        hole = Circle(1.0)
        upright = blank
        for y in 2.0:2.0:100.0
            upright = drill(upright, hole, translate(front(upright), 1.5, y))
        end

        @test upright isa RectangularTube
        @test !isstock(upright)
        @test upright.op === Drill(hole, front(upright, 1.5, 100.0), ThroughAll())
        @test upright.length === 103.0

        drilled_holes = holes(upright)
        @test length(drilled_holes) == 50
        @test first(drilled_holes) === Drill(hole, front(upright, 1.5, 2.0), ThroughAll())
        @test last(drilled_holes) === Drill(hole, front(upright, 1.5, 100.0), ThroughAll())

        # Immutability of upstream nodes
        @test isstock(stick)
        @test stick.length === 288.0
        @test isempty(holes(blank))

        # ThroughWall on left/right faces
        p1 = drill(blank, hole, left(blank, 1.5, 10.0), ThroughWall())
        pw1 = pierced_walls(p1, p1.op)
        @test length(pw1) == 1
        @test pw1[1][1] == left(p1)
        @test pw1[1][2] ≈ SVector(0.0, 1.5, 10.0) atol = 1e-12
        @test length(holes(p1)) == 1

        # Sawing a drilled part above all existing holes
        p2 = saw(p1, 50.0)
        @test p2.length === 50.0
        @test p2.parent === p1
        @test p2.op === Saw(50.0)
        @test length(holes(p2)) == 1
    end

    @testset "Invalid holes: non-existent datum walls, oversized, off-center, out-of-bounds, and overlapping" begin
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        blank = saw(stick, 103.0)
        inner_width = 3.0 - 2.0 * 0.120 # 2.76; inner cavity spans x ∈ (0.120, 2.880)

        # RectangularTube has no top/bottom (+Z / -Z) wall datums
        @test_throws ArgumentError top(blank)
        @test_throws ArgumentError bottom(blank)
        top_datum = Datum(Translation(1.5, 1.5, 103.0))
        @test_throws ArgumentError drill(blank, Circle(1.0), top_datum)
        # Datum floating off the front wall surface (z_offset != 0) fails
        floating_datum = translate(front(blank), 1.5, 20.0, 0.5)
        @test_throws ArgumentError drill(blank, Circle(1.0), floating_datum)

        # Oversized holes at centerline x = 1.5
        @test_throws ArgumentError drill(blank, Circle(4.0), front(blank, 1.5, 51.0))
        @test_throws ArgumentError drill(blank, Circle(3.0), front(blank, 1.5, 51.0))
        @test_throws ArgumentError drill(blank, Circle(inner_width), front(blank, 1.5, 51.0)) # touches wall at 0.12 and 2.88
        @test_throws ArgumentError drill(blank, Circle(2.8), front(blank, 1.5, 51.0))

        # Off-center holes encroaching on left wall (x - r <= 0.12) or right wall (x + r >= 2.88)
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 0.62, 51.0))   # 0.62 - 0.5 = 0.12 fails
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 0.5, 51.0))    # 0.5 - 0.5 = 0.0 fails
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 0.0, 51.0))    # at left edge fails
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 2.38, 51.0))   # 2.38 + 0.5 = 2.88 fails
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 2.5, 51.0))    # 2.5 + 0.5 = 3.0 fails
        @test_throws ArgumentError drill(blank, Circle(1.0), left(blank, 2.38, 51.0))    # same bound on side faces

        # Valid off-center hole just inside the inner cavity edge
        valid_offcenter = drill(blank, Circle(1.0), front(blank, 2.37, 51.0)) # 2.37 + 0.5 = 2.87 < 2.88
        @test length(holes(valid_offcenter)) == 1

        # Out-of-bounds holes (d/2 < y < length - d/2)
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 1.5, 0.5))
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 1.5, 0.25))
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 1.5, -1.0))
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 1.5, 102.5))
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 1.5, 102.8))
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 1.5, 103.0))
        @test_throws ArgumentError drill(blank, Circle(1.0), front(blank, 1.5, 150.0))

        # Overlapping holes on the same face
        p1 = drill(blank, Circle(1.0), front(blank, 1.5, 20.0))
        @test_throws ArgumentError drill(p1, Circle(1.0), front(p1, 1.5, 20.0))     # duplicate
        @test_throws ArgumentError drill(p1, Circle(1.0), front(p1, 1.5, 20.5))     # overlap in y
        @test_throws ArgumentError drill(p1, Circle(1.0), front(p1, 1.5, 19.5))     # overlap in y below
        @test_throws ArgumentError drill(p1, Circle(1.0), front(p1, 1.5, 21.0))     # tangent in y
        @test_throws ArgumentError drill(p1, Circle(1.0), front(p1, 2.0, 20.0))     # overlap in x at same y
        @test_throws ArgumentError drill(p1, Circle(1.0), front(p1, 1.8, 20.3))     # overlap diagonally
        @test_throws ArgumentError drill(p1, Circle(2.0), front(p1, 1.5, 21.5))     # tangent with different diameters

        # ThroughWall pierces only datum face; ThroughAll pierces opposite wall too
        tw_blank = saw(RectangularTube(3.0, 3.0, 288.0, 0.120), 103.0)
        tw_front = drill(tw_blank, Circle(1.0), front(tw_blank, 1.5, 20.0), ThroughWall())
        # Drilling the opposite wall at the same height is allowed since the first cut
        # pierced only the front wall into the cavity.
        tw_back = drill(tw_front, Circle(1.0), back(tw_front, 1.5, 20.0), ThroughWall())
        @test length(holes(tw_back)) == 2
        @test_throws ArgumentError drill(tw_back, Circle(1.0), back(tw_back, 1.5, 20.0), ThroughWall())
        @test_throws ArgumentError drill(tw_front, Circle(1.0), back(tw_front, 1.5, 20.0), ThroughAll())
        @test_throws ArgumentError drill(tw_front, Circle(1.0), front(tw_front, 1.5, 20.5), ThroughWall())

        # ThroughAll on front occupies both walls
        @test_throws ArgumentError drill(p1, Circle(1.0), back(p1, 1.5, 20.0), ThroughAll())
        @test_throws ArgumentError drill(p1, Circle(1.0), back(p1, 1.5, 20.5), ThroughWall())

        # Off-center ThroughAll hole on front at x = 0.8 exits back at 3D (0.8, 3.0, 30.0),
        # which is back(off_front, 2.2, 30.0) in the back wall's local frame!
        off_front = drill(blank, Circle(0.5), front(blank, 0.8, 30.0), ThroughAll())
        @test_throws ArgumentError drill(off_front, Circle(0.5), back(off_front, 2.2, 30.0), ThroughWall())
        # Whereas local x = 0.8 on back is at 3D (2.2, 3.0, 30.0), 1.4″ away and does not overlap!
        off_back_ok = drill(off_front, Circle(0.5), back(off_front, 0.8, 30.0), ThroughWall())
        @test length(holes(off_back_ok)) == 2

        # Same-face offset that avoids overlap in (x, y) even at same y
        offset_ok = drill(p1, Circle(0.5), front(p1, 2.6, 20.0)) # |2.6 - 1.5| = 1.1 > (1.0+0.5)/2 = 0.75
        @test length(holes(offset_ok)) == 2
    end

    @testset "Invalid saw cuts" begin
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        blank = saw(stick, 103.0)

        # Target length longer than stock/part length
        @test_throws ArgumentError saw(stick, 300.0)
        @test_throws ArgumentError saw(blank, 103.1)
        @test_throws ArgumentError saw(blank, 288.0)

        # Invalid target length values
        @test_throws DomainError saw(blank, 0.0)
        @test_throws DomainError saw(blank, -10.0)
        @test_throws DomainError saw(blank, NaN)
        @test_throws DomainError saw(blank, Inf)

        # Cuts across or below existing holes (hole at Z = 50.0, d = 1.0 spans [49.5, 50.5])
        drilled = drill(blank, Circle(1.0), front(blank, 1.5, 50.0))
        @test_throws ArgumentError saw(drilled, 50.0)
        @test_throws ArgumentError saw(drilled, 50.2)
        @test_throws ArgumentError saw(drilled, 49.8)
        @test_throws ArgumentError saw(drilled, 50.5)
        @test_throws ArgumentError saw(drilled, 49.5)
        @test_throws ArgumentError saw(drilled, 30.0)
    end

    @testset "Dispatch enforcement" begin
        struct UnsupportedPart <: AbstractPart end
        struct UnsupportedTube <: AbstractTube end
        struct SquareProfile <: AbstractProfile end

        blank = saw(RectangularTube(3.0, 3.0, 288.0, 0.120), 103.0)
        d = front(blank, 1.5, 2.0)

        # Unsupported parts
        @test_throws MethodError saw(UnsupportedPart(), 10.0)
        @test_throws MethodError drill(UnsupportedPart(), Circle(1.0), d)
        @test_throws MethodError drill(UnsupportedTube(), Circle(1.0), d)

        # Non-circular profiles and Blind penetration on a thin-wall tube fail by dispatch
        @test_throws MethodError drill(blank, SquareProfile(), d)
        @test_throws MethodError drill(blank, Circle(1.0), d, Blind(0.5))
    end
end

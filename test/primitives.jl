@testset "Primitives and manufacturability rules" begin
    @testset "Type hierarchy and valid construction" begin
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
        @test isempty(holes(stick))

        c = Circle(1)
        @test c isa Circle
        @test c isa AbstractProfile
        @test c.diameter === 1.0

        d = FaceDatum(:front, 2)
        @test d isa FaceDatum
        @test d isa AbstractDatum
        @test d.face === :front
        @test d.x === 0.0
        @test d.z === 2.0
        @test FaceDatum(:left, 0.5, 10).x === 0.5
        @test FaceDatum(:left, 0.5, 10).z === 10.0

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
        @test pierced_faces(dr_op) == (:front, :back)
        @test pierced_faces(Drill(c, d, ThroughWall())) == (:front, nothing)
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

    @testset "Invalid Circle, FaceDatum, Blind, and Saw construction" begin
        @test_throws DomainError Circle(0.0)
        @test_throws DomainError Circle(-1.0)
        @test_throws DomainError Circle(NaN)
        @test_throws DomainError Circle(Inf)

        @test_throws ArgumentError FaceDatum(:top, 0.0, 5.0)
        @test_throws ArgumentError FaceDatum(:bottom, 0.0, 5.0)
        @test_throws ArgumentError FaceDatum(:invalid, 0.0, 5.0)
        @test_throws DomainError FaceDatum(:front, NaN, 5.0)
        @test_throws DomainError FaceDatum(:front, Inf, 5.0)
        @test_throws DomainError FaceDatum(:front, 0.0, NaN)
        @test_throws DomainError FaceDatum(:front, 0.0, Inf)

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

        # Sequential ThroughAll drilling (default penetration)
        hole = Circle(1.0)
        upright = blank
        for z in 2.0:2.0:100.0
            upright = drill(upright, hole, FaceDatum(:front, 0.0, z))
        end

        @test upright isa RectangularTube
        @test !isstock(upright)
        @test upright.op === Drill(hole, FaceDatum(:front, 0.0, 100.0), ThroughAll())
        @test upright.length === 103.0

        drilled_holes = holes(upright)
        @test length(drilled_holes) == 50
        @test first(drilled_holes) === Drill(hole, FaceDatum(:front, 0.0, 2.0), ThroughAll())
        @test last(drilled_holes) === Drill(hole, FaceDatum(:front, 0.0, 100.0), ThroughAll())

        # Immutability of upstream nodes
        @test isstock(stick)
        @test stick.length === 288.0
        @test isempty(holes(blank))

        # ThroughWall on left/right faces
        p1 = drill(blank, hole, FaceDatum(:left, 0.0, 10.0), ThroughWall())
        @test pierced_faces(p1.op) == (:left, nothing)
        @test length(holes(p1)) == 1

        # Sawing a drilled part above all existing holes
        p2 = saw(p1, 50.0)
        @test p2.length === 50.0
        @test p2.parent === p1
        @test p2.op === Saw(50.0)
        @test length(holes(p2)) == 1
    end

    @testset "Invalid holes: oversized, off-center, out-of-bounds, and overlapping" begin
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        blank = saw(stick, 103.0)
        inner_width = 3.0 - 2.0 * 0.120 # 2.76 (half = 1.38)

        # Oversized holes (abs(x) + d/2 < face_width / 2 - wall = 1.38)
        @test_throws ArgumentError drill(blank, Circle(4.0), FaceDatum(:front, 0.0, 51.0))
        @test_throws ArgumentError drill(blank, Circle(3.0), FaceDatum(:front, 0.0, 51.0))
        @test_throws ArgumentError drill(blank, Circle(inner_width), FaceDatum(:front, 0.0, 51.0)) # 1.38 < 1.38 fails
        @test_throws ArgumentError drill(blank, Circle(2.8), FaceDatum(:front, 0.0, 51.0))

        # Off-center holes (x too large)
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, 0.88, 51.0))  # 0.88 + 0.5 = 1.38 fails
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, 1.0, 51.0))   # 1.0 + 0.5 = 1.5 fails
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, -1.0, 51.0))  # abs(-1.0) + 0.5 = 1.5 fails
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:left, 0.88, 51.0))   # same bound on side faces

        # Valid off-center hole just inside the inner cavity edge
        valid_offcenter = drill(blank, Circle(1.0), FaceDatum(:front, 0.87, 51.0)) # 0.87 + 0.5 = 1.37 < 1.38
        @test length(holes(valid_offcenter)) == 1

        # Out-of-bounds holes (d/2 < z < length - d/2)
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, 0.0, 0.5))
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, 0.0, 0.25))
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, 0.0, -1.0))
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, 0.0, 102.5))
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, 0.0, 102.8))
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, 0.0, 103.0))
        @test_throws ArgumentError drill(blank, Circle(1.0), FaceDatum(:front, 0.0, 150.0))

        # Overlapping holes on the same face
        p1 = drill(blank, Circle(1.0), FaceDatum(:front, 0.0, 20.0))
        @test_throws ArgumentError drill(p1, Circle(1.0), FaceDatum(:front, 0.0, 20.0))     # duplicate
        @test_throws ArgumentError drill(p1, Circle(1.0), FaceDatum(:front, 0.0, 20.5))     # overlap in z
        @test_throws ArgumentError drill(p1, Circle(1.0), FaceDatum(:front, 0.0, 19.5))     # overlap in z below
        @test_throws ArgumentError drill(p1, Circle(1.0), FaceDatum(:front, 0.0, 21.0))     # tangent in z
        @test_throws ArgumentError drill(p1, Circle(1.0), FaceDatum(:front, 0.5, 20.0))     # overlap in x at same z
        @test_throws ArgumentError drill(p1, Circle(1.0), FaceDatum(:front, 0.3, 20.3))     # overlap diagonally
        @test_throws ArgumentError drill(p1, Circle(2.0), FaceDatum(:front, 0.0, 21.5))     # tangent with different diameters

        # ThroughWall pierces only datum face; ThroughAll pierces opposite wall too
        tw_blank = saw(RectangularTube(3.0, 3.0, 288.0, 0.120), 103.0)
        tw_front = drill(tw_blank, Circle(1.0), FaceDatum(:front, 0.0, 20.0), ThroughWall())
        # Drilling the opposite wall at the same z is allowed since the first cut
        # pierced only :front into the cavity.
        tw_back = drill(tw_front, Circle(1.0), FaceDatum(:back, 0.0, 20.0), ThroughWall())
        @test length(holes(tw_back)) == 2
        @test_throws ArgumentError drill(tw_back, Circle(1.0), FaceDatum(:back, 0.0, 20.0), ThroughWall())
        @test_throws ArgumentError drill(tw_front, Circle(1.0), FaceDatum(:back, 0.0, 20.0), ThroughAll())
        @test_throws ArgumentError drill(tw_front, Circle(1.0), FaceDatum(:front, 0.0, 20.5), ThroughWall())

        # ThroughAll on :front occupies both walls
        @test_throws ArgumentError drill(p1, Circle(1.0), FaceDatum(:back, 0.0, 20.0), ThroughAll())
        @test_throws ArgumentError drill(p1, Circle(1.0), FaceDatum(:back, 0.0, 20.5), ThroughWall())

        # Same-face offset that avoids overlap in (x, z) even at same z
        offset_ok = drill(p1, Circle(0.5), FaceDatum(:front, 1.1, 20.0)) # sqrt(1.1^2) > (1.0+0.5)/2 = 0.75
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

        # Cuts across or below existing holes (hole at z = 50.0, d = 1.0 spans [49.5, 50.5])
        drilled = drill(blank, Circle(1.0), FaceDatum(:front, 0.0, 50.0))
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

        # Unsupported parts
        @test_throws MethodError saw(UnsupportedPart(), 10.0)
        @test_throws MethodError drill(UnsupportedPart(), Circle(1.0), FaceDatum(:front, 0.0, 2.0))
        @test_throws MethodError drill(UnsupportedTube(), Circle(1.0), FaceDatum(:front, 0.0, 2.0))

        # Non-circular profiles and Blind penetration on a thin-wall tube fail by dispatch
        @test_throws MethodError drill(blank, SquareProfile(), FaceDatum(:front, 0.0, 2.0))
        @test_throws MethodError drill(blank, Circle(1.0), FaceDatum(:front, 0.0, 2.0), Blind(0.5))
    end
end

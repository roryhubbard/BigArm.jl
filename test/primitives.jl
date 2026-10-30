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

        h = Hole(1, 2)
        @test h isa Hole
        @test h.diameter === 1.0
        @test h.z === 2.0

        s_op = Saw(103)
        @test s_op isa Saw
        @test s_op isa AbstractOperation
        @test s_op.length === 103.0

        d_op = Drill(h)
        @test d_op isa Drill
        @test d_op isa AbstractOperation
        @test d_op.hole === h
        @test Drill(1.0, 2.0) === d_op
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

    @testset "Invalid Hole and Saw construction" begin
        @test_throws DomainError Hole(0.0, 5.0)
        @test_throws DomainError Hole(-1.0, 5.0)
        @test_throws DomainError Hole(NaN, 5.0)
        @test_throws DomainError Hole(Inf, 5.0)
        @test_throws DomainError Hole(1.0, 0.0)
        @test_throws DomainError Hole(1.0, -2.0)
        @test_throws DomainError Hole(1.0, NaN)
        @test_throws DomainError Hole(1.0, Inf)

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
        @test blank.width === 3.0
        @test blank.height === 3.0
        @test blank.length === 103.0
        @test blank.wall === 0.120
        @test isempty(holes(blank))

        # Sawing to exact same length is permitted (length <= current length)
        same_len = saw(blank, 103.0)
        @test same_len.length === 103.0
        @test same_len.parent === blank

        # Sequential drilling
        upright = blank
        for z in 2.0:2.0:100.0
            upright = drill(upright, Hole(1.0, z))
        end

        @test upright isa RectangularTube
        @test !isstock(upright)
        @test upright.op === Drill(Hole(1.0, 100.0))
        @test upright.length === 103.0

        drilled_holes = holes(upright)
        @test length(drilled_holes) == 50
        @test first(drilled_holes) == Hole(1.0, 2.0)
        @test last(drilled_holes) == Hole(1.0, 100.0)

        # Immutability of upstream nodes
        @test isstock(stick)
        @test stick.length === 288.0
        @test isempty(holes(blank))

        # Sawing a drilled part above all existing holes
        p1 = drill(blank, Hole(1.0, 10.0))
        p2 = saw(p1, 50.0)
        @test p2.length === 50.0
        @test p2.parent === p1
        @test p2.op === Saw(50.0)
        @test holes(p2) == [Hole(1.0, 10.0)]
    end

    @testset "Invalid holes: oversized, out-of-bounds, and overlapping" begin
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        blank = saw(stick, 103.0)
        inner_width = 3.0 - 2.0 * 0.120 # 2.76

        # Oversized holes (must satisfy diameter < width - 2 * wall)
        @test_throws ArgumentError drill(blank, Hole(4.0, 51.0))
        @test_throws ArgumentError drill(blank, Hole(3.0, 51.0))
        @test_throws ArgumentError drill(blank, Hole(2.8, 51.0))
        @test_throws ArgumentError drill(blank, Hole(inner_width, 51.0))

        # Out-of-bounds holes (must satisfy diameter / 2 < z < length - diameter / 2)
        @test_throws ArgumentError drill(blank, Hole(1.0, 0.5))   # tangent to bottom end (z == r)
        @test_throws ArgumentError drill(blank, Hole(1.0, 0.25))  # breaks through bottom end (z < r)
        @test_throws ArgumentError drill(blank, Hole(1.0, 102.5)) # tangent to top end (z == L - r)
        @test_throws ArgumentError drill(blank, Hole(1.0, 102.8)) # breaks through top end (z > L - r)
        @test_throws ArgumentError drill(blank, Hole(1.0, 103.0)) # centered on top end
        @test_throws ArgumentError drill(blank, Hole(1.0, 150.0)) # beyond tube length

        # Overlapping holes
        p1 = drill(blank, Hole(1.0, 20.0))
        @test_throws ArgumentError drill(p1, Hole(1.0, 20.0))     # concentric duplicate
        @test_throws ArgumentError drill(p1, Hole(1.0, 20.5))     # partial overlap
        @test_throws ArgumentError drill(p1, Hole(1.0, 19.5))     # partial overlap below
        @test_throws ArgumentError drill(p1, Hole(1.0, 21.0))     # tangent (zero web between holes)
        @test_throws ArgumentError drill(p1, Hole(1.0, 19.0))     # tangent below
        @test_throws ArgumentError drill(p1, Hole(2.0, 21.5))     # tangent with different diameters ((1+2)/2 == 1.5)
        @test_throws ArgumentError drill(p1, Hole(2.0, 21.2))     # overlap with different diameters

        # Non-overlapping hole just past tangent succeeds
        p2 = drill(p1, Hole(1.0, 21.01))
        @test length(holes(p2)) == 2
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
        drilled = drill(blank, Hole(1.0, 50.0))
        @test_throws ArgumentError saw(drilled, 50.0)  # through hole center
        @test_throws ArgumentError saw(drilled, 50.2)  # through upper half of hole
        @test_throws ArgumentError saw(drilled, 49.8)  # through lower half of hole
        @test_throws ArgumentError saw(drilled, 50.5)  # tangent to top edge of hole
        @test_throws ArgumentError saw(drilled, 49.5)  # tangent to bottom edge of hole
        @test_throws ArgumentError saw(drilled, 30.0)  # below existing hole
    end

    @testset "Dispatch enforcement on unsupported part kinds" begin
        struct UnsupportedPart <: AbstractPart end
        struct UnsupportedTube <: AbstractTube end

        @test_throws MethodError saw(UnsupportedPart(), 10.0)
        @test_throws MethodError drill(UnsupportedPart(), Hole(1.0, 2.0))
        @test_throws MethodError drill(UnsupportedTube(), Hole(1.0, 2.0))
    end
end

using CoordinateTransformations: AffineMap, LinearMap, Translation
using Rotations: RotX, RotZ
using StaticArrays: SVector

at(d::Datum, x::Real, y::Real, z::Real=0.0) =
    Datum(d.transform ∘ Translation(SVector{3,Float64}(x, y, z)))

@testset "Primitives and manufacturability rules" begin
    @testset "Type hierarchy, Datum SE(3) transforms, and valid construction" begin
        stick = RectangularTube(3, 3, 288, 0.120)
        @test stick isa RectangularTube
        @test stick isa AbstractTube
        @test stick isa AbstractPart
        @test stick.width === 3.0
        @test stick.height === 3.0
        @test stick.length === 288.0
        @test stick.thickness === 0.120
        @test stick.parent === nothing
        @test stick.op === nothing
        @test parent(stick) === nothing
        @test operation(stick) === nothing
        @test isstock(stick)
        @test length(datums(stick)) == 4
        @test isempty(holes(stick))

        c = Circle(1)
        @test c isa Circle
        @test c isa AbstractProfile
        @test c.diameter === 1.0

        f = datum(stick, SVector(0, -1, 0))
        @test f isa Datum
        @test f isa AbstractDatum
        @test origin(f) ≈ SVector(0.0, 0.0, 0.0) atol = 1e-12
        @test xaxis(f) ≈ SVector(1.0, 0.0, 0.0) atol = 1e-12
        @test yaxis(f) ≈ SVector(0.0, 0.0, 1.0) atol = 1e-12
        @test zaxis(f) ≈ SVector(0.0, -1.0, 0.0) atol = 1e-12

        d = at(f, 1.5, 2.0)
        @test origin(d) ≈ SVector(1.5, 0.0, 2.0) atol = 1e-12

        @test origin(at(datum(stick, SVector(1, 0, 0)), 0.5, 10.0)) ≈ SVector(3.0, 0.5, 10.0) atol = 1e-12
        @test origin(at(datum(stick, SVector(0, 1, 0)), 0.8, 10.0)) ≈ SVector(2.2, 3.0, 10.0) atol = 1e-12
        @test origin(at(datum(stick, SVector(-1, 0, 0)), 0.5, 10.0)) ≈ SVector(0.0, 2.5, 10.0) atol = 1e-12

        @test ThroughAll() isa AbstractPenetration
        @test ThroughWall() isa AbstractPenetration
        @test Blind(0.5).depth === 0.5
        @test Saw(103).length === 103.0

        dr_op = Drill(c, d, ThroughAll())
        @test dr_op isa Drill
        @test dr_op isa AbstractOperation
        @test dr_op.profile === c
        @test dr_op.datum === d
        @test dr_op.penetration === ThroughAll()
    end

    @testset "Invalid construction" begin
        @test_throws DomainError RectangularTube(0.0, 3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, -3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, 0.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, 100.0, 0.0)
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 100.0, 1.5)

        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 103.0, 0.120, stick, nothing)

        @test_throws DomainError Circle(0.0)
        @test_throws DomainError Circle(-1.0)

        @test_throws MethodError Datum(Translation(0.0, 0.0, 0.0) ∘ LinearMap([2.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0]))
        @test_throws MethodError Datum(Translation(1.0, 2.0) ∘ LinearMap(RotX(0.0)))

        @test_throws DomainError Blind(0.0)
        @test_throws DomainError Saw(0.0)
    end

    @testset "Valid operation chains" begin
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        blank = saw(stick, 103.0)
        f = datum(blank, SVector(0, -1, 0))

        hole = Circle(1.0)
        upright = blank
        for y in 2.0:2.0:100.0
            upright = drill(upright, hole, at(f, 1.5, y), ThroughAll())
        end

        @test length(holes(upright)) == 50
        @test isstock(stick)
        @test !isstock(blank)
        @test parent(blank) === stick
        @test operation(blank) isa Saw
        @test operation(blank) === blank.op
        @test parent(upright) isa RectangularTube
        @test operation(upright) isa Drill
        @test isempty(holes(blank))

        # Sawing a drilled tube is allowed
        shortened = saw(upright, 50.0)
        @test shortened.length === 50.0
    end

    @testset "Invalid holes and saw cuts" begin
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        blank = saw(stick, 103.0)
        f = datum(blank, SVector(0, -1, 0))
        b = datum(blank, SVector(0, 1, 0))

        # Non-existent wall normal (+Z) or floating datum
        @test_throws ArgumentError datum(blank, SVector(0, 0, 1))
        @test_throws ArgumentError drill(blank, Circle(1.0), Datum(Translation(1.5, 1.5, 103.0) ∘ LinearMap(RotX(0.0))), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 1.5, 20.0, 0.5), ThroughAll())

        # Oversized / off-center / out-of-bounds
        @test_throws ArgumentError drill(blank, Circle(4.0), at(f, 1.5, 51.0), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(2.8), at(f, 1.5, 51.0), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 0.5, 51.0), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 2.5, 51.0), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 1.5, 0.5), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 1.5, 102.5), ThroughAll())

        # Saw longer than current length fails
        @test_throws ArgumentError saw(blank, 104.0)
    end

    @testset "Dispatch enforcement" begin
        struct UnsupportedPart <: AbstractPart end
        struct UnsupportedTube <: AbstractTube end
        struct SquareProfile <: AbstractProfile end

        blank = saw(RectangularTube(3.0, 3.0, 288.0, 0.120), 103.0)
        d = at(datum(blank, SVector(0, -1, 0)), 1.5, 2.0)

        @test_throws MethodError parent(UnsupportedPart())
        @test_throws MethodError operation(UnsupportedPart())
        @test_throws MethodError isstock(UnsupportedPart())
        @test_throws MethodError holes(UnsupportedPart())
        @test_throws MethodError graph(UnsupportedPart())
        @test_throws MethodError saw(UnsupportedPart(), 10.0)
        @test_throws MethodError drill(UnsupportedPart(), Circle(1.0), d, ThroughAll())
        @test_throws MethodError drill(UnsupportedTube(), Circle(1.0), d, ThroughAll())
        @test_throws MethodError drill(blank, SquareProfile(), d, ThroughAll())
        @test_throws MethodError drill(blank, Circle(1.0), d, Blind(0.5))
    end
end

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

        plate = Plate(6, 4, 0.1875)
        @test plate isa Plate
        @test plate isa AbstractPart
        @test plate.width === 6.0
        @test plate.height === 4.0
        @test plate.thickness === 0.1875
        @test plate.parent === nothing
        @test plate.op === nothing
        @test parent(plate) === nothing
        @test operation(plate) === nothing
        @test isstock(plate)
        @test length(datums(plate)) == 2
        @test isempty(holes(plate))

        top = datum(plate, SVector(0, 0, 1))
        @test top isa Datum
        @test origin(top) ≈ SVector(0.0, 0.0, 0.1875) atol = 1e-12
        @test xaxis(top) ≈ SVector(1.0, 0.0, 0.0) atol = 1e-12
        @test yaxis(top) ≈ SVector(0.0, 1.0, 0.0) atol = 1e-12
        @test zaxis(top) ≈ SVector(0.0, 0.0, 1.0) atol = 1e-12
        @test origin(at(top, 1.5, 2.0)) ≈ SVector(1.5, 2.0, 0.1875) atol = 1e-12

        bottom = datum(plate, SVector(0, 0, -1))
        @test bottom isa Datum
        @test origin(bottom) ≈ SVector(6.0, 0.0, 0.0) atol = 1e-12
        @test xaxis(bottom) ≈ SVector(-1.0, 0.0, 0.0) atol = 1e-12
        @test yaxis(bottom) ≈ SVector(0.0, 1.0, 0.0) atol = 1e-12
        @test zaxis(bottom) ≈ SVector(0.0, 0.0, -1.0) atol = 1e-12
        @test origin(at(bottom, 1.5, 2.0)) ≈ SVector(4.5, 2.0, 0.0) atol = 1e-12
    end

    @testset "Invalid construction" begin
        @test_throws DomainError RectangularTube(0.0, 3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, -3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, 0.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, 100.0, 0.0)
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 100.0, 1.5)

        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 103.0, 0.120, stick, nothing)

        @test_throws DomainError Plate(0.0, 4.0, 0.25)
        @test_throws DomainError Plate(-6.0, 4.0, 0.25)
        @test_throws DomainError Plate(6.0, 0.0, 0.25)
        @test_throws DomainError Plate(6.0, -4.0, 0.25)
        @test_throws DomainError Plate(6.0, 4.0, 0.0)
        @test_throws DomainError Plate(6.0, 4.0, -0.25)

        plate = Plate(6.0, 4.0, 0.25)
        @test_throws ArgumentError Plate(6.0, 4.0, 0.25, plate, nothing)
        @test_throws ArgumentError Plate(6.0, 4.0, 0.25, nothing, Saw(4.0))

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

        # Drilling a plate from top (+Z) and bottom (-Z) surfaces with ThroughAll, ThroughWall, and Blind
        raw_plate = Plate(8.0, 6.0, 0.25)
        top = datum(raw_plate, SVector(0, 0, 1))
        bottom = datum(raw_plate, SVector(0, 0, -1))
        p1 = drill(raw_plate, Circle(1.0), at(top, 2.0, 3.0), ThroughAll())
        p2 = drill(p1, Circle(1.0), at(bottom, 2.0, 3.0), ThroughWall())
        p3 = drill(p2, Circle(0.5), at(top, 4.0, 3.0), Blind(0.125))

        @test isstock(raw_plate)
        @test !isstock(p1)
        @test !isstock(p2)
        @test !isstock(p3)
        @test parent(p1) === raw_plate
        @test parent(p2) === p1
        @test parent(p3) === p2
        @test operation(p1) isa Drill
        @test operation(p1).penetration === ThroughAll()
        @test operation(p2) isa Drill
        @test operation(p2).penetration === ThroughWall()
        @test operation(p3) isa Drill
        @test operation(p3).penetration === Blind(0.125)
        @test length(holes(p3)) == 3
        @test origin(holes(p3)[1].datum) ≈ SVector(2.0, 3.0, 0.25) atol = 1e-12
        @test origin(holes(p3)[2].datum) ≈ SVector(6.0, 3.0, 0.0) atol = 1e-12
        @test origin(holes(p3)[3].datum) ≈ SVector(4.0, 3.0, 0.25) atol = 1e-12
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

        # Invalid plate datums and holes
        plate = Plate(6.0, 4.0, 0.25)
        top = datum(plate, SVector(0, 0, 1))
        bottom = datum(plate, SVector(0, 0, -1))

        # Edge normals do not exist on a 2.5D plate
        @test_throws ArgumentError datum(plate, SVector(1, 0, 0))
        @test_throws ArgumentError datum(plate, SVector(-1, 0, 0))
        @test_throws ArgumentError datum(plate, SVector(0, 1, 0))
        @test_throws ArgumentError datum(plate, SVector(0, -1, 0))
        @test_throws ArgumentError drill(plate, Circle(1.0), f, ThroughAll())

        # Floating datum off the plate surface
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 2.0, 0.1), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(bottom, 3.0, 2.0, -0.1), ThroughAll())

        # Oversized or edge-breaking holes
        @test_throws ArgumentError drill(plate, Circle(5.0), at(top, 3.0, 2.0), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 0.5, 2.0), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 5.5, 2.0), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 0.5), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 3.5), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(bottom, 0.5, 2.0), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(bottom, 5.5, 2.0), ThroughAll())

        # Blind hole depth >= plate thickness fails
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 2.0), Blind(0.25))
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 2.0), Blind(0.5))
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
        @test_throws MethodError ManufacturingGraph(UnsupportedPart())
        @test_throws MethodError saw(UnsupportedPart(), 10.0)
        @test_throws MethodError drill(UnsupportedPart(), Circle(1.0), d, ThroughAll())
        @test_throws MethodError drill(UnsupportedTube(), Circle(1.0), d, ThroughAll())
        @test_throws MethodError drill(blank, SquareProfile(), d, ThroughAll())
        @test_throws MethodError drill(blank, Circle(1.0), d, Blind(0.5))

        plate = Plate(6.0, 4.0, 0.25)
        dp = at(datum(plate, SVector(0, 0, 1)), 3.0, 2.0)
        @test_throws MethodError saw(plate, 3.0)
        @test_throws MethodError drill(plate, SquareProfile(), dp, ThroughAll())
    end
end

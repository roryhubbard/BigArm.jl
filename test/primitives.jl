using CoordinateTransformations: AffineMap, LinearMap, Translation
using Rotations: Rotation, RotX, RotY, RotZ
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
        @test isempty(operations(stick))

        c = Circle(1)
        @test c isa Circle
        @test c isa AbstractProfile
        @test c.diameter === 1.0

        rect = Rectangle(9, 8.5)
        @test rect isa Polygon
        @test rect isa AbstractProfile
        @test length(rect.points) == 4

        tri = Polygon([SVector(-1.0, -1.0), SVector(1.0, -1.0), SVector(0.0, 1.0)])
        @test tri isa Polygon
        @test tri isa AbstractProfile
        @test length(tri.points) == 3

        round_stick = RoundTube(1, 240, 0.120)
        @test round_stick isa RoundTube
        @test round_stick isa AbstractTube
        @test round_stick isa AbstractPart
        @test round_stick.outer_diameter === 1.0
        @test round_stick.length === 240.0
        @test round_stick.thickness === 0.120
        @test round_stick.parent === nothing
        @test round_stick.op === nothing
        @test parent(round_stick) === nothing
        @test operation(round_stick) === nothing
        @test isstock(round_stick)
        @test isempty(operations(round_stick))

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

        saw_d = Datum(Translation(0.0, 0.0, 103.0))
        saw_op = Saw(saw_d)
        @test saw_op isa Saw
        @test saw_op isa AbstractOperation
        @test saw_op.datum === saw_d

        dr_op = Drill(c, d, ThroughAll())
        @test dr_op isa Drill
        @test dr_op isa AbstractOperation
        @test dr_op.profile === c
        @test dr_op.datum === d
        @test dr_op.penetration === ThroughAll()

        cut_op = Cut(rect, d)
        @test cut_op isa Cut
        @test cut_op isa AbstractOperation
        @test cut_op.profile === rect
        @test cut_op.datum === d

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
        @test isempty(operations(plate))

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

        @test origin(Datum(Translation(1.0, 2.0, 3.0))) ≈ SVector(1.0, 2.0, 3.0) atol = 1e-12
        @test zaxis(Datum(LinearMap(RotX(π / 2)))) ≈ SVector(0.0, -1.0, 0.0) atol = 1e-12

        w = weld([
            stick => IdentityRigidTransform,
            plate => Translation(3.0, 0.0, 10.0) ∘ LinearMap(RotX(π / 2)),
        ])
        @test w isa Weldment
        @test w isa AbstractPart
        @test !isstock(w)
        @test operation(w) === Weld()
        @test operation(w) isa Weld
        @test operation(w) isa AbstractOperation
        @test length(w) == 2
        @test parent(w) == AbstractPart[stick, plate]
        @test w[1].first === stick
        @test w[2].first === plate
        @test sprint(show, w) == "Weldment(2 parts)"

        asm = Assembly([
            stick => IdentityRigidTransform,
            plate => Translation(3.0, 0.0, 10.0) ∘ LinearMap(RotX(π / 2)),
        ])
        @test asm isa Assembly
        @test !(asm isa AbstractPart)
        @test length(asm) == 2
        @test asm[1].first === stick
        @test asm[2].first === plate
        @test asm[1].second(SVector(1.0, 2.0, 3.0)) ≈ SVector(1.0, 2.0, 3.0) atol = 1e-12
        @test asm[2].second(SVector(1.0, 2.0, 0.0)) ≈ SVector(4.0, 0.0, 12.0) atol = 1e-12
        @test sprint(show, asm) == "Assembly(2 parts)"
    end

    @testset "Invalid construction" begin
        @test_throws DomainError RectangularTube(0.0, 3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, -3.0, 100.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, 0.0, 0.120)
        @test_throws DomainError RectangularTube(3.0, 3.0, 100.0, 0.0)
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 100.0, 1.5)

        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        @test_throws ArgumentError RectangularTube(3.0, 3.0, 103.0, 0.120, stick, nothing)

        @test_throws DomainError RoundTube(0.0, 20.0, 0.120)
        @test_throws DomainError RoundTube(-1.0, 20.0, 0.120)
        @test_throws DomainError RoundTube(1.0, 0.0, 0.120)
        @test_throws DomainError RoundTube(1.0, -20.0, 0.120)
        @test_throws DomainError RoundTube(1.0, 20.0, 0.0)
        @test_throws DomainError RoundTube(1.0, 20.0, -0.120)
        @test_throws ArgumentError RoundTube(1.0, 20.0, 0.5)
        @test_throws ArgumentError RoundTube(1.0, 20.0, 0.6)

        round_stick = RoundTube(1.0, 240.0, 0.120)
        saw_20 = Saw(Datum(Translation(0.0, 0.0, 20.0)))
        @test_throws ArgumentError RoundTube(1.0, 20.0, 0.120, round_stick, nothing)
        @test_throws ArgumentError RoundTube(1.0, 20.0, 0.120, nothing, saw_20)

        @test_throws DomainError Plate(0.0, 4.0, 0.25)
        @test_throws DomainError Plate(-6.0, 4.0, 0.25)
        @test_throws DomainError Plate(6.0, 0.0, 0.25)
        @test_throws DomainError Plate(6.0, -4.0, 0.25)
        @test_throws DomainError Plate(6.0, 4.0, 0.0)
        @test_throws DomainError Plate(6.0, 4.0, -0.25)

        plate = Plate(6.0, 4.0, 0.25)
        @test_throws ArgumentError Plate(6.0, 4.0, 0.25, plate, nothing)
        @test_throws ArgumentError Plate(6.0, 4.0, 0.25, nothing, saw_20)

        @test_throws DomainError Circle(0.0)
        @test_throws DomainError Circle(-1.0)
        @test_throws DomainError Rectangle(0.0, 4.0)
        @test_throws DomainError Rectangle(-2.0, 4.0)
        @test_throws DomainError Rectangle(2.0, 0.0)
        @test_throws DomainError Rectangle(2.0, -4.0)
        @test_throws ArgumentError Polygon([SVector(0.0, 0.0), SVector(1.0, 0.0)])
        @test_throws ArgumentError Polygon([SVector(0.0, 0.0), SVector(1.0, 0.0), SVector(2.0, 0.0)])

        @test_throws MethodError Datum(Translation(0.0, 0.0, 0.0) ∘ LinearMap([2.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0]))
        @test_throws MethodError Datum(Translation(1.0, 2.0) ∘ LinearMap(RotX(0.0)))

        @test_throws DomainError Blind(0.0)
        @test_throws MethodError Saw(0.0)

        @test_throws ArgumentError Weldment(Pair{AbstractPart,RigidTransform}[])
        @test_throws ArgumentError Weldment([plate => IdentityRigidTransform])
        @test_throws ArgumentError weld([plate => IdentityRigidTransform])
        @test_throws MethodError Weldment([
            plate => IdentityRigidTransform,
            stick => Translation(0.0, 0.0, 0.0) ∘ LinearMap([2.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0]),
        ])
        @test_throws MethodError Weldment([
            plate => IdentityRigidTransform,
            stick => Translation(1.0, 2.0) ∘ LinearMap(RotX(0.0)),
        ])

        @test_throws ArgumentError Assembly(Pair{AbstractPart,RigidTransform}[])
        @test_throws MethodError Assembly([plate => Translation(0.0, 0.0, 0.0) ∘ LinearMap([2.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0])])
        @test_throws MethodError Assembly([plate => Translation(1.0, 2.0) ∘ LinearMap(RotX(0.0))])
    end

    @testset "Valid operation chains" begin
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        blank = saw(stick, Datum(Translation(0.0, 0.0, 103.0)))
        f = datum(blank, SVector(0, -1, 0))

        hole = Circle(1.0)
        upright = blank
        for y in 2.0:2.0:100.0
            upright = drill(upright, hole, at(f, 1.5, y), ThroughAll())
        end

        @test length(operations_of_type(Drill, upright)) == 50
        @test length(operations_of_type(Saw, upright)) == 1
        @test isstock(stick)
        @test !isstock(blank)
        @test parent(blank) === stick
        @test operation(blank) isa Saw
        @test operation(blank) === blank.op
        @test parent(upright) isa RectangularTube
        @test operation(upright) isa Drill
        @test isempty(operations_of_type(Drill, blank))

        # Sawing a drilled tube is allowed
        shortened = saw(upright, Datum(Translation(0.0, 0.0, 50.0)))
        @test shortened.length === 50.0
        @test length(operations_of_type(Saw, shortened)) == 2

        # 45° miter saw cuts on top (+Z) and bottom (-Z) ends of a RectangularTube, plus drill and polygonal cut
        miter_top = saw(blank, Datum(Translation(1.5, 1.5, 50.0) ∘ LinearMap(RotY(-π / 4))))
        @test miter_top isa RectangularTube
        @test miter_top.length ≈ 51.5 atol = 1e-12
        miter_both = saw(miter_top, Datum(Translation(1.5, 1.5, 5.0) ∘ LinearMap(RotY(-3π / 4))))
        @test miter_both.length ≈ 51.5 - 3.5 atol = 1e-12
        miter_drilled = drill(miter_both, Circle(1.0), at(f, 1.5, 25.0), ThroughAll())
        miter_slotted = cut(miter_drilled, Rectangle(1.0, 4.0), at(f, 1.5, 35.0))
        @test length(operations_of_type(Drill, miter_slotted)) == 1
        @test length(operations_of_type(Cut, miter_slotted)) == 1

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
        p3_drills = operations_of_type(Drill, p3)
        @test length(p3_drills) == 3
        @test origin(p3_drills[1].datum) ≈ SVector(2.0, 3.0, 0.25) atol = 1e-12
        @test origin(p3_drills[2].datum) ≈ SVector(6.0, 3.0, 0.0) atol = 1e-12
        @test origin(p3_drills[3].datum) ≈ SVector(4.0, 3.0, 0.25) atol = 1e-12

        # Bandsawing a Plate along straight and angled lines in XY
        sawn_plate = saw(raw_plate, Datum(Translation(6.0, 0.0, 0.0) ∘ LinearMap(RotY(π / 2))))
        @test sawn_plate isa Plate
        @test sawn_plate.width ≈ 6.0 atol = 1e-12
        @test sawn_plate.height ≈ 6.0 atol = 1e-12
        chamfered_plate = saw(
            sawn_plate,
            Datum(Translation(4.0, 6.0, 0.0) ∘ LinearMap(RotZ(π / 4) * RotY(π / 2))),
        )
        @test length(operations_of_type(Saw, chamfered_plate)) == 2
        chamfered_drilled = drill(chamfered_plate, Circle(1.0), at(top, 2.0, 2.0), ThroughAll())
        @test length(operations_of_type(Drill, chamfered_drilled)) == 1

        # Sawing round tube stock (steel axle shaft and foam roller), including miter cuts
        axle_stock = RoundTube(1.0, 240.0, 0.120)
        axle = saw(axle_stock, Datum(Translation(0.0, 0.0, 20.0)))
        @test axle isa RoundTube
        @test axle.outer_diameter === 1.0
        @test axle.length === 20.0
        @test axle.thickness === 0.120
        @test !isstock(axle)
        @test parent(axle) === axle_stock
        @test origin(operation(axle).datum) ≈ SVector(0.0, 0.0, 20.0) atol = 1e-12
        @test saw(axle, Datum(Translation(0.0, 0.0, 10.0))).length === 10.0
        miter_axle = saw(axle, Datum(Translation(0.0, 0.0, 15.0) ∘ LinearMap(RotX(π / 4))))
        @test miter_axle.length ≈ 15.0 + 0.5 * tan(π / 4) atol = 1e-12

        roller_stock = RoundTube(4.5, 96.0, 1.75)
        roller = saw(roller_stock, Datum(Translation(0.0, 0.0, 8.0)))
        @test roller isa RoundTube
        @test roller.outer_diameter === 4.5
        @test roller.length === 8.0
        @test roller.thickness === 1.75
        @test parent(roller) === roller_stock

        # Cutting rectangular and arbitrary polygonal through-windows in a Plate (and combining with drill)
        footplate_stock = Plate(24.0, 14.0, 0.1875)
        fp_top = datum(footplate_stock, SVector(0, 0, 1))
        fp_bottom = datum(footplate_stock, SVector(0, 0, -1))
        windowed = cut(footplate_stock, Rectangle(9.0, 8.5), at(fp_top, 12.0, 7.0))
        windowed_drilled = drill(windowed, Circle(1.0), at(fp_top, 3.0, 7.0), ThroughAll())
        tri_cutout = Polygon([SVector(-1.0, -0.75), SVector(1.0, -0.75), SVector(0.0, 0.75)])
        windowed_two = cut(windowed_drilled, tri_cutout, at(fp_bottom, 3.0, 7.0))

        @test windowed isa Plate
        @test !isstock(windowed)
        @test parent(windowed) === footplate_stock
        @test operation(windowed) isa Cut
        @test operation(windowed).profile.points == Rectangle(9.0, 8.5).points
        @test length(operations_of_type(Cut, windowed)) == 1
        @test isempty(operations_of_type(Drill, windowed))
        cut_ops = operations_of_type(Cut, windowed_two)
        @test length(cut_ops) == 2
        @test length(operations_of_type(Drill, windowed_two)) == 1
        @test origin(cut_ops[1].datum) ≈ SVector(12.0, 7.0, 0.1875) atol = 1e-12
        @test origin(cut_ops[2].datum) ≈ SVector(21.0, 7.0, 0.0) atol = 1e-12
    end

    @testset "Invalid holes and saw cuts" begin
        stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
        blank = saw(stick, Datum(Translation(0.0, 0.0, 103.0)))
        f = datum(blank, SVector(0, -1, 0))
        b = datum(blank, SVector(0, 1, 0))

        # Non-existent wall normal (+Z) or floating datum
        @test_throws ArgumentError datum(blank, SVector(0, 0, 1))
        @test_throws ArgumentError drill(blank, Circle(1.0), Datum(Translation(1.5, 1.5, 103.0)), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 1.5, 20.0, 0.5), ThroughAll())

        # Oversized / off-center / out-of-bounds on RectangularTube
        @test_throws ArgumentError drill(blank, Circle(4.0), at(f, 1.5, 51.0), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(2.8), at(f, 1.5, 51.0), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 0.5, 51.0), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 2.5, 51.0), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 1.5, 0.5), ThroughAll())
        @test_throws ArgumentError drill(blank, Circle(1.0), at(f, 1.5, 102.5), ThroughAll())
        @test_throws ArgumentError cut(blank, Rectangle(3.0, 2.0), at(f, 1.5, 51.0))
        @test_throws ArgumentError cut(blank, Rectangle(1.0, 2.0), at(f, 1.5, 1.0))
        @test_throws ArgumentError cut(blank, Rectangle(1.0, 2.0), at(f, 1.5, 102.0))

        # Invalid tube saw cuts: beyond top, at/below bottom, or zero Z-normal
        @test_throws ArgumentError saw(blank, Datum(Translation(0.0, 0.0, 104.0)))
        @test_throws ArgumentError saw(blank, Datum(Translation(0.0, 0.0, 0.0)))
        @test_throws ArgumentError saw(blank, Datum(Translation(0.0, 0.0, -5.0)))
        @test_throws ArgumentError saw(blank, Datum(Translation(1.5, 1.5, 50.0) ∘ LinearMap(RotY(π / 2))))
        # Miter cut extending past the 103.0 top end on one side fails
        @test_throws ArgumentError saw(blank, Datum(Translation(1.5, 1.5, 102.5) ∘ LinearMap(RotY(-π / 4))))

        round_blank = saw(RoundTube(1.0, 240.0, 0.120), Datum(Translation(0.0, 0.0, 20.0)))
        @test_throws ArgumentError saw(round_blank, Datum(Translation(0.0, 0.0, 21.0)))
        @test_throws ArgumentError saw(round_blank, Datum(Translation(0.0, 0.0, 0.0)))
        @test_throws ArgumentError saw(round_blank, Datum(Translation(0.0, 0.0, 10.0) ∘ LinearMap(RotY(π / 2))))

        # Invalid plate datums, saw cuts, and holes
        plate = Plate(6.0, 4.0, 0.25)
        top = datum(plate, SVector(0, 0, 1))
        bottom = datum(plate, SVector(0, 0, -1))

        # Plate saw normal must lie in XY plane and slice through the plate interior
        @test_throws ArgumentError saw(plate, Datum(Translation(0.0, 0.0, 0.1)))
        @test_throws ArgumentError saw(plate, Datum(Translation(3.0, 2.0, 0.0) ∘ LinearMap(RotY(π / 4))))
        @test_throws ArgumentError saw(plate, Datum(Translation(7.0, 0.0, 0.0) ∘ LinearMap(RotY(π / 2))))
        @test_throws ArgumentError saw(plate, Datum(Translation(-1.0, 0.0, 0.0) ∘ LinearMap(RotY(π / 2))))

        # Hole or cutout encroaching on a bandsawn edge fails
        chamfered = saw(
            plate,
            Datum(Translation(3.0, 4.0, 0.0) ∘ LinearMap(RotZ(π / 4) * RotY(π / 2))),
        )
        @test_throws ArgumentError drill(chamfered, Circle(1.0), at(top, 5.0, 3.0), ThroughAll())
        @test_throws ArgumentError cut(chamfered, Rectangle(2.0, 2.0), at(top, 4.5, 2.5))

        # Edge normals do not exist on a 2.5D plate
        @test_throws ArgumentError datum(plate, SVector(1, 0, 0))
        @test_throws ArgumentError datum(plate, SVector(-1, 0, 0))
        @test_throws ArgumentError datum(plate, SVector(0, 1, 0))
        @test_throws ArgumentError datum(plate, SVector(0, -1, 0))
        @test_throws ArgumentError drill(plate, Circle(1.0), f, ThroughAll())
        @test_throws ArgumentError cut(plate, Rectangle(1.0, 1.0), f)

        # Floating datum off the plate surface
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 2.0, 0.1), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(bottom, 3.0, 2.0, -0.1), ThroughAll())
        @test_throws ArgumentError cut(plate, Rectangle(1.0, 1.0), at(top, 3.0, 2.0, 0.1))
        @test_throws ArgumentError cut(plate, Rectangle(1.0, 1.0), at(bottom, 3.0, 2.0, -0.1))

        # Oversized or edge-breaking holes
        @test_throws ArgumentError drill(plate, Circle(5.0), at(top, 3.0, 2.0), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 0.5, 2.0), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 5.5, 2.0), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 0.5), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 3.5), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(bottom, 0.5, 2.0), ThroughAll())
        @test_throws ArgumentError drill(plate, Circle(1.0), at(bottom, 5.5, 2.0), ThroughAll())

        # Oversized or edge-breaking rectangular cutouts
        @test_throws ArgumentError cut(plate, Rectangle(6.0, 2.0), at(top, 3.0, 2.0))
        @test_throws ArgumentError cut(plate, Rectangle(2.0, 4.0), at(top, 3.0, 2.0))
        @test_throws ArgumentError cut(plate, Rectangle(2.0, 1.0), at(top, 1.0, 2.0))
        @test_throws ArgumentError cut(plate, Rectangle(2.0, 1.0), at(top, 5.0, 2.0))
        @test_throws ArgumentError cut(plate, Rectangle(2.0, 1.0), at(top, 3.0, 0.5))
        @test_throws ArgumentError cut(plate, Rectangle(2.0, 1.0), at(top, 3.0, 3.5))
        @test_throws ArgumentError cut(plate, Rectangle(2.0, 1.0), at(bottom, 1.0, 2.0))
        @test_throws ArgumentError cut(plate, Rectangle(2.0, 1.0), at(bottom, 5.0, 2.0))

        # Blind hole depth >= plate thickness fails
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 2.0), Blind(0.25))
        @test_throws ArgumentError drill(plate, Circle(1.0), at(top, 3.0, 2.0), Blind(0.5))
    end

    @testset "Dispatch enforcement" begin
        struct UnsupportedPart <: AbstractPart end
        struct UnsupportedTube <: AbstractTube end
        struct SquareProfile <: AbstractProfile end

        blank = saw(RectangularTube(3.0, 3.0, 288.0, 0.120), Datum(Translation(0.0, 0.0, 103.0)))
        round_blank = saw(RoundTube(1.0, 240.0, 0.120), Datum(Translation(0.0, 0.0, 20.0)))
        d = at(datum(blank, SVector(0, -1, 0)), 1.5, 2.0)

        @test_throws MethodError saw(UnsupportedPart(), Datum(Translation(0.0, 0.0, 10.0)))
        @test_throws MethodError saw(blank, 10.0)
        @test_throws MethodError drill(UnsupportedPart(), Circle(1.0), d, ThroughAll())
        @test_throws MethodError drill(UnsupportedTube(), Circle(1.0), d, ThroughAll())
        @test_throws MethodError drill(round_blank, Circle(0.25), d, ThroughAll())
        @test_throws MethodError datums(round_blank)
        @test_throws MethodError drill(blank, SquareProfile(), d, ThroughAll())
        @test_throws MethodError drill(blank, Rectangle(1.0, 1.0), d, ThroughAll())
        @test_throws MethodError drill(blank, Circle(1.0), d, Blind(0.5))
        @test_throws MethodError cut(round_blank, Rectangle(0.5, 0.5), d)
        @test_throws MethodError cut(blank, Rectangle(1.0, 1.0), d, ThroughAll())

        plate = Plate(6.0, 4.0, 0.25)
        dp = at(datum(plate, SVector(0, 0, 1)), 3.0, 2.0)
        @test_throws MethodError saw(plate, 3.0)
        @test_throws MethodError drill(plate, SquareProfile(), dp, ThroughAll())
        @test_throws MethodError drill(plate, Rectangle(1.0, 1.0), dp, ThroughAll())
        @test_throws MethodError cut(UnsupportedPart(), Rectangle(1.0, 1.0), dp)
        @test_throws MethodError cut(plate, SquareProfile(), dp)
        @test_throws MethodError cut(plate, Circle(1.0), dp)
        @test_throws MethodError cut(plate, Rectangle(1.0, 1.0), dp, ThroughAll())

        w = weld([blank => IdentityRigidTransform, plate => IdentityRigidTransform])
        @test_throws MethodError saw(w, Datum(Translation(0.0, 0.0, 10.0)))
        @test_throws MethodError drill(w, Circle(1.0), dp, ThroughAll())
        @test_throws MethodError cut(w, Rectangle(1.0, 1.0), dp)
        @test_throws MethodError datums(w)
    end
end

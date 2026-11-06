using CoordinateTransformations: Translation
using StaticArrays: SVector

place(d::Datum, x::Real, y::Real, z::Real=0.0) =
    Datum(d.transform ∘ Translation(SVector{3,Float64}(x, y, z)))

function chain_parts()
    stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
    blank = saw(stick, 103.0)
    front = datum(blank, SVector(0, -1, 0))
    first = drill(blank, Circle(1.0), place(front, 1.5, 10.0), ThroughAll())
    second = drill(first, Circle(0.75), place(front, 1.5, 20.0), ThroughWall())
    return (; stick, blank, front, first, second)
end

function fork_parts()
    stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
    blank = saw(stick, 103.0)
    front = datum(blank, SVector(0, -1, 0))
    right = datum(blank, SVector(1, 0, 0))
    a = drill(blank, Circle(1.0), place(front, 1.5, 10.0), ThroughAll())
    b = drill(blank, Circle(0.5), place(front, 1.5, 30.0), ThroughWall())
    c = drill(a, Circle(0.5), place(right, 1.5, 40.0), ThroughAll())
    return (; stick, blank, front, right, a, b, c)
end

@testset "Manufacturing graph" begin
    @testset "Node counts and edge ordering" begin
        parts = chain_parts()
        g = ManufacturingGraph(parts.second)

        @test g isa ManufacturingGraph
        @test length(g.nodes) == 4
        @test g.nodes[1] === parts.stick
        @test g.nodes[2] === parts.blank
        @test g.nodes[3] === parts.first
        @test g.nodes[4] === parts.second

        @test isstock(g.nodes[1])
        @test isnothing(operation(g.nodes[1]))
        for i in 2:length(g.nodes)
            @test parent(g.nodes[i]) === g.nodes[i - 1]
            @test operation(g.nodes[i]) isa AbstractOperation
        end
        @test operation(g.nodes[2]) isa Saw
        @test operation(g.nodes[2]).length === 103.0
        @test operation(g.nodes[3]) isa Drill
        @test operation(g.nodes[3]).penetration isa ThroughAll
        @test operation(g.nodes[4]) isa Drill
        @test operation(g.nodes[4]).penetration isa ThroughWall

        @test !isstock(g.nodes[end])
        @test isempty(holes(g.nodes[1]))
        @test length(holes(g.nodes[end])) == 2

        stock = ManufacturingGraph(parts.stick)
        @test length(stock.nodes) == 1
        @test stock.nodes[1] === parts.stick
        @test isstock(only(stock.nodes))
        @test_throws MethodError ManufacturingGraph(parts.stick, parts.blank)
    end

    @testset "A part's history is one chain" begin
        parts = fork_parts()
        from_c = ManufacturingGraph(parts.c)
        @test length(from_c.nodes) == 4
        @test from_c.nodes[1] === parts.stick
        @test from_c.nodes[2] === parts.blank
        @test from_c.nodes[3] === parts.a
        @test from_c.nodes[4] === parts.c
        @test !any(node -> node === parts.b, from_c.nodes)

        from_a = ManufacturingGraph(parts.a)
        @test length(from_a.nodes) == 3
        @test from_a.nodes[end] === parts.a
        @test !any(node -> node === parts.b || node === parts.c, from_a.nodes)
    end

    @testset "Operation readbacks" begin
        parts = chain_parts()
        ops = operations(parts.second)

        @test ops isa Vector{AbstractOperation}
        @test length(ops) == 3
        @test ops[1] === operation(parts.blank)
        @test ops[2] === operation(parts.first)
        @test ops[3] === operation(parts.second)
        @test ops == [operation(node) for node in ManufacturingGraph(parts.second).nodes if !isstock(node)]
        @test isempty(operations(parts.stick))
        @test length(operations(parts.blank)) == 1
        @test operations(parts.blank)[1] isa Saw

        @test ops[2] isa Drill
        @test ops[2].profile.diameter === 1.0
        @test ops[2].penetration isa ThroughAll
        @test origin(ops[2].datum) ≈ SVector(1.5, 0.0, 10.0) atol = 1e-9
        @test ops[3] isa Drill
        @test ops[3].profile.diameter === 0.75
        @test ops[3].penetration isa ThroughWall
        @test origin(ops[3].datum) ≈ SVector(1.5, 0.0, 20.0) atol = 1e-9

        shortened = saw(parts.second, 40.0)
        shortened_ops = operations(shortened)
        @test length(shortened_ops) == 4
        @test shortened_ops[1] isa Saw
        @test shortened_ops[1].length === 103.0
        @test shortened_ops[2] isa Drill
        @test shortened_ops[3] isa Drill
        @test shortened_ops[4] isa Saw
        @test shortened_ops[4].length === 40.0

        forked = fork_parts()
        @test length(operations(forked.a)) == 2
        @test operations(forked.a)[end].profile.diameter === 1.0
        @test length(operations(forked.b)) == 2
        @test operations(forked.b)[end].profile.diameter === 0.5
        @test operations(forked.b)[end].penetration isa ThroughWall
        @test length(operations(forked.c)) == 3
        @test operations(forked.c)[end].penetration isa ThroughAll
        @test origin(operations(forked.c)[end].datum) ≈ SVector(3.0, 1.5, 40.0) atol = 1e-9

        stick = RectangularTube(3.0, 3.0, 80.0, 0.120)
        blank = saw(stick, 40.0)
        front = datum(blank, SVector(0, -1, 0))
        upright = blank
        ys = 2.0:2.0:20.0
        for y in ys
            upright = drill(upright, Circle(1.0), place(front, 1.5, y), ThroughAll())
        end
        drilled = ManufacturingGraph(upright)
        repeated = filter(op -> op isa Drill, operations(upright))
        @test length(drilled.nodes) == 2 + length(ys)
        @test length(operations(upright)) == length(drilled.nodes) - 1
        @test operation(drilled.nodes[2]) isa Saw
        @test length(repeated) == length(ys)
        @test all(op -> op.profile.diameter === 1.0 && op.penetration isa ThroughAll, repeated)
        @test [origin(op.datum)[3] for op in repeated] ≈ collect(ys) atol = 1e-9
    end

    @testset "DOT string generation" begin
        parts = chain_parts()
        one = drill(parts.blank, Circle(1.0), place(parts.front, 1.5, 10.0), ThroughAll())
        expected = join([
            "digraph ManufacturingGraph {",
            "  rankdir=TB;",
            "  node [shape=box];",
            "  n0 [label=\"stock RectangularTube\\n3.0 × 3.0 × 288.0\\nt=0.12\"];",
            "  n1 [label=\"part RectangularTube\\n3.0 × 3.0 × 103.0\\nt=0.12\"];",
            "  n2 [label=\"part RectangularTube\\n3.0 × 3.0 × 103.0\\nt=0.12\"];",
            "  n0 -> n1 [label=\"Saw(103.0)\"];",
            "  n1 -> n2 [label=\"Drill(Circle(1.0), (1.5, 0.0, 10.0), ThroughAll)\"];",
            "}",
        ], '\n') * '\n'
        @test to_dot(ManufacturingGraph(one)) == expected

        dot = to_dot(ManufacturingGraph(parts.second))
        @test startswith(dot, "digraph ManufacturingGraph {\n")
        @test endswith(dot, "}\n")
        @test occursin("rankdir=TB;", dot)
        @test occursin("node [shape=box];", dot)
        @test occursin("n0 [label=\"stock RectangularTube\\n3.0 × 3.0 × 288.0\\nt=0.12\"];", dot)
        @test occursin("n3 [label=\"part RectangularTube\\n3.0 × 3.0 × 103.0\\nt=0.12\"];", dot)
        @test occursin("n0 -> n1 [label=\"Saw(103.0)\"];", dot)
        @test occursin("n2 -> n3 [label=\"Drill(Circle(0.75), (1.5, 0.0, 20.0), ThroughWall)\"];", dot)
        @test count(==('{'), dot) == 1
        @test count(==('}'), dot) == 1
        @test iseven(count(==('"'), dot))

        lines = split(chomp(dot), '\n')
        node_lines = filter(line -> occursin(r"^  n\d+ \[label=", line), lines)
        edge_lines = filter(line -> occursin("->", line), lines)
        @test length(node_lines) == 4
        @test length(edge_lines) == 3
        @test findfirst(line -> occursin("[label=", line), lines) < findfirst(line -> occursin("->", line), lines)
        for line in edge_lines
            matched = match(r"n(\d+) -> n(\d+)", line)
            @test matched !== nothing
            @test parse(Int, matched[1]) < parse(Int, matched[2])
        end
        @test !occursin("stock", node_lines[end])
        @test !any(line -> occursin("Saw(", line) || occursin("Drill(", line), node_lines)
        @test all(line -> occursin("Saw(", line) || occursin("Drill(", line), edge_lines)

        stock_dot = to_dot(ManufacturingGraph(parts.stick))
        @test stock_dot == join([
            "digraph ManufacturingGraph {",
            "  rankdir=TB;",
            "  node [shape=box];",
            "  n0 [label=\"stock RectangularTube\\n3.0 × 3.0 × 288.0\\nt=0.12\"];",
            "}",
        ], '\n') * '\n'
        @test !occursin("->", stock_dot)

        deep_dot = to_dot(ManufacturingGraph(fork_parts().c))
        @test occursin("Drill(Circle(0.5), (3.0, 1.5, 40.0), ThroughAll)", deep_dot)

        @test sprint(show, ManufacturingGraph(parts.second)) == "ManufacturingGraph(4 nodes)"
        @test !occursin("AffineMap", sprint(show, ManufacturingGraph(parts.second)))
        @test startswith(sprint(show, MIME("text/plain"), ManufacturingGraph(one)), "digraph ManufacturingGraph {")

        raw_plate = Plate(8.0, 4.0, 0.1875)
        top = datum(raw_plate, SVector(0, 0, 1))
        plate_h1 = drill(raw_plate, Circle(1.0), place(top, 2.0, 2.0), ThroughAll())
        plate_h2 = drill(plate_h1, Circle(0.625), place(top, 6.0, 2.0), ThroughAll())
        plate_g = ManufacturingGraph(plate_h2)
        @test length(plate_g.nodes) == 3
        @test plate_g.nodes[1] === raw_plate
        @test plate_g.nodes[2] === plate_h1
        @test plate_g.nodes[3] === plate_h2
        @test length(operations(plate_h2)) == 2
        @test all(op -> op isa Drill && op.penetration isa ThroughAll, operations(plate_h2))
        expected_plate_dot = join([
            "digraph ManufacturingGraph {",
            "  rankdir=TB;",
            "  node [shape=box];",
            "  n0 [label=\"stock Plate\\n8.0 × 4.0\\nt=0.1875\"];",
            "  n1 [label=\"part Plate\\n8.0 × 4.0\\nt=0.1875\"];",
            "  n2 [label=\"part Plate\\n8.0 × 4.0\\nt=0.1875\"];",
            "  n0 -> n1 [label=\"Drill(Circle(1.0), (2.0, 2.0, 0.1875), ThroughAll)\"];",
            "  n1 -> n2 [label=\"Drill(Circle(0.625), (6.0, 2.0, 0.1875), ThroughAll)\"];",
            "}",
        ], '\n') * '\n'
        @test to_dot(plate_g) == expected_plate_dot
    end
end

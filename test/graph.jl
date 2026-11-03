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
        g = graph(parts.second)

        @test g isa ManufacturingGraph
        @test length(g.nodes) == 4
        @test length(g.edges) == 3
        @test g.nodes[1] === parts.stick
        @test g.nodes[2] === parts.blank
        @test g.nodes[3] === parts.first
        @test g.nodes[4] === parts.second

        for (i, edge) in enumerate(g.edges)
            @test edge isa ManufacturingEdge
            @test edge.parent === g.nodes[i]
            @test edge.child === g.nodes[i + 1]
            @test edge.op === operation(edge.child)
            @test edge.parent === parent(edge.child)
        end

        @test g.edges[1].op isa Saw
        @test g.edges[1].op.length === 103.0
        @test g.edges[2].op isa Drill
        @test g.edges[2].op.penetration isa ThroughAll
        @test g.edges[3].op isa Drill
        @test g.edges[3].op.penetration isa ThroughWall

        @test isstock(g.nodes[1])
        @test !isstock(g.nodes[end])
        @test isempty(holes(g.nodes[1]))
        @test length(holes(g.nodes[end])) == 2
        @test length(ManufacturingGraph(g.nodes, g.edges).edges) == length(g.edges)

        stock = graph(parts.stick)
        @test length(stock.nodes) == 1
        @test stock.nodes[1] === parts.stick
        @test isempty(stock.edges)

        again = graph(parts.second, parts.stick, parts.blank, parts.first)
        @test length(again.nodes) == 4
        @test again.nodes[1] === parts.stick
        @test again.nodes[2] === parts.blank
        @test again.nodes[3] === parts.first
        @test again.nodes[4] === parts.second

        empty = graph()
        @test isempty(empty.nodes)
        @test isempty(empty.edges)
    end

    @testset "Shared-ancestor forks" begin
        parts = fork_parts()
        g = graph(parts.a, parts.b)

        @test length(g.nodes) == 4
        @test length(g.edges) == 3
        @test g.nodes[1] === parts.stick
        @test g.nodes[2] === parts.blank
        @test g.nodes[3] === parts.a
        @test g.nodes[4] === parts.b
        @test count(node -> node === parts.stick, g.nodes) == 1
        @test count(node -> node === parts.blank, g.nodes) == 1
        @test count(edge -> edge.parent === parts.blank, g.edges) == 2
        @test count(edge -> edge.op isa Saw, g.edges) == 1
        @test g.edges[1].parent === parts.stick
        @test g.edges[1].child === parts.blank
        @test g.edges[2].parent === parts.blank
        @test g.edges[2].child === parts.a
        @test g.edges[3].parent === parts.blank
        @test g.edges[3].child === parts.b

        swapped = graph(parts.b, parts.a)
        @test swapped.nodes[1] === parts.stick
        @test swapped.nodes[2] === parts.blank
        @test swapped.nodes[3] === parts.b
        @test swapped.nodes[4] === parts.a
        @test swapped.edges[2].child === parts.b
        @test swapped.edges[3].child === parts.a

        only_a = graph(parts.a)
        @test length(only_a.nodes) == 3
        @test only_a.nodes[end] === parts.a
        @test !any(node -> node === parts.b, only_a.nodes)

        # c continues a; b forks from the shared blank.
        deep = graph(parts.c, parts.b)
        @test length(deep.nodes) == 5
        @test length(deep.edges) == 4
        @test deep.nodes[1] === parts.stick
        @test deep.nodes[2] === parts.blank
        @test deep.nodes[3] === parts.a
        @test deep.nodes[4] === parts.c
        @test deep.nodes[5] === parts.b
        @test deep.edges[3].parent === parts.a
        @test deep.edges[3].child === parts.c
        @test deep.edges[4].parent === parts.blank
        @test deep.edges[4].child === parts.b
        @test count(node -> node === parts.blank, deep.nodes) == 1

        s1 = RectangularTube(3.0, 3.0, 288.0, 0.120)
        s2 = RectangularTube(2.0, 4.0, 80.0, 0.100)
        p1 = saw(s1, 40.0)
        p2 = saw(s2, 20.0)
        separate = graph(p2, p1)
        @test length(separate.nodes) == 4
        @test length(separate.edges) == 2
        @test separate.nodes[1] === s2
        @test separate.nodes[2] === p2
        @test separate.nodes[3] === s1
        @test separate.nodes[4] === p1
        @test separate.edges[1].parent === s2
        @test separate.edges[1].child === p2
        @test separate.edges[2].parent === s1
        @test separate.edges[2].child === p1
    end

    @testset "Operations and hole-table readbacks" begin
        parts = chain_parts()
        ops = operations(parts.second)

        @test ops isa Vector{AbstractOperation}
        @test length(ops) == 3
        @test ops[1] === operation(parts.blank)
        @test ops[2] === operation(parts.first)
        @test ops[3] === operation(parts.second)
        @test ops == [edge.op for edge in graph(parts.second).edges]
        @test isempty(operations(parts.stick))
        @test length(operations(parts.blank)) == 1
        @test operations(parts.blank)[1] isa Saw

        table = hole_table(parts.second)
        @test length(table) == 2
        @test [row.index for row in table] == [1, 2]
        @test table[1] isa HoleTableRow
        @test table[1].diameter === 1.0
        @test table[1].local_x ≈ 1.5 atol = 1e-9
        @test table[1].local_y ≈ 10.0 atol = 1e-9
        @test table[1].position ≈ SVector(1.5, 0.0, 10.0) atol = 1e-9
        @test table[1].normal ≈ SVector(0.0, -1.0, 0.0) atol = 1e-9
        @test table[1].penetration isa ThroughAll
        @test table[2].diameter === 0.75
        @test table[2].local_x ≈ 1.5 atol = 1e-9
        @test table[2].local_y ≈ 20.0 atol = 1e-9
        @test table[2].position ≈ SVector(1.5, 0.0, 20.0) atol = 1e-9
        @test table[2].penetration isa ThroughWall

        drills = filter(op -> op isa Drill, ops)
        @test length(table) == length(drills)
        for (row, op) in zip(table, drills)
            @test row.diameter === op.profile.diameter
            @test row.penetration === op.penetration
            @test row.position ≈ origin(op.datum) atol = 1e-12
            @test row.normal ≈ zaxis(op.datum) atol = 1e-12
        end

        @test isempty(hole_table(parts.stick))
        @test isempty(hole_table(parts.blank))

        shortened = saw(parts.second, 40.0)
        shortened_ops = operations(shortened)
        @test length(shortened_ops) == 4
        @test shortened_ops[1] isa Saw
        @test shortened_ops[1].length === 103.0
        @test shortened_ops[2] isa Drill
        @test shortened_ops[3] isa Drill
        @test shortened_ops[4] isa Saw
        @test shortened_ops[4].length === 40.0
        shortened_table = hole_table(shortened)
        @test length(shortened_table) == 2
        @test shortened_table[1].local_y ≈ table[1].local_y atol = 1e-9
        @test shortened_table[2].diameter === table[2].diameter
        @test shortened_table[2].penetration isa ThroughWall

        back = datum(parts.blank, SVector(0, 1, 0))
        backed = drill(parts.blank, Circle(0.5), place(back, 0.8, 12.0), ThroughWall())
        back_row = only(hole_table(backed))
        @test back_row.index == 1
        @test back_row.diameter === 0.5
        @test back_row.local_x ≈ 0.8 atol = 1e-9
        @test back_row.local_y ≈ 12.0 atol = 1e-9
        @test back_row.position ≈ SVector(2.2, 3.0, 12.0) atol = 1e-9
        @test back_row.normal ≈ SVector(0.0, 1.0, 0.0) atol = 1e-9
        @test back_row.penetration isa ThroughWall

        forked = fork_parts()
        @test length(hole_table(forked.a)) == 1
        @test length(hole_table(forked.b)) == 1
        @test hole_table(forked.a)[1].diameter === 1.0
        @test hole_table(forked.b)[1].diameter === 0.5
        @test hole_table(forked.b)[1].penetration isa ThroughWall
        @test length(operations(forked.a)) == 2
        @test length(operations(forked.c)) == 3
        @test operations(forked.c)[end].penetration isa ThroughAll
        right_row = hole_table(forked.c)[2]
        @test right_row.local_x ≈ 1.5 atol = 1e-9
        @test right_row.local_y ≈ 40.0 atol = 1e-9
        @test right_row.position ≈ SVector(3.0, 1.5, 40.0) atol = 1e-9
        @test right_row.normal ≈ SVector(1.0, 0.0, 0.0) atol = 1e-9

        stick = RectangularTube(3.0, 3.0, 80.0, 0.120)
        blank = saw(stick, 40.0)
        front = datum(blank, SVector(0, -1, 0))
        upright = blank
        ys = 2.0:2.0:20.0
        for y in ys
            upright = drill(upright, Circle(1.0), place(front, 1.5, y), ThroughAll())
        end
        drilled = graph(upright)
        @test length(drilled.nodes) == 2 + length(ys)
        @test length(drilled.edges) == length(drilled.nodes) - 1
        @test length(operations(upright)) == length(drilled.edges)
        @test drilled.edges[1].op isa Saw
        repeated = hole_table(upright)
        @test length(repeated) == length(ys)
        @test [row.index for row in repeated] == 1:length(ys)
        @test [row.local_y for row in repeated] ≈ collect(ys) atol = 1e-9
        @test all(row -> row.local_x ≈ 1.5 && row.diameter === 1.0 && row.penetration isa ThroughAll, repeated)
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
        @test to_dot(graph(one)) == expected

        dot = to_dot(graph(parts.second))
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

        stock_dot = to_dot(graph(parts.stick))
        @test stock_dot == join([
            "digraph ManufacturingGraph {",
            "  rankdir=TB;",
            "  node [shape=box];",
            "  n0 [label=\"stock RectangularTube\\n3.0 × 3.0 × 288.0\\nt=0.12\"];",
            "}",
        ], '\n') * '\n'
        @test !occursin("->", stock_dot)

        @test to_dot(graph()) == "digraph ManufacturingGraph {\n  rankdir=TB;\n  node [shape=box];\n}\n"

        forked = fork_parts()
        fork_dot = to_dot(graph(forked.a, forked.b))
        @test occursin("n1 -> n2 [label=\"Drill(Circle(1.0), (1.5, 0.0, 10.0), ThroughAll)\"];", fork_dot)
        @test occursin("n1 -> n3 [label=\"Drill(Circle(0.5), (1.5, 0.0, 30.0), ThroughWall)\"];", fork_dot)
        @test count(line -> occursin("->", line), split(fork_dot, '\n')) == 3
        @test count(line -> occursin("stock RectangularTube", line), split(fork_dot, '\n')) == 1

        deep_dot = to_dot(graph(forked.c))
        @test occursin("Drill(Circle(0.5), (3.0, 1.5, 40.0), ThroughAll)", deep_dot)

        @test sprint(show, graph(parts.second)) == "ManufacturingGraph(4 nodes, 3 edges)"
        @test !occursin("AffineMap", sprint(show, graph(parts.second)))
        @test startswith(sprint(show, MIME("text/plain"), graph(one)), "digraph ManufacturingGraph {")
        @test occursin("diameter=1.0", sprint(show, hole_table(parts.second)[1]))
    end

    @testset "Constructor invariants" begin
        stick = RectangularTube(3.0, 3.0, 10.0, 0.1)
        blank = saw(stick, 5.0)
        edge = ManufacturingEdge(stick, blank.op, blank)

        @test_throws ArgumentError ManufacturingGraph([stick, stick], ManufacturingEdge[])
        @test_throws ArgumentError ManufacturingGraph([stick, blank], ManufacturingEdge[])
        @test_throws ArgumentError ManufacturingGraph([blank, stick], [edge])
        @test ManufacturingGraph([stick, blank], [edge]).nodes[2] === blank
    end
end

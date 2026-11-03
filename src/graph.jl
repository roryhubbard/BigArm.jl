"""
    ManufacturingEdge(parent, op, child)

Directed edge `parent ──op──► child` in a manufacturing graph. `op` is the shop
operation that produced `child` from `parent`.
"""
struct ManufacturingEdge
    parent::AbstractPart
    op::AbstractOperation
    child::AbstractPart
end

"""
    ManufacturingGraph(nodes, edges)

Unique part nodes and the directed operation edges that connect them, both in
topological order (every parent appears before its children).

Every node is a valid `AbstractPart`. Stock nodes have no incoming edge; every
other node has exactly one, matching its `parent` and `op`.
"""
struct ManufacturingGraph
    nodes::Vector{AbstractPart}
    edges::Vector{ManufacturingEdge}

    function ManufacturingGraph(
        nodes::AbstractVector{<:AbstractPart},
        edges::AbstractVector{<:ManufacturingEdge},
    )
        nodevec = AbstractPart[node for node in nodes]
        edgevec = ManufacturingEdge[edge for edge in edges]

        index = IdDict{AbstractPart,Int}()
        for (i, node) in enumerate(nodevec)
            haskey(index, node) && throw(ArgumentError("ManufacturingGraph nodes must be unique"))
            index[node] = i
        end

        incoming = IdDict{AbstractPart,Nothing}()
        for edge in edgevec
            haskey(index, edge.parent) || throw(ArgumentError("ManufacturingEdge parent is not in the graph"))
            haskey(index, edge.child) || throw(ArgumentError("ManufacturingEdge child is not in the graph"))
            index[edge.parent] < index[edge.child] || throw(
                ArgumentError("ManufacturingGraph edges must be in topological order (parent before child)"),
            )
            parent(edge.child) === edge.parent || throw(
                ArgumentError("ManufacturingEdge parent does not match parent(child)"),
            )
            operation(edge.child) === edge.op || throw(
                ArgumentError("ManufacturingEdge op does not match operation(child)"),
            )
            haskey(incoming, edge.child) && throw(
                ArgumentError("ManufacturingGraph node has more than one incoming edge"),
            )
            incoming[edge.child] = nothing
        end

        for node in nodevec
            if isstock(node)
                haskey(incoming, node) && throw(ArgumentError("stock node has an incoming edge"))
            else
                haskey(incoming, node) || throw(ArgumentError("non-stock node is missing the edge from its parent"))
            end
        end

        return new(nodevec, edgevec)
    end
end

"""
    graph(parts::AbstractPart...) -> ManufacturingGraph

Collect the manufacturing history of one or more parts.

Walks parent links to raw stock, deduplicates shared ancestors, and returns
the unique nodes and directed edges `parent ──op──► child` in topological
order. Parts are visited in argument order. Each part is emitted after its
unseen ancestors, so sibling order follows the order of `parts`.
"""
function graph(parts::AbstractPart...)
    nodes = AbstractPart[]
    edges = ManufacturingEdge[]
    seen = IdDict{AbstractPart,Nothing}()

    function visit(part::AbstractPart)
        haskey(seen, part) && return
        seen[part] = nothing
        upstream = parent(part)
        if !isnothing(upstream)
            visit(upstream)
        end
        push!(nodes, part)
        if !isnothing(upstream)
            produced_by = operation(part)
            produced_by isa AbstractOperation || throw(ArgumentError("non-stock $(typeof(part)) is missing an operation"))
            push!(edges, ManufacturingEdge(upstream, produced_by, part))
        end
        return nothing
    end

    for part in parts
        visit(part)
    end
    return ManufacturingGraph(nodes, edges)
end

"""
    operations(part::AbstractPart) -> Vector{AbstractOperation}

Return the shop operations that produced `part`, from raw stock to `part`,
read from `graph(part)`.
"""
function operations(part::AbstractPart)
    return AbstractOperation[edge.op for edge in graph(part).edges]
end

"""
    HoleTableRow

One drilled hole, read back from a part's manufacturing graph.

`index` is chronological, starting at 1. `local_x` and `local_y` are the hole
center in the entry-face datum (bottom-left origin, inches). `position` and
`normal` are the center and outward normal in the part frame.
"""
struct HoleTableRow
    index::Int
    diameter::Float64
    local_x::Float64
    local_y::Float64
    position::SVector{3,Float64}
    normal::SVector{3,Float64}
    penetration::AbstractPenetration
end

function _hole_row(part::RectangularTube, op::Drill, index::Int)
    _, local_x, local_y = _match_face(part, op.datum)
    return HoleTableRow(
        index,
        op.profile.diameter,
        Float64(local_x),
        Float64(local_y),
        SVector{3,Float64}(origin(op.datum)),
        SVector{3,Float64}(zaxis(op.datum)),
        op.penetration,
    )
end

"""
    hole_table(part::AbstractPart) -> Vector{HoleTableRow}

Inspection read of every `Drill` in the manufacturing graph of `part`, in
chronological order. Non-drill operations are omitted. Local coordinates are
measured on the part node that drill produced. Holes are reported even if a
later `saw` shortened the tube past them: the table is a read of the graph,
not a filter of the current solid.
"""
function hole_table(part::AbstractPart)
    rows = HoleTableRow[]
    index = 0
    for edge in graph(part).edges
        op = edge.op
        op isa Drill || continue
        index += 1
        push!(rows, _hole_row(edge.child, op, index))
    end
    return rows
end

# Six decimal places suppresses RotX(π/2) noise (~1e-16) in DOT labels without
# changing shop-inch coordinates. Signed zero is canonicalized so labels are stable.
function _fmt(x::Real)
    y = round(Float64(x); digits=6)
    return string(y == 0.0 ? 0.0 : y)
end

function _dot_escape(text::AbstractString)
    return replace(text, '\\' => "\\\\", '"' => "\\\"", '\n' => "\\n")
end

function _part_summary(part::RectangularTube)
    kind = isstock(part) ? "stock" : "part"
    dims = "$(_fmt(part.width)) × $(_fmt(part.height)) × $(_fmt(part.length))"
    return "$kind RectangularTube\n$dims\nt=$(_fmt(part.thickness))"
end

function _part_summary(part::AbstractPart)
    kind = isstock(part) ? "stock" : "part"
    return "$kind $(nameof(typeof(part)))"
end

_pen_label(::ThroughAll) = "ThroughAll"
_pen_label(::ThroughWall) = "ThroughWall"
_pen_label(penetration::Blind) = "Blind($(_fmt(penetration.depth)))"
_pen_label(penetration::AbstractPenetration) = string(nameof(typeof(penetration)))

_op_label(op::Saw) = "Saw($(_fmt(op.length)))"

function _op_label(op::Drill)
    p = origin(op.datum)
    position = "($(_fmt(p[1])), $(_fmt(p[2])), $(_fmt(p[3])))"
    return "Drill(Circle($(_fmt(op.profile.diameter))), $position, $(_pen_label(op.penetration)))"
end

_op_label(op::AbstractOperation) = string(nameof(typeof(op)))

"""
    to_dot(g::ManufacturingGraph) -> String

Serialize `g` as a Graphviz DOT digraph. Nodes are stock/part summaries and
edges are labeled with the operation that produced the child. Node ids `n0`,
`n1`, ... follow `g.nodes` order.
"""
function to_dot(g::ManufacturingGraph)
    lines = String["digraph ManufacturingGraph {", "  rankdir=TB;", "  node [shape=box];"]
    index = IdDict{AbstractPart,Int}()
    for (i, node) in enumerate(g.nodes)
        id = i - 1
        index[node] = id
        label = _dot_escape(_part_summary(node))
        push!(lines, "  n$id [label=\"$label\"];")
    end
    for edge in g.edges
        label = _dot_escape(_op_label(edge.op))
        push!(lines, "  n$(index[edge.parent]) -> n$(index[edge.child]) [label=\"$label\"];")
    end
    push!(lines, "}")
    return join(lines, '\n') * '\n'
end

function Base.show(io::IO, edge::ManufacturingEdge)
    print(io, "ManufacturingEdge(")
    print(io, nameof(typeof(edge.parent)), " ──", _op_label(edge.op), "──► ")
    print(io, nameof(typeof(edge.child)), ")")
    return nothing
end

function Base.show(io::IO, g::ManufacturingGraph)
    print(io, "ManufacturingGraph(", length(g.nodes), " nodes, ", length(g.edges), " edges)")
    return nothing
end

function Base.show(io::IO, ::MIME"text/plain", g::ManufacturingGraph)
    print(io, to_dot(g))
    return nothing
end

function Base.show(io::IO, row::HoleTableRow)
    print(io, "HoleTableRow(")
    print(io, row.index, ", diameter=", _fmt(row.diameter))
    print(io, ", local=(", _fmt(row.local_x), ", ", _fmt(row.local_y), "), ")
    print(io, nameof(typeof(row.penetration)), ")")
    return nothing
end

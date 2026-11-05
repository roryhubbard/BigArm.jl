"""
    ManufacturingGraph(part::AbstractPart)

Manufacturing history of `part`, from raw stock to `part`.

The history is a chain. Nodes are in that order, stock first. An edge is not
stored: it is `parent(node) ──operation(node)──► node`.
"""
struct ManufacturingGraph
    nodes::Vector{AbstractPart}

    function ManufacturingGraph(part::AbstractPart)
        nodes = AbstractPart[]
        curr = part
        while !isnothing(curr)
            push!(nodes, curr)
            curr = parent(curr)
        end
        reverse!(nodes)
        return new(nodes)
    end
end

"""
    operations(part::AbstractPart) -> Vector{AbstractOperation}

Return the shop operations that produced `part`, from raw stock to `part`,
read from `ManufacturingGraph(part)`.
"""
function operations(part::AbstractPart)
    return AbstractOperation[operation(node) for node in ManufacturingGraph(part).nodes if !isstock(node)]
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

Serialize `g` as a Graphviz DOT digraph. Nodes are stock/part summaries. Each
non-stock node emits an edge `parent(node) → node`, labeled with
`operation(node)`. Node ids `n0`, `n1`, ... follow `g.nodes` order.
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
    for node in g.nodes
        isstock(node) && continue
        label = _dot_escape(_op_label(operation(node)))
        push!(lines, "  n$(index[parent(node)]) -> n$(index[node]) [label=\"$label\"];")
    end
    push!(lines, "}")
    return join(lines, '\n') * '\n'
end

function Base.show(io::IO, g::ManufacturingGraph)
    print(io, "ManufacturingGraph(", length(g.nodes), " nodes)")
    return nothing
end

function Base.show(io::IO, ::MIME"text/plain", g::ManufacturingGraph)
    print(io, to_dot(g))
    return nothing
end

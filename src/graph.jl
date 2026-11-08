"""
    ManufacturingGraph(part::AbstractPart)

Directed acyclic manufacturing graph of `part`, ordered topologically from raw
stock roots to `part`.

Each non-stock `node` has incoming edge(s) from `parent(node)` labeled with
`operation(node)`.
"""
struct ManufacturingGraph
    nodes::Vector{AbstractPart}

    function ManufacturingGraph(part::AbstractPart)
        return new(_collect_nodes(part))
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

function _part_summary(part::RoundTube)
    kind = isstock(part) ? "stock" : "part"
    dims = "$(_fmt(part.outer_diameter)) × $(_fmt(part.length))"
    return "$kind RoundTube\n$dims\nt=$(_fmt(part.thickness))"
end

function _part_summary(part::Plate)
    kind = isstock(part) ? "stock" : "part"
    dims = "$(_fmt(part.width)) × $(_fmt(part.height))"
    return "$kind Plate\n$dims\nt=$(_fmt(part.thickness))"
end

function _part_summary(part::Weldment)
    return "part Weldment\n$(length(part.parts)) parts"
end

function _part_summary(part::AbstractPart)
    kind = isstock(part) ? "stock" : "part"
    return "$kind $(nameof(typeof(part)))"
end

_pen_label(::ThroughAll) = "ThroughAll"
_pen_label(::ThroughWall) = "ThroughWall"
_pen_label(penetration::Blind) = "Blind($(_fmt(penetration.depth)))"
_pen_label(penetration::AbstractPenetration) = string(nameof(typeof(penetration)))

function _op_label(op::Saw)
    p = origin(op.datum)
    n = zaxis(op.datum)
    position = "($(_fmt(p[1])), $(_fmt(p[2])), $(_fmt(p[3])))"
    normal = "($(_fmt(n[1])), $(_fmt(n[2])), $(_fmt(n[3])))"
    return "Saw($position, $normal)"
end

function _op_label(op::Drill)
    p = origin(op.datum)
    position = "($(_fmt(p[1])), $(_fmt(p[2])), $(_fmt(p[3])))"
    return "Drill(Circle($(_fmt(op.profile.diameter))), $position, $(_pen_label(op.penetration)))"
end

function _profile_label(profile::Polygon)
    pts = profile.points
    if length(pts) == 4
        w = pts[2][1] - pts[1][1]
        h = pts[3][2] - pts[2][2]
        if w > 0.0 &&
           h > 0.0 &&
           pts == Point2d[
               Point2d(-w / 2.0, -h / 2.0),
               Point2d( w / 2.0, -h / 2.0),
               Point2d( w / 2.0,  h / 2.0),
               Point2d(-w / 2.0,  h / 2.0),
           ]
            return "Rectangle($(_fmt(w)), $(_fmt(h)))"
        end
    end
    return "Polygon($(length(pts)))"
end

function _op_label(op::Cut)
    p = origin(op.datum)
    position = "($(_fmt(p[1])), $(_fmt(p[2])), $(_fmt(p[3])))"
    return "Cut($(_profile_label(op.profile)), $position)"
end

_op_label(op::AbstractOperation) = string(nameof(typeof(op)))

"""
    to_dot(g::ManufacturingGraph) -> String

Serialize `g` as a Graphviz DOT digraph. Nodes are stock/part summaries. Each
non-stock node emits an edge `p → node` for each parent `p`, labeled with
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
        ps = parent(node)
        if ps isa AbstractVector
            seen = IdSet{AbstractPart}()
            for p in ps
                p in seen && continue
                push!(seen, p)
                push!(lines, "  n$(index[p]) -> n$(index[node]) [label=\"$label\"];")
            end
        else
            push!(lines, "  n$(index[ps]) -> n$(index[node]) [label=\"$label\"];")
        end
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

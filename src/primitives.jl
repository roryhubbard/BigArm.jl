"""
    AbstractPart

Supertype for all nodes in a manufacturing graph.
"""
abstract type AbstractPart end

"""
    AbstractTube <: AbstractPart

Supertype for tubular parts.
"""
abstract type AbstractTube <: AbstractPart end

"""
    AbstractOperation

Supertype for shop manufacturing operations applied to parts.
"""
abstract type AbstractOperation end

"""
    RectangularTube(width, height, length, wall, parent=nothing, op=nothing)

Rectangular-tube part node with outer cross-section `width` (X) and `height` (Y),
tube `length` (Z), and `wall` thickness, all in inches (`Float64`).

Raw mill stock is a root `RectangularTube` whose `parent` and `op` are `nothing`.
Enforces finite positive dimensions and `2 * wall < min(width, height)`.
"""
struct RectangularTube <: AbstractTube
    width::Float64
    height::Float64
    length::Float64
    wall::Float64
    parent::Union{Nothing,AbstractPart}
    op::Union{Nothing,AbstractOperation}

    function RectangularTube(
        width::Real,
        height::Real,
        length::Real,
        wall::Real,
        parent::Union{Nothing,AbstractPart}=nothing,
        op::Union{Nothing,AbstractOperation}=nothing,
    )
        w = Float64(width)
        h = Float64(height)
        l = Float64(length)
        t = Float64(wall)
        (isfinite(w) && w > 0.0) || throw(DomainError(width, "RectangularTube width must be finite and positive"))
        (isfinite(h) && h > 0.0) || throw(DomainError(height, "RectangularTube height must be finite and positive"))
        (isfinite(l) && l > 0.0) || throw(DomainError(length, "RectangularTube length must be finite and positive"))
        (isfinite(t) && t > 0.0) || throw(DomainError(wall, "RectangularTube wall thickness must be finite and positive"))
        2.0 * t < min(w, h) || throw(
            ArgumentError("RectangularTube wall thickness ($t) must be strictly less than half of width ($w) and height ($h)"),
        )
        isnothing(parent) == isnothing(op) || throw(
            ArgumentError("RectangularTube parent and op must either both be nothing (raw stock) or both be present"),
        )
        return new(w, h, l, t, parent, op)
    end
end

"""
    isstock(part::AbstractPart) -> Bool

Return `true` if `part` is raw mill stock (a root node with `parent === nothing`).
"""
isstock(p::AbstractPart) = isnothing(p.parent)

"""
    Hole(diameter, z)

Cross-drilled circular through-hole specification with `diameter` and center
position `z` along the tube length (inches, `Float64`), drilled along the Y axis
through the front (`+Y`) and back (`-Y`) walls.
"""
struct Hole
    diameter::Float64
    z::Float64

    function Hole(diameter::Real, z::Real)
        d = Float64(diameter)
        zf = Float64(z)
        (isfinite(d) && d > 0.0) || throw(DomainError(diameter, "Hole diameter must be finite and positive"))
        (isfinite(zf) && zf > 0.0) || throw(DomainError(z, "Hole z position must be finite and positive"))
        return new(d, zf)
    end
end

"""
    Saw(length)

Cutoff operation producing a part of target `length` (inches, `Float64`).
"""
struct Saw <: AbstractOperation
    length::Float64

    function Saw(length::Real)
        l = Float64(length)
        (isfinite(l) && l > 0.0) || throw(DomainError(length, "Saw target length must be finite and positive"))
        return new(l)
    end
end

"""
    Drill(hole::Hole)

Cross-drilling operation applying `hole` to a part.
"""
struct Drill <: AbstractOperation
    hole::Hole
end

Drill(diameter::Real, z::Real) = Drill(Hole(diameter, z))

"""
    holes(part::AbstractPart) -> Vector{Hole}

Collect all `Hole`s drilled in the ancestor chain of `part` in chronological
order (from root stock to `part`).
"""
function holes(part::AbstractPart)
    result = Hole[]
    curr = part
    while !isnothing(curr)
        op = curr.op
        if op isa Drill
            push!(result, op.hole)
        end
        curr = curr.parent
    end
    reverse!(result)
    return result
end

_sawn_tube(part::RectangularTube, target::Float64, op::Saw) =
    RectangularTube(part.width, part.height, target, part.wall, part, op)

"""
    saw(part::AbstractTube, length::Real)

Cut a tubular `part` to target `length`. Validates that `length` is finite,
positive, at most the current tube length, and does not cut across or remove
any existing drilled hole.
"""
function saw(part::AbstractTube, op::Saw)
    target = op.length
    target <= part.length || throw(
        ArgumentError("Cannot saw tube of length $(part.length) to longer target length $target"),
    )
    for h in holes(part)
        r = h.diameter / 2.0
        if target <= h.z + r
            throw(
                ArgumentError(
                    "Cannot saw at length $target across or below existing hole at z=$(h.z) (diameter=$(h.diameter))",
                ),
            )
        end
    end
    return _sawn_tube(part, target, op)
end

saw(part::AbstractTube, length::Real) = saw(part, Saw(length))

"""
    drill(part::RectangularTube, hole::Hole)

Cross-drill `hole` through the front and back walls of `part`.
Validates that the hole fits within the inner wall width
(`diameter < width - 2 * wall`), lies wholly within the tube ends
(`diameter / 2 < z < length - diameter / 2`), and does not overlap any existing
hole along the tube.
"""
function drill(part::RectangularTube, op::Drill)
    h = op.hole
    inner_width = part.width - 2.0 * part.wall
    h.diameter < inner_width || throw(
        ArgumentError(
            "Hole diameter ($(h.diameter)) must be strictly less than inner tube width ($inner_width)",
        ),
    )
    r = h.diameter / 2.0
    (r < h.z && h.z < part.length - r) || throw(
        ArgumentError(
            "Hole (diameter=$(h.diameter), z=$(h.z)) must lie wholly within tube ends (0.0, $(part.length))",
        ),
    )
    for existing in holes(part)
        min_center_dist = (h.diameter + existing.diameter) / 2.0
        abs(h.z - existing.z) > min_center_dist || throw(
            ArgumentError(
                "Hole at z=$(h.z) (diameter=$(h.diameter)) overlaps existing hole at z=$(existing.z) (diameter=$(existing.diameter))",
            ),
        )
    end
    return RectangularTube(part.width, part.height, part.length, part.wall, part, op)
end

drill(part::RectangularTube, hole::Hole) = drill(part, Drill(hole))

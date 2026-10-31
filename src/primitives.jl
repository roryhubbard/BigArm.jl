"""
    AbstractPart

Supertype for the workpiece in the machine (nodes in a manufacturing graph).
"""
abstract type AbstractPart end

"""
    AbstractTube <: AbstractPart

Supertype for tubular parts.
"""
abstract type AbstractTube <: AbstractPart end

"""
    AbstractProfile

Supertype for pure 2D shapes cut in a datum's XY plane.
"""
abstract type AbstractProfile end

"""
    AbstractDatum

Supertype for coordinate frames on a part.
"""
abstract type AbstractDatum end

"""
    AbstractPenetration

Supertype for discrete cut penetration / termination modes.
"""
abstract type AbstractPenetration end

"""
    AbstractOperation

Supertype for shop actions applied to parts.
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
    Circle(diameter)

Circular 2D cut profile with finite positive `diameter` (inches, `Float64`).
"""
struct Circle <: AbstractProfile
    diameter::Float64

    function Circle(diameter::Real)
        d = Float64(diameter)
        (isfinite(d) && d > 0.0) || throw(DomainError(diameter, "Circle diameter must be finite and positive"))
        return new(d)
    end
end

const _ATOL = 1e-9

"""
    Datum(transform::Transformation)

Right-handed 3D coordinate frame (`SE(3)` rigid transformation relative to the
part's 3D origin), constructed from a `CoordinateTransformations` object
(`AffineMap`, `LinearMap`, or `Translation`):
- `origin(d)` is the 3D origin `SVector{3,Float64}` in part coordinates,
- `x_axis(d)` and `y_axis(d)` are the orthonormal 2D axes on the datum surface,
- `z_axis(d)` is the outward surface normal (tools approach along `-z_axis(d)`).
"""
struct Datum <: AbstractDatum
    transform::AffineMap{RotMatrix3{Float64},SVector{3,Float64}}

    function Datum(transform::AffineMap)
        M = transform.linear
        p = SVector{3,Float64}(transform.translation)
        all(isfinite, p) || throw(DomainError(p, "Datum origin coordinates must be finite"))
        all(isfinite, M) || throw(DomainError(M, "Datum rotation matrix entries must be finite"))
        isrotation(M) || throw(ArgumentError("Datum linear map must be a valid right-handed SO(3) rotation"))
        return new(AffineMap(RotMatrix3{Float64}(M), p))
    end
end

Datum(t::LinearMap) = Datum(Translation(0.0, 0.0, 0.0) ∘ t)
Datum(t::Translation) = Datum(t ∘ LinearMap(one(RotMatrix3{Float64})))

origin(d::Datum) = d.transform.translation
x_axis(d::Datum) = d.transform.linear[:, 1]
y_axis(d::Datum) = d.transform.linear[:, 2]
z_axis(d::Datum) = d.transform.linear[:, 3]

(d::Datum)(p::SVector{3,Float64}) = d.transform(p)
(d::Datum)(x::Real, y::Real, z::Real=0.0) = d.transform(SVector{3,Float64}(x, y, z))

"""
    translate(d::Datum, x::Real, y::Real, z::Real=0.0) -> Datum

Return a new `Datum` composed with a local `Translation(x, y, z)` in `d`'s frame
(`d.transform ∘ Translation(x, y, z)`).
"""
function translate(d::Datum, x::Real, y::Real, z::Real=0.0)
    v = SVector{3,Float64}(x, y, z)
    all(isfinite, v) || throw(DomainError((x, y, z), "Datum translation coordinates must be finite"))
    return Datum(d.transform ∘ Translation(v))
end

"""
    local_coords(frame::Datum, point_or_datum) -> SVector{3,Float64}

Return the local `(x, y, z)` coordinates of a 3D point or another `Datum`'s
origin in `frame` (`inv(frame.transform)(p)`).
"""
local_coords(frame::Datum, p::SVector{3,Float64}) = inv(frame.transform)(p)
local_coords(frame::Datum, target::Datum) = local_coords(frame, origin(target))

"""
    datums(part::RectangularTube) -> NTuple{4,Datum}

Return the four outer wall `Datum` frames of a `RectangularTube` (front `-Y`,
right `+X`, back `+Y`, left `-X`), each with `(0, 0)` at its bottom-left corner
when looking straight at the wall (`-z_axis`) with `+Z` pointing up.
"""
function datums(part::RectangularTube)
    w = part.width
    h = part.height
    return (
        Datum(Translation(0.0, 0.0, 0.0) ∘ LinearMap(RotX(π / 2))),                  # front (-Y)
        Datum(Translation(w,   0.0, 0.0) ∘ LinearMap(RotZ(π / 2) * RotX(π / 2))),    # right (+X)
        Datum(Translation(w,   h,   0.0) ∘ LinearMap(RotZ(π) * RotX(π / 2))),        # back  (+Y)
        Datum(Translation(0.0, h,   0.0) ∘ LinearMap(RotZ(-π / 2) * RotX(π / 2))),   # left  (-X)
    )
end

"""
    datum(part::AbstractPart, normal) -> Datum

Select the `Datum` in `datums(part)` whose outward normal `z_axis(d)` matches
`normal`.
"""
function datum(part::AbstractPart, normal::Union{AbstractVector{<:Real},Tuple{Real,Real,Real}})
    n = SVector{3,Float64}(normal)
    for d in datums(part)
        if norm(z_axis(d) - n) <= _ATOL
            return d
        end
    end
    throw(ArgumentError("Part $(typeof(part)) has no datum with normal $n"))
end

front(part::AbstractPart)  = datum(part, SVector( 0.0, -1.0,  0.0))
back(part::AbstractPart)   = datum(part, SVector( 0.0,  1.0,  0.0))
right(part::AbstractPart)  = datum(part, SVector( 1.0,  0.0,  0.0))
left(part::AbstractPart)   = datum(part, SVector(-1.0,  0.0,  0.0))
top(part::AbstractPart)    = datum(part, SVector( 0.0,  0.0,  1.0))
bottom(part::AbstractPart) = datum(part, SVector( 0.0,  0.0, -1.0))

front(part::AbstractPart, x::Real, y::Real)  = translate(front(part), x, y)
back(part::AbstractPart, x::Real, y::Real)   = translate(back(part), x, y)
right(part::AbstractPart, x::Real, y::Real)  = translate(right(part), x, y)
left(part::AbstractPart, x::Real, y::Real)   = translate(left(part), x, y)
top(part::AbstractPart, x::Real, y::Real)    = translate(top(part), x, y)
bottom(part::AbstractPart, x::Real, y::Real) = translate(bottom(part), x, y)

"""
    ThroughAll()

Singleton penetration mode: cut through the entire cross-section (both datum
and opposite walls).
"""
struct ThroughAll <: AbstractPenetration end

"""
    ThroughWall()

Singleton penetration mode: cut through the datum wall into the cavity only.
"""
struct ThroughWall <: AbstractPenetration end

"""
    Blind(depth)

Blind-hole penetration of `depth` (inches, `Float64`) for solid stock. Not valid
on thin-wall tubes.
"""
struct Blind <: AbstractPenetration
    depth::Float64

    function Blind(depth::Real)
        d = Float64(depth)
        (isfinite(d) && d > 0.0) || throw(DomainError(depth, "Blind depth must be finite and positive"))
        return new(d)
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
    Drill(profile::Circle, datum::Datum, penetration::AbstractPenetration)

Drilling operation cutting `profile` at `datum` with `penetration` mode.
"""
struct Drill <: AbstractOperation
    profile::Circle
    datum::Datum
    penetration::AbstractPenetration
end

"""
    isstock(part::AbstractPart) -> Bool

Return `true` if `part` is raw mill stock (a root node with `parent === nothing`).
"""
isstock(p::AbstractPart) = isnothing(p.parent)

"""
    holes(part::AbstractPart) -> Vector{Drill}

Collect all `Drill` operations in the ancestor chain of `part` in chronological
order (from root stock to `part`).
"""
function holes(part::AbstractPart)
    result = Drill[]
    curr = part
    while !isnothing(curr)
        op = curr.op
        if op isa Drill
            push!(result, op)
        end
        curr = curr.parent
    end
    reverse!(result)
    return result
end

function _wall_span(part::RectangularTube, wall::Datum)
    dims = SVector(part.width, part.height, part.length)
    wall_width = dot(abs.(x_axis(wall)), dims)
    cross_depth = dot(abs.(z_axis(wall)), dims)
    return (wall_width, cross_depth)
end

function _match_wall(part::RectangularTube, d::Datum)
    wall = datum(part, z_axis(d))
    norm(d.transform.linear - wall.transform.linear) <= _ATOL ||
        throw(ArgumentError("Datum axes must align with the target wall datum on RectangularTube"))
    x, y, z = local_coords(wall, d)
    abs(z) <= _ATOL ||
        throw(ArgumentError("Datum origin $(origin(d)) does not lie on the surface of wall with normal $(z_axis(wall)) (normal offset z=$z)"))
    return (wall, x, y)
end

"""
    pierced_walls(part::RectangularTube, d::Datum, penetration::AbstractPenetration)
    pierced_walls(part::RectangularTube, op::Drill)

Return the tuple of `(wall::Datum, center_3d::SVector{3,Float64})` pairs for
each wall of `part` pierced by a drilling operation.
"""
function pierced_walls(part::RectangularTube, d::Datum, ::ThroughWall)
    wall, _, _ = _match_wall(part, d)
    return ((wall, origin(d)),)
end

function pierced_walls(part::RectangularTube, d::Datum, ::ThroughAll)
    wall, _, _ = _match_wall(part, d)
    _, cross_depth = _wall_span(part, wall)
    opp_wall = datum(part, -z_axis(wall))
    exit_center = d(0.0, 0.0, -cross_depth)
    return ((wall, origin(d)), (opp_wall, exit_center))
end

pierced_walls(part::RectangularTube, op::Drill) =
    pierced_walls(part, op.datum, op.penetration)

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
    for drill_op in holes(part)
        r = drill_op.profile.diameter / 2.0
        hole_z = origin(drill_op.datum)[3]
        if target <= hole_z + r
            throw(
                ArgumentError(
                    "Cannot saw at length $target across or below existing hole at Z=$hole_z (diameter=$(drill_op.profile.diameter))",
                ),
            )
        end
    end
    return RectangularTube(part.width, part.height, target, part.wall, part, op)
end

saw(part::AbstractTube, length::Real) = saw(part, Saw(length))

"""
    drill(
        part::RectangularTube,
        profile::Circle,
        d::Datum,
        penetration::Union{ThroughAll,ThroughWall}=ThroughAll(),
    )

Drill circular `profile` into `part` at datum `d`. Validates that `d` lies on a
valid wall of `part`, the circle fits strictly within the inner cavity width of
that wall (`wall + diameter / 2 < x < wall_width - wall - diameter / 2`), lies
wholly within the tube ends (`diameter / 2 < y < length - diameter / 2`), and
does not overlap any existing hole on any wall it pierces.
"""
function drill(
    part::RectangularTube,
    profile::Circle,
    d::Datum,
    penetration::Union{ThroughAll,ThroughWall},
)
    wall, x, y = _match_wall(part, d)
    wall_width, _ = _wall_span(part, wall)
    r = profile.diameter / 2.0
    (part.wall + r < x && x < wall_width - part.wall - r) || throw(
        ArgumentError(
            "Circle (diameter=$(profile.diameter)) at local x=$x must fit strictly within inner cavity x ∈ ($(part.wall), $(wall_width - part.wall))",
        ),
    )
    (r < y && y < part.length - r) || throw(
        ArgumentError(
            "Hole (diameter=$(profile.diameter), y=$y) must lie wholly within tube ends (0.0, $(part.length))",
        ),
    )

    op = Drill(profile, d, penetration)
    new_pierced = pierced_walls(part, op)
    for existing in holes(part)
        existing_pierced = pierced_walls(part, existing)
        for (w1, c1) in new_pierced
            for (w2, c2) in existing_pierced
                if norm(z_axis(w1) - z_axis(w2)) <= _ATOL
                    min_center_dist = r + existing.profile.diameter / 2.0
                    if norm(c1 - c2) <= min_center_dist
                        throw(
                            ArgumentError(
                                "Hole at 3D center $c1 overlaps existing hole at $c2 on wall with normal $(z_axis(w1))",
                            ),
                        )
                    end
                end
            end
        end
    end
    return RectangularTube(part.width, part.height, part.length, part.wall, part, op)
end

drill(part::RectangularTube, profile::Circle, d::Datum) =
    drill(part, profile, d, ThroughAll())

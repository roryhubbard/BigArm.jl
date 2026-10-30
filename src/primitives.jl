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

Supertype for pure 2D shapes cut in a datum face's plane.
"""
abstract type AbstractProfile end

"""
    AbstractDatum

Supertype for locations on a part (face/axis and coordinates).
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

const VALID_FACES = (:front, :back, :left, :right)

"""
    FaceDatum(face, x=0.0, z)

Location on a part face with valid `face::Symbol`
(`:front`, `:back`, `:left`, `:right`), horizontal offset `x` from the face
centerline, and axial position `z` from the base (`Z = 0`), in inches
(`Float64`).
"""
struct FaceDatum <: AbstractDatum
    face::Symbol
    x::Float64
    z::Float64

    function FaceDatum(face::Symbol, x::Real, z::Real)
        face in VALID_FACES || throw(
            ArgumentError("Invalid face :$face; must be one of :front, :back, :left, :right"),
        )
        xf = Float64(x)
        zf = Float64(z)
        isfinite(xf) || throw(DomainError(x, "FaceDatum x must be finite"))
        isfinite(zf) || throw(DomainError(z, "FaceDatum z must be finite"))
        return new(face, xf, zf)
    end
end

FaceDatum(face::Symbol, z::Real) = FaceDatum(face, 0.0, z)

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
    Drill(profile::Circle, datum::FaceDatum, penetration::AbstractPenetration)

Drilling operation cutting `profile` at `datum` with `penetration` mode.
"""
struct Drill <: AbstractOperation
    profile::Circle
    datum::FaceDatum
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

_face_width(part::RectangularTube, face::Symbol) =
    (face === :front || face === :back) ? part.width : part.height

_opposite_face(face::Symbol) =
    face === :front ? :back :
    face === :back ? :front :
    face === :left ? :right : :left

"""
    pierced_faces(op::Drill) -> NTuple{2,Symbol}

Faces pierced by drilling operation `op`: only `op.datum.face` for `ThroughWall`
(with a `nothing` second entry), or both `op.datum.face` and its opposite wall
for `ThroughAll`.
"""
pierced_faces(op::Drill) =
    op.penetration isa ThroughWall ? (op.datum.face, nothing) :
    (op.datum.face, _opposite_face(op.datum.face))

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
        if target <= drill_op.datum.z + r
            throw(
                ArgumentError(
                    "Cannot saw at length $target across or below existing hole at z=$(drill_op.datum.z) (diameter=$(drill_op.profile.diameter))",
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
        datum::FaceDatum,
        penetration::Union{ThroughAll,ThroughWall}=ThroughAll(),
    )

Drill circular `profile` into `part` at `datum`. Validates that the circle fits
strictly within the inner cavity width of that face
(`abs(datum.x) + diameter / 2 < face_width / 2 - wall`), lies wholly within the
tube ends (`diameter / 2 < datum.z < length - diameter / 2`), and does not
overlap any existing hole on any wall it pierces.
"""
function drill(
    part::RectangularTube,
    profile::Circle,
    datum::FaceDatum,
    penetration::Union{ThroughAll,ThroughWall},
)
    r = profile.diameter / 2.0
    face_width = _face_width(part, datum.face)
    inner_width = face_width - 2.0 * part.wall
    abs(datum.x) + r < inner_width / 2.0 || throw(
        ArgumentError(
            "Circle (diameter=$(profile.diameter)) at x=$(datum.x) on face :$(datum.face) must fit strictly within inner cavity width ($inner_width)",
        ),
    )
    (r < datum.z && datum.z < part.length - r) || throw(
        ArgumentError(
            "Hole (diameter=$(profile.diameter), z=$(datum.z)) must lie wholly within tube ends (0.0, $(part.length))",
        ),
    )

    op = Drill(profile, datum, penetration)
    new_faces = pierced_faces(op)
    for existing in holes(part)
        existing_faces = pierced_faces(existing)
        same_face = datum.face == existing.datum.face
        if same_face || new_faces[2] in existing_faces || existing_faces[2] in new_faces
            min_center_dist = r + existing.profile.diameter / 2.0
            intersects = if same_face
                # Circles on the same planar wall must not intersect in 2D.
                hypot(datum.x - existing.datum.x, datum.z - existing.datum.z) <= min_center_dist
            else
                # ThroughWall on one wall and ThroughWall on the opposite wall
                # pierce different wall material (into the same open cavity).
                penetration isa ThroughWall && existing.penetration isa ThroughWall ?
                false : abs(datum.z - existing.datum.z) <= min_center_dist
            end
            intersects && throw(
                ArgumentError(
                    "Hole at (face=:$(datum.face), x=$(datum.x), z=$(datum.z)) overlaps existing hole at (face=:$(existing.datum.face), x=$(existing.datum.x), z=$(existing.datum.z))",
                ),
            )
        end
    end
    return RectangularTube(part.width, part.height, part.length, part.wall, part, op)
end

drill(part::RectangularTube, profile::Circle, datum::FaceDatum) =
    drill(part, profile, datum, ThroughAll())

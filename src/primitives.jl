"""
    AbstractPart

Supertype for the workpiece in the machine (nodes in a manufacturing graph).

Concrete parts must implement `parent(part)` and `operation(part)`. Both are
`nothing` for raw stock, and both are present when a shop operation produced
the part. `parent` extends `Base.parent`; `operation` has no fallback.
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
    RectangularTube(width, height, length, thickness, parent=nothing, op=nothing)

Rectangular-tube part node with outer cross-section `width` (X) and `height` (Y),
tube `length` (Z), and wall `thickness`, all in inches (`Float64`).

Raw mill stock is a root `RectangularTube` whose `parent` and `op` are `nothing`.
Enforces positive dimensions and `2 * thickness < min(width, height)`.
"""
struct RectangularTube <: AbstractTube
    width::Float64
    height::Float64
    length::Float64
    thickness::Float64
    parent::Union{Nothing,AbstractPart}
    op::Union{Nothing,AbstractOperation}

    function RectangularTube(
        width::Real,
        height::Real,
        length::Real,
        thickness::Real,
        parent::Union{Nothing,AbstractPart}=nothing,
        op::Union{Nothing,AbstractOperation}=nothing,
    )
        w = Float64(width)
        h = Float64(height)
        l = Float64(length)
        t = Float64(thickness)
        w > 0.0 || throw(DomainError(width, "RectangularTube width must be positive"))
        h > 0.0 || throw(DomainError(height, "RectangularTube height must be positive"))
        l > 0.0 || throw(DomainError(length, "RectangularTube length must be positive"))
        t > 0.0 || throw(DomainError(thickness, "RectangularTube thickness must be positive"))
        2.0 * t < min(w, h) || throw(
            ArgumentError("RectangularTube thickness ($t) must be strictly less than half of width ($w) and height ($h)"),
        )
        isnothing(parent) == isnothing(op) || throw(
            ArgumentError("RectangularTube parent and op must either both be nothing (raw stock) or both be present"),
        )
        return new(w, h, l, t, parent, op)
    end
end

"""
    Plate(width, height, thickness, parent=nothing, op=nothing)

Flat rectangular plate part node with dimensions `width` (X), `height` (Y), and
`thickness` (Z), all in inches (`Float64`).

Raw plate stock is a root `Plate` whose `parent` and `op` are `nothing`.
Enforces positive dimensions.
"""
struct Plate <: AbstractPart
    width::Float64
    height::Float64
    thickness::Float64
    parent::Union{Nothing,AbstractPart}
    op::Union{Nothing,AbstractOperation}

    function Plate(
        width::Real,
        height::Real,
        thickness::Real,
        parent::Union{Nothing,AbstractPart}=nothing,
        op::Union{Nothing,AbstractOperation}=nothing,
    )
        w = Float64(width)
        h = Float64(height)
        t = Float64(thickness)
        w > 0.0 || throw(DomainError(width, "Plate width must be positive"))
        h > 0.0 || throw(DomainError(height, "Plate height must be positive"))
        t > 0.0 || throw(DomainError(thickness, "Plate thickness must be positive"))
        isnothing(parent) == isnothing(op) || throw(
            ArgumentError("Plate parent and op must either both be nothing (raw stock) or both be present"),
        )
        return new(w, h, t, parent, op)
    end
end

"""
    Circle(diameter)

Circular 2D cut profile with positive `diameter` (inches, `Float64`).
"""
struct Circle <: AbstractProfile
    diameter::Float64

    function Circle(diameter::Real)
        d = Float64(diameter)
        d > 0.0 || throw(DomainError(diameter, "Circle diameter must be positive"))
        return new(d)
    end
end

const _ATOL = 1e-9

const RigidTransform = AffineMap{<:Rotation{3},<:SVector{3}}

"""
    Datum(transform::RigidTransform)

Right-handed 3D coordinate frame (`SE(3)` rigid transformation relative to the
part's 3D origin):
- `origin(datum)` is the 3D origin in part coordinates,
- `xaxis(datum)` and `yaxis(datum)` are the orthonormal 2D axes on the datum surface,
- `zaxis(datum)` is the outward surface normal (tools approach along `-zaxis(datum)`).
"""
struct Datum <: AbstractDatum
    transform::RigidTransform
end

origin(datum::Datum) = datum.transform.translation
xaxis(datum::Datum) = datum.transform.linear[:, 1]
yaxis(datum::Datum) = datum.transform.linear[:, 2]
zaxis(datum::Datum) = datum.transform.linear[:, 3]

"""
    datums(part::RectangularTube) -> NTuple{4,Datum}

Return the four outer face `Datum` frames of a `RectangularTube` (front `-Y`,
right `+X`, back `+Y`, left `-X`), each with `(0, 0)` at its bottom-left corner
when looking straight at the face (`-zaxis`) with `+Z` pointing up.
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
    datums(part::Plate) -> NTuple{2,Datum}

Return the two flat surface `Datum` frames of a `Plate` (top `+Z` at
`z = thickness` and bottom `-Z` at `z = 0`), each with `(0, 0)` at its
bottom-left corner when looking straight at the face (`-zaxis`) with `+Y`
pointing up.
"""
function datums(part::Plate)
    w = part.width
    t = part.thickness
    return (
        Datum(Translation(0.0, 0.0, t)   ∘ LinearMap(RotX(0.0))),                  # top    (+Z)
        Datum(Translation(w,   0.0, 0.0) ∘ LinearMap(RotZ(π) * RotX(π))),          # bottom (-Z)
    )
end

"""
    datum(part::AbstractPart, normal) -> Datum

Select the `Datum` in `datums(part)` whose outward normal `zaxis(datum)` matches
`normal`.
"""
function datum(part::AbstractPart, normal::SVector{3,<:Real})
    n = normalize(normal)
    for datum in datums(part)
        if dot(zaxis(datum), n) > 1.0 - _ATOL
            return datum
        end
    end
    throw(ArgumentError("Part $(typeof(part)) has no datum with normal $normal"))
end

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
        d > 0.0 || throw(DomainError(depth, "Blind depth must be positive"))
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
        l > 0.0 || throw(DomainError(length, "Saw target length must be positive"))
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
    parent(part) -> Union{Nothing,AbstractPart}

Return the part `part` was made from, or `nothing` for raw stock.

Each concrete `AbstractPart` implements this method of `Base.parent`. It is
the single input of operations such as `saw` and `drill`.
"""
parent(part::RectangularTube) = part.parent
parent(part::Plate) = part.parent

"""
    operation(part) -> Union{Nothing,AbstractOperation}

Return the shop operation that produced `part` from `parent(part)`, or
`nothing` for raw stock.

Each concrete `AbstractPart` implements this. `parent(part)` and
`operation(part)` are either both `nothing` or both present.
"""
operation(part::RectangularTube) = part.op
operation(part::Plate) = part.op

"""
    isstock(part::AbstractPart) -> Bool

Return `true` if `part` is raw mill stock (`parent(part) === nothing`).
"""
isstock(part::AbstractPart) = isnothing(parent(part))

"""
    holes(part::AbstractPart) -> Vector{Drill}

Collect all `Drill` operations in the ancestor chain of `part` in chronological
order (from root stock to `part`).
"""
function holes(part::AbstractPart)
    result = Drill[]
    curr = part
    while !isnothing(curr)
        produced_by = operation(curr)
        if produced_by isa Drill
            push!(result, produced_by)
        end
        curr = parent(curr)
    end
    reverse!(result)
    return result
end

"""
    _face_width(part::RectangularTube, face::Datum) -> Float64

Return the outer width of `face` on `part` along `xaxis(face)`.
"""
function _face_width(part::RectangularTube, face::Datum)
    dims = SVector(part.width, part.height, part.length)
    return abs(dot(xaxis(face), dims))
end

"""
    _match_face(part::AbstractPart, datum::Datum) -> Tuple{Datum,Float64,Float64}

Find the outer `face` `Datum` on `part` whose normal matches `zaxis(datum)`,
verify that `origin(datum)` lies on `face`'s surface (`z == 0`), and return
`(face, x, y)` in `face`'s local 2D coordinates.
"""
function _match_face(part::AbstractPart, datum::Datum)
    face = BigArm.datum(part, zaxis(datum))
    T_face_datum = inv(face.transform) ∘ datum.transform
    x, y, z = T_face_datum.translation
    abs(z) <= _ATOL ||
        throw(ArgumentError("Datum origin $(origin(datum)) does not lie on the surface of face with normal $(zaxis(face)) (normal offset z=$z)"))
    return (face, x, y)
end

"""
    saw(part::AbstractTube, length::Real)

Cut a tubular `part` to target `length`. Validates that `length` is positive
and at most the current tube length.
"""
function saw(part::AbstractTube, op::Saw)
    target = op.length
    target <= part.length || throw(
        ArgumentError("Cannot saw tube of length $(part.length) to longer target length $target"),
    )
    return RectangularTube(part.width, part.height, target, part.thickness, part, op)
end

saw(part::AbstractTube, length::Real) = saw(part, Saw(length))

"""
    drill(
        part::RectangularTube,
        profile::Circle,
        datum::Datum,
        penetration::Union{ThroughAll,ThroughWall},
    )

Drill circular `profile` into `part` at `datum` with `penetration`. Validates
that `datum` lies on a valid face of `part`, the circle fits strictly within
the inner cavity width of that face
(`thickness + diameter / 2 < x < face_width - thickness - diameter / 2`), and
lies wholly within the tube ends (`diameter / 2 < y < length - diameter / 2`).
"""
function drill(
    part::RectangularTube,
    profile::Circle,
    datum::Datum,
    penetration::Union{ThroughAll,ThroughWall},
)
    face, x, y = _match_face(part, datum)
    face_width = _face_width(part, face)
    r = profile.diameter / 2.0
    (part.thickness + r < x && x < face_width - part.thickness - r) || throw(
        ArgumentError(
            "Circle (diameter=$(profile.diameter)) at local x=$x must fit strictly within inner cavity x ∈ ($(part.thickness), $(face_width - part.thickness))",
        ),
    )
    (r < y && y < part.length - r) || throw(
        ArgumentError(
            "Hole (diameter=$(profile.diameter), y=$y) must lie wholly within tube ends (0.0, $(part.length))",
        ),
    )

    op = Drill(profile, datum, penetration)
    return RectangularTube(part.width, part.height, part.length, part.thickness, part, op)
end

"""
    drill(
        part::Plate,
        profile::Circle,
        datum::Datum,
        penetration::AbstractPenetration,
    )

Drill circular `profile` into `part` at `datum` with `penetration`. Validates
that `datum` lies on a valid flat surface of `part`, that the circle fits
strictly within the plate perimeter (`diameter / 2 < x < width - diameter / 2`
and `diameter / 2 < y < height - diameter / 2`), and for `Blind` holes that
`penetration.depth < part.thickness`.
"""
function drill(
    part::Plate,
    profile::Circle,
    datum::Datum,
    penetration::AbstractPenetration,
)
    _, x, y = _match_face(part, datum)
    r = profile.diameter / 2.0
    (r < x && x < part.width - r) || throw(
        ArgumentError(
            "Hole (diameter=$(profile.diameter), x=$x) must lie wholly within plate width (0.0, $(part.width))",
        ),
    )
    (r < y && y < part.height - r) || throw(
        ArgumentError(
            "Hole (diameter=$(profile.diameter), y=$y) must lie wholly within plate height (0.0, $(part.height))",
        ),
    )
    if penetration isa Blind
        penetration.depth < part.thickness || throw(
            ArgumentError(
                "Blind hole depth ($(penetration.depth)) must be strictly less than plate thickness ($(part.thickness))",
            ),
        )
    end

    op = Drill(profile, datum, penetration)
    return Plate(part.width, part.height, part.thickness, part, op)
end

"""
    Assembly(parts)

Multi-part 3D assembly positioning each `AbstractPart` in a shared right-handed
3D coordinate frame via a `Pair` `part => transform`, where `transform` is a
`RigidTransform` (`AffineMap{<:Rotation{3},<:SVector{3}}`).
"""
struct Assembly
    parts::Vector{Pair{<:AbstractPart,<:RigidTransform}}

    function Assembly(
        parts::AbstractVector{<:Pair{<:AbstractPart,<:RigidTransform}},
    )
        isempty(parts) && throw(ArgumentError("Assembly must contain at least one part"))
        return new(collect(Pair{<:AbstractPart,<:RigidTransform}, parts))
    end
end

Base.length(assembly::Assembly) = length(assembly.parts)
Base.iterate(assembly::Assembly, state...) = iterate(assembly.parts, state...)
Base.getindex(assembly::Assembly, i::Integer) = assembly.parts[i]

function Base.show(io::IO, assembly::Assembly)
    print(io, "Assembly(", length(assembly.parts), " parts)")
    return nothing
end

"""
    AbstractPart

Supertype for the workpiece in the machine (nodes in a manufacturing graph).

Concrete parts carry `parent` and `op` fields accessed via `parent(part)` and
`operation(part)`. Both are `nothing` for raw stock, and both are present when
a shop operation produced the part. `parent` extends `Base.parent`.
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

A root `RectangularTube` (`parent === nothing` and `op === nothing`) represents
raw stock. Enforces positive dimensions and `2 * thickness < min(width, height)`.
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
    RoundTube(outer_diameter, length, thickness, parent=nothing, op=nothing)

Round-tube part node with outer cross-section `outer_diameter`, tube `length` (Z),
and wall `thickness`, all in inches (`Float64`).

A root `RoundTube` (`parent === nothing` and `op === nothing`) represents raw
stock. Enforces positive dimensions and `2 * thickness < outer_diameter`.
"""
struct RoundTube <: AbstractTube
    outer_diameter::Float64
    length::Float64
    thickness::Float64
    parent::Union{Nothing,AbstractPart}
    op::Union{Nothing,AbstractOperation}

    function RoundTube(
        outer_diameter::Real,
        length::Real,
        thickness::Real,
        parent::Union{Nothing,AbstractPart}=nothing,
        op::Union{Nothing,AbstractOperation}=nothing,
    )
        od = Float64(outer_diameter)
        l = Float64(length)
        t = Float64(thickness)
        od > 0.0 || throw(DomainError(outer_diameter, "RoundTube outer_diameter must be positive"))
        l > 0.0 || throw(DomainError(length, "RoundTube length must be positive"))
        t > 0.0 || throw(DomainError(thickness, "RoundTube thickness must be positive"))
        2.0 * t < od || throw(
            ArgumentError("RoundTube thickness ($t) must be strictly less than half of outer_diameter ($od)"),
        )
        isnothing(parent) == isnothing(op) || throw(
            ArgumentError("RoundTube parent and op must either both be nothing (raw stock) or both be present"),
        )
        return new(od, l, t, parent, op)
    end
end

"""
    Plate(width, height, thickness, parent=nothing, op=nothing)

Flat rectangular plate part node with dimensions `width` (X), `height` (Y), and
`thickness` (Z), all in inches (`Float64`).

A root `Plate` (`parent === nothing` and `op === nothing`) represents raw stock.
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

function _to_geos_polygon(loop::AbstractVector{Point2d})
    coords = Vector{Float64}[[p[1], p[2]] for p in loop]
    push!(coords, [loop[1][1], loop[1][2]])
    return LibGEOS.Polygon([coords])
end

_is_ccw(poly::LibGEOS.Polygon) =
    LibGEOS.isCCW(LibGEOS.getCoordSeq(LibGEOS.exteriorRing(poly)))

function _from_geos_polygon(poly::LibGEOS.Polygon)
    seq = LibGEOS.getCoordSeq(LibGEOS.exteriorRing(poly))
    coords = LibGEOS.getCoordinates(seq)
    pts = Point2d[Point2d(c[1], c[2]) for c in @view coords[1:(end - 1)]]
    LibGEOS.isCCW(seq) || reverse!(pts)
    return pts
end

"""
    Polygon(points)

Polygonal 2D cut profile in a `Datum`'s XY plane defined by at least 3
non-collinear vertices (inches, `Float64`), normalized to counter-clockwise
winding order.
"""
struct Polygon <: AbstractProfile
    points::Vector{Point2d}

    function Polygon(points::AbstractVector{<:AbstractVector{<:Real}})
        length(points) >= 3 || throw(ArgumentError("Polygon must have at least 3 vertices"))
        pts = Point2d[Point2d(Float64(p[1]), Float64(p[2])) for p in points]
        poly = _to_geos_polygon(pts)
        LibGEOS.area(poly) > _ATOL || throw(ArgumentError("Polygon vertices must not be collinear"))
        _is_ccw(poly) || reverse!(pts)
        return new(pts)
    end
end

"""
    Rectangle(width, height) -> Polygon

Convenience constructor returning a centered `width × height` rectangular
`Polygon` profile in a `Datum`'s XY plane.
"""
function Rectangle(width::Real, height::Real)
    w = Float64(width)
    h = Float64(height)
    w > 0.0 || throw(DomainError(width, "Rectangle width must be positive"))
    h > 0.0 || throw(DomainError(height, "Rectangle height must be positive"))
    hw = w / 2.0
    hh = h / 2.0
    return Polygon(Point2d[
        Point2d(-hw, -hh),
        Point2d( hw, -hh),
        Point2d( hw,  hh),
        Point2d(-hw,  hh),
    ])
end

const RigidTransform = Union{
    AffineMap{<:Rotation{3},<:SVector{3,<:Real}},
    Translation{<:SVector{3,<:Real}},
    LinearMap{<:Rotation{3}},
}

const IdentityRigidTransform = AffineMap(one(RotMatrix3), zero(SVector{3}))

"""
    Datum(transform::RigidTransform=IdentityRigidTransform)

Right-handed 3D coordinate frame (`SE(3)` rigid transformation relative to the
part's 3D origin):
- `origin(datum)` is the 3D origin in part coordinates,
- `xaxis(datum)` and `yaxis(datum)` are the orthonormal 2D axes on the datum surface,
- `zaxis(datum)` is the outward surface normal (tools approach along `-zaxis(datum)`).
"""
struct Datum <: AbstractDatum
    transform::AffineMap{<:Rotation{3},SVector{3,Float64}}

    function Datum(transform::RigidTransform=IdentityRigidTransform)
        return new(IdentityRigidTransform ∘ transform)
    end
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
        Datum(LinearMap(RotX(π / 2))),                                              # front (-Y)
        Datum(Translation(w,   0.0, 0.0) ∘ LinearMap(RotZ(π / 2) * RotX(π / 2))),   # right (+X)
        Datum(Translation(w,   h,   0.0) ∘ LinearMap(RotZ(π) * RotX(π / 2))),       # back  (+Y)
        Datum(Translation(0.0, h,   0.0) ∘ LinearMap(RotZ(-π / 2) * RotX(π / 2))),  # left  (-X)
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
        Datum(Translation(0.0, 0.0, t)),                                            # top    (+Z)
        Datum(Translation(w,   0.0, 0.0) ∘ LinearMap(RotZ(π) * RotX(π))),           # bottom (-Z)
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
    Saw(datum::Datum)

Saw operation slicing along the XY plane of `datum`, keeping the half-space on
the `-zaxis(datum)` side (`dot(p - origin(datum), zaxis(datum)) <= 0`) so that
`zaxis(datum)` is the outward surface normal of the newly cut face.
"""
struct Saw <: AbstractOperation
    datum::Datum
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
    Cut(profile::Polygon, datum::Datum)

Profile-cutting operation cutting polygonal `profile` through the wall at `datum`.
"""
struct Cut <: AbstractOperation
    profile::Polygon
    datum::Datum
end

"""
    Weld()

Welding operation joining two or more parts into a rigid `Weldment`.
"""
struct Weld <: AbstractOperation end

"""
    Weldment(parts, op=Weld())

Welded multi-part `AbstractPart` node joining two or more `AbstractPart`s in a
shared right-handed 3D coordinate frame via `part => transform` pairs, where
each `transform` is a `RigidTransform`.
"""
struct Weldment <: AbstractPart
    parts::Vector{Pair{AbstractPart,RigidTransform}}
    op::Weld

    function Weldment(parts::AbstractVector{<:Pair}, op::Weld=Weld())
        length(parts) >= 2 || throw(ArgumentError("Weldment must contain at least two parts"))
        return new(Pair{AbstractPart,RigidTransform}[p.first => p.second for p in parts], op)
    end
end

Base.length(weldment::Weldment) = length(weldment.parts)
Base.iterate(weldment::Weldment, state...) = iterate(weldment.parts, state...)
Base.getindex(weldment::Weldment, i::Integer) = weldment.parts[i]

function Base.show(io::IO, weldment::Weldment)
    print(io, "Weldment(", length(weldment.parts), " parts)")
    return nothing
end

"""
    parent(part::AbstractPart) -> Union{Nothing,AbstractPart,Vector{AbstractPart}}

Return the part(s) `part` was made from: `nothing` for raw stock, the single
input `AbstractPart` for operations such as `saw`, `drill`, and `cut`, or a
`Vector{AbstractPart}` of welded components for a `Weldment`.
"""
parent(part::AbstractPart) = part.parent
parent(part::Weldment) = AbstractPart[p.first for p in part.parts]

"""
    operation(part::AbstractPart) -> Union{Nothing,AbstractOperation}

Return the shop operation that produced `part`, or `nothing` for raw stock.
"""
operation(part::AbstractPart) = part.op

"""
    isstock(part::AbstractPart) -> Bool

Return `true` if `part` is raw mill stock (`parent(part) === nothing`).
"""
isstock(part::AbstractPart) = isnothing(parent(part))

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
    _rebuild(part::T; kwargs...) where {T<:AbstractPart} -> T

Construct a new part of the same concrete type `T` as `part`, copying all fields
from `part` except those overridden by keyword arguments in `kwargs` (such as
`parent`, `op`, or updated dimensions).
"""
function _rebuild(part::T; kwargs...) where {T<:AbstractPart}
    vals = map(fieldnames(T)) do name
        return get(values(kwargs), name, getfield(part, name))
    end
    return T(vals...)
end

function _root_stock(part::AbstractPart)
    curr = part
    while !isnothing(parent(curr))
        curr = parent(curr)
    end
    return curr
end

function _plane_z(datum::Datum, x::Real, y::Real)
    p0 = origin(datum)
    n = zaxis(datum)
    return p0[3] - (n[1] * (x - p0[1]) + n[2] * (y - p0[2])) / n[3]
end

function _collect_nodes(part::AbstractPart)
    nodes = AbstractPart[]
    visited = IdSet{AbstractPart}()
    function visit(node::AbstractPart)
        node in visited && return
        push!(visited, node)
        p = parent(node)
        if p isa AbstractVector
            foreach(visit, p)
        elseif !isnothing(p)
            visit(p)
        end
        push!(nodes, node)
        return nothing
    end
    visit(part)
    return nodes
end

"""
    operations_of_type(::Type{T}, part::AbstractPart) where {T<:AbstractOperation} -> Vector{T}

Collect all operations of type `T` in the manufacturing DAG of `part` in
topological order (from root stock to `part`).
"""
function operations_of_type(::Type{T}, part::AbstractPart) where {T<:AbstractOperation}
    result = T[]
    for node in _collect_nodes(part)
        op = operation(node)
        op isa T && push!(result, op)
    end
    return result
end

function _tube_z_bounds(part::AbstractTube, x::Real, y::Real)
    root = _root_stock(part)
    zb = 0.0
    zt = root.length
    for op in operations_of_type(Saw, part)
        zp = _plane_z(op.datum, x, y)
        if zaxis(op.datum)[3] > 0.0
            zt = min(zt, zp)
        else
            zb = max(zb, zp)
        end
    end
    return (zb, zt)
end

_cross_section_points(part::RectangularTube) = (
    SVector(0.0, 0.0),
    SVector(part.width, 0.0),
    SVector(part.width, part.height),
    SVector(0.0, part.height),
)

function _cross_section_points(part::RoundTube)
    r = part.outer_diameter / 2.0
    return [
        SVector(r * cos(2π * (i - 1) / 32), r * sin(2π * (i - 1) / 32))
        for i in 1:32
    ]
end

function _halfplane_polygon(
    poly::LibGEOS.Polygon,
    p0::SVector{3,Float64},
    n::SVector{3,Float64},
)
    c = SVector(p0[1], p0[2])
    u = SVector(n[1], n[2])
    v = SVector(-u[2], u[1])
    xmin, xmax = LibGEOS.getXMin(poly), LibGEOS.getXMax(poly)
    ymin, ymax = LibGEOS.getYMin(poly), LibGEOS.getYMax(poly)
    span = 2.0 * (hypot(xmax - xmin, ymax - ymin) + hypot(c[1] - xmin, c[2] - ymin) + 1.0)
    return _to_geos_polygon(Point2d[
        Point2d(c - span * v),
        Point2d(c + span * v),
        Point2d(c + span * v - span * u),
        Point2d(c - span * v - span * u),
    ])
end

function _plate_polygon(part::Plate)
    root = _root_stock(part)
    poly = _to_geos_polygon(Point2d[
        Point2d(0.0, 0.0),
        Point2d(root.width, 0.0),
        Point2d(root.width, root.height),
        Point2d(0.0, root.height),
    ])
    for op in operations_of_type(Saw, part)
        poly = LibGEOS.intersection(poly, _halfplane_polygon(poly, origin(op.datum), zaxis(op.datum)))
    end
    return poly
end

"""
    saw(part::AbstractTube, datum::Datum)

Saw a tubular `part` (cutoff or miter) along the XY plane of `datum`, keeping
the `-zaxis(datum)` half-space. Validates that the saw plane slices across the
entire tube cross-section within the current tube ends.
"""
function saw(part::AbstractTube, datum::Datum)
    p0 = origin(datum)
    n = zaxis(datum)
    pts = _cross_section_points(part)
    for p in pts
        zb, zt = _tube_z_bounds(part, p[1], p[2])
        d_bot = dot(SVector(p[1], p[2], zb) - p0, n)
        d_top = dot(SVector(p[1], p[2], zt) - p0, n)
        (min(d_bot, d_top) < -_ATOL && max(d_bot, d_top) >= -_ATOL) || throw(
            ArgumentError(
                "Tube saw plane at $p0 with normal $n must slice across the tube between z=$zb and z=$zt at (x=$(p[1]), y=$(p[2]))",
            ),
        )
    end
    op = Saw(datum)
    sawn = _rebuild(part; parent=part, op=op)
    bounds = [_tube_z_bounds(sawn, p[1], p[2]) for p in pts]
    new_length = maximum(last, bounds) - minimum(first, bounds)
    return _rebuild(sawn; length=new_length)
end

"""
    saw(part::Plate, datum::Datum)

Bandsaw a `Plate` along the XY plane of `datum` (whose normal `zaxis(datum)`
must lie in the plate's XY plane), keeping the `-zaxis(datum)` half-space.
Validates that the cut line slices through the interior of the current plate
polygon.
"""
function saw(part::Plate, datum::Datum)
    n = zaxis(datum)
    abs(n[3]) <= _ATOL || throw(
        ArgumentError("Plate saw datum normal $n must lie in the XY plane (z-component must be 0)"),
    )
    p0 = origin(datum)
    poly = _plate_polygon(part)
    clipped = LibGEOS.intersection(poly, _halfplane_polygon(poly, p0, n))
    (clipped isa LibGEOS.Polygon && _ATOL < LibGEOS.area(clipped) < LibGEOS.area(poly) - _ATOL) || throw(
        ArgumentError(
            "Plate saw plane at $p0 with normal $n must slice through the plate interior",
        ),
    )
    new_w = LibGEOS.getXMax(clipped) - LibGEOS.getXMin(clipped)
    new_h = LibGEOS.getYMax(clipped) - LibGEOS.getYMin(clipped)
    op = Saw(datum)
    return _rebuild(part; width=new_w, height=new_h, parent=part, op=op)
end

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
lies wholly within the sawn tube ends.
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
    cross_depth = abs(dot(zaxis(face), SVector(part.width, part.height, part.length)))
    z_depths =
        penetration isa ThroughAll ?
        (0.0, -part.thickness, -cross_depth + part.thickness, -cross_depth) :
        (0.0, -part.thickness)
    for u in (x - r, x + r), zd in z_depths
        p3 = face.transform(SVector(u, y, zd))
        zb, zt = _tube_z_bounds(part, p3[1], p3[2])
        (zb + r < y && y < zt - r) || throw(
            ArgumentError(
                "Hole (diameter=$(profile.diameter), y=$y) must lie wholly within tube ends ($zb, $zt)",
            ),
        )
    end

    op = Drill(profile, datum, penetration)
    return _rebuild(part; parent=part, op=op)
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
strictly within the plate perimeter, and for `Blind` holes that
`penetration.depth < part.thickness`.
"""
function drill(
    part::Plate,
    profile::Circle,
    datum::Datum,
    penetration::AbstractPenetration,
)
    _match_face(part, datum)
    r = profile.diameter / 2.0
    shrunk = LibGEOS.buffer(_plate_polygon(part), -(r + _ATOL))
    p3 = origin(datum)
    LibGEOS.contains(shrunk, LibGEOS.Point(p3[1], p3[2])) || throw(
        ArgumentError(
            "Hole (diameter=$(profile.diameter)) at ($(p3[1]), $(p3[2])) must lie wholly within plate perimeter",
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
    return _rebuild(part; parent=part, op=op)
end

"""
    cut(part::RectangularTube, profile::Polygon, datum::Datum)

Cut polygonal `profile` through the wall of `part` at `datum`. Validates that
`datum` lies on a valid outer face of `part` and that every vertex of `profile`
fits strictly within the inner cavity width and sawn tube ends.
"""
function cut(
    part::RectangularTube,
    profile::Polygon,
    datum::Datum,
)
    face, _, _ = _match_face(part, datum)
    face_width = _face_width(part, face)
    T_face_datum = inv(face.transform) ∘ datum.transform
    for u in profile.points
        pf = T_face_datum(SVector(u[1], u[2], 0.0))
        (part.thickness < pf[1] && pf[1] < face_width - part.thickness) || throw(
            ArgumentError(
                "Cutout vertex at local x=$(pf[1]) must fit strictly within inner cavity x ∈ ($(part.thickness), $(face_width - part.thickness))",
            ),
        )
        for zd in (0.0, -part.thickness)
            p3 = face.transform(SVector(pf[1], pf[2], zd))
            zb, zt = _tube_z_bounds(part, p3[1], p3[2])
            (zb < pf[2] && pf[2] < zt) || throw(
                ArgumentError(
                    "Cutout vertex at local y=$(pf[2]) must lie wholly within tube ends ($zb, $zt)",
                ),
            )
        end
    end

    op = Cut(profile, datum)
    return _rebuild(part; parent=part, op=op)
end

"""
    cut(part::Plate, profile::Polygon, datum::Datum)

Cut polygonal `profile` through `part`, where `profile` is expressed in the
coordinate frame of `datum`. Validates that `datum` lies on a valid flat surface
of `part` and that `profile` fits strictly within the plate perimeter.
"""
function cut(
    part::Plate,
    profile::Polygon,
    datum::Datum,
)
    _match_face(part, datum)
    shrunk = LibGEOS.buffer(_plate_polygon(part), -_ATOL)
    cutout = _to_geos_polygon(Point2d[
        Point2d(datum.transform(SVector(u[1], u[2], 0.0))[1:2]) for u in profile.points
    ])
    LibGEOS.contains(shrunk, cutout) || throw(
        ArgumentError(
            "Cutout at $(origin(datum)) must lie wholly within plate perimeter",
        ),
    )

    op = Cut(profile, datum)
    return _rebuild(part; parent=part, op=op)
end

"""
    weld(parts::AbstractVector{<:Pair}) -> Weldment

Weld two or more `part => transform` pairs (`AbstractPart => RigidTransform`)
into a rigid `Weldment`.
"""
weld(parts::AbstractVector{<:Pair}) = Weldment(parts)

"""
    Assembly(parts)

Multi-part 3D assembly positioning each `AbstractPart` in a shared right-handed
3D coordinate frame via a `Pair` `part => transform`, where `transform` is a
`RigidTransform`.
"""
struct Assembly
    parts::Vector{Pair{AbstractPart,RigidTransform}}

    function Assembly(parts::AbstractVector{<:Pair})
        isempty(parts) && throw(ArgumentError("Assembly must contain at least one part"))
        return new(Pair{AbstractPart,RigidTransform}[p.first => p.second for p in parts])
    end
end

Base.length(assembly::Assembly) = length(assembly.parts)
Base.iterate(assembly::Assembly, state...) = iterate(assembly.parts, state...)
Base.getindex(assembly::Assembly, i::Integer) = assembly.parts[i]

function Base.show(io::IO, assembly::Assembly)
    print(io, "Assembly(", length(assembly.parts), " parts)")
    return nothing
end

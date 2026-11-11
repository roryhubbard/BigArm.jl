"""
    AbstractPenetration

Supertype for discrete cut penetration / termination modes.
"""
abstract type AbstractPenetration end

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
    Weld(datum_a::Datum, datum_b::Datum, profile::AbstractProfile)

Welding operation joining `part_b` (at `datum_b`) to `part_a` (at `datum_a`)
along `profile`.
"""
struct Weld <: AbstractOperation
    datum_a::Datum
    datum_b::Datum
    profile::AbstractProfile
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

# Evaluate the Z height where `datum`'s XY plane intersects the vertical line at (x, y).
function _plane_z(datum::Datum, x::Real, y::Real)
    p0 = origin(datum)
    n = zaxis(datum)
    return p0[3] - (n[1] * (x - p0[1]) + n[2] * (y - p0[2])) / n[3]
end

# Post-order DFS over `parent(node)` collecting all unique ancestor parts in topological order.
function _collect_nodes(part::AbstractPart)
    nodes = AbstractPart[]
    visited = IdSet{AbstractPart}()
    function visit(node::AbstractPart)
        node in visited && return
        push!(visited, node)
        p = parent(node)
        if p isa Pair
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

# Compute the active (z_bottom, z_top) axial bounds of `part` along the vertical line (x, y)
# after applying all `Saw` operations in its history (each keeping the `-zaxis` half-space).
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

_cross_section_points(part::RectangularTube) = _rect_outline(0.0, 0.0, part.width, part.height)
_cross_section_points(part::RoundTube) = _outline(Circle(part.outer_diameter))

"""
    _tube_wall_outline(part::RectangularTube, face::Datum, inset::Real=0.0) -> Vector{Point2d}

Return the 4 corners of one flat wall `face` of a `RectangularTube` in `face`'s
2D coordinates:
- `inset = 0` gives the **outer** wall surface (`x` from `0` to `w`).
- `inset = part.thickness` gives the **inner** cavity wall surface (`x` from `t`
  to `w - t`, stepped `t` inward so its corners meet the adjacent inner walls).

At the left (`x0`) and right (`x1`) edges, `_tube_z_bounds` finds the bottom and
top heights after any straight or miter `Saw` cuts.
"""
function _tube_wall_outline(part::RectangularTube, face::Datum, inset::Real=0.0)
    d = Float64(inset)
    x0, x1 = d, _face_width(part, face) - d
    p0 = face.transform(SVector(x0, 0.0, -d))
    p1 = face.transform(SVector(x1, 0.0, -d))
    zb0, zt0 = _tube_z_bounds(part, p0[1], p0[2])
    zb1, zt1 = _tube_z_bounds(part, p1[1], p1[2])
    return Point2d[Point2d(x0, zb0), Point2d(x1, zb1), Point2d(x1, zt1), Point2d(x0, zt0)]
end

# Build a large 2D bounding polygon representing the kept half-plane `{q : dot(q - p0, n) <= 0}`
# so that `LibGEOS.intersection(poly, halfplane)` clips `poly` against the saw line.
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

# Replay all `Saw` cuts on `part` starting from the root rectangular stock to obtain
# the current outer 2D polygon of the plate.
function _plate_polygon(part::Plate)
    root = _root_stock(part)
    poly = _to_geos_polygon(_rect_outline(0.0, 0.0, root.width, root.height))
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
    cutout = _to_geos_polygon(_project_outline(profile, datum))
    LibGEOS.contains(shrunk, cutout) || throw(
        ArgumentError(
            "Cutout at $(origin(datum)) must lie wholly within plate perimeter",
        ),
    )

    op = Cut(profile, datum)
    return _rebuild(part; parent=part, op=op)
end

function _pierces_face(op::Drill, face::Datum)
    d = dot(zaxis(op.datum), zaxis(face))
    return d > 1.0 - _ATOL || (op.penetration isa ThroughAll && d < -1.0 + _ATOL)
end

_pierces_face(op::Cut, face::Datum) = dot(zaxis(op.datum), zaxis(face)) > 1.0 - _ATOL

# Return the 3D vertices of a weld seam `profile` in `datum`'s frame:
# - For `LineSegment(L)`, the two endpoints `(0, 0, 0)` and `(L, 0, 0)` along `+xaxis`.
# - For `Circle(d)`, the closed 32-segment outline in the datum XY plane.
function _seam_points_3d(profile::LineSegment, datum::Datum)
    return SVector{3,Float64}[
        datum.transform(SVector(0.0, 0.0, 0.0)),
        datum.transform(SVector(profile.length, 0.0, 0.0)),
    ]
end

function _seam_points_3d(profile::Circle, datum::Datum)
    pts_3d = SVector{3,Float64}[datum.transform(SVector(u[1], u[2], 0.0)) for u in _outline(profile)]
    # Repeat the first vertex at the end so `LibGEOS.LineString` includes the closing segment `pts_3d[end] -> pts_3d[1]`.
    push!(pts_3d, pts_3d[1])
    return pts_3d
end

"""
    _covers_seam(poly_2d::LibGEOS.Polygon, pts_3d, on_face::Bool) -> Bool

Check whether the 1D XY projection (`seam_1d`) of 3D seam vertices `pts_3d` is
covered (within `_ATOL`) by the 2D polygon `poly_2d` when `on_face=true` (seam
lies on a flat face or end cap), or by its 1D border `LibGEOS.boundary(poly_2d)`
when `on_face=false` (seam runs along an outer edge or hole rim).
"""
function _covers_seam(
    poly_2d::LibGEOS.Polygon,
    pts_3d::AbstractVector{SVector{3,Float64}},
    on_face::Bool,
)
    target = on_face ? poly_2d : LibGEOS.boundary(poly_2d)
    seam_1d = LibGEOS.LineString(Vector{Float64}[[p[1], p[2]] for p in pts_3d])
    return LibGEOS.covers(LibGEOS.buffer(target, _ATOL), seam_1d)
end

"""
    _seam_on_slab(poly_2d::LibGEOS.Polygon, pts_3d, z_lo::Real, z_hi::Real) -> Bool

Check whether 3D seam points `pts_3d` lie on the surface of a 2.5D slab with 2D
footprint `poly_2d` spanning `z ∈ [z_lo, z_hi]`: all points must lie within
`[z_lo, z_hi]`, and their XY projection must be covered by `poly_2d` when on the
flat faces (`z ≈ z_lo` or `z ≈ z_hi`) or by `LibGEOS.boundary(poly_2d)` when
along the side/hole walls.
"""
function _seam_on_slab(
    poly_2d::LibGEOS.Polygon,
    pts_3d::AbstractVector{SVector{3,Float64}},
    z_lo::Real,
    z_hi::Real,
)
    all(p -> z_lo - _ATOL <= p[3] <= z_hi + _ATOL, pts_3d) || return false
    on_face =
        all(p -> abs(p[3] - z_lo) <= _ATOL, pts_3d) ||
        all(p -> abs(p[3] - z_hi) <= _ATOL, pts_3d)
    return _covers_seam(poly_2d, pts_3d, on_face)
end

function _within_tube_z(part::AbstractTube, pts_3d::AbstractVector{SVector{3,Float64}})
    return all(pts_3d) do p
        zb, zt = _tube_z_bounds(part, p[1], p[2])
        return zb - _ATOL <= p[3] <= zt + _ATOL
    end
end

function _on_tube_end_cap(part::AbstractTube, pts_3d::AbstractVector{SVector{3,Float64}})
    return all(p -> abs(p[3] - _tube_z_bounds(part, p[1], p[2])[1]) <= _ATOL, pts_3d) ||
           all(p -> abs(p[3] - _tube_z_bounds(part, p[1], p[2])[2]) <= _ATOL, pts_3d)
end

# Validate that `profile` at `datum` lies on the surface of `part::Plate` using `LibGEOS`:
# subtract all active hole/cutout polygons from `_plate_polygon(part)` via `LibGEOS.difference`,
# then check `_seam_on_slab` across `z ∈ [0, thickness]`.
function _seam_on_part(part::Plate, datum::Datum, profile::Union{LineSegment,Circle})
    pts_3d = _seam_points_3d(profile, datum)
    t = part.thickness
    poly_2d = _plate_polygon(part)
    for op in operations_of_type(Union{Drill,Cut}, part)
        if op isa Drill && op.penetration isa Blind
            in_depth =
                zaxis(op.datum)[3] > 0.0 ?
                any(p -> p[3] >= t - op.penetration.depth - _ATOL, pts_3d) :
                any(p -> p[3] <= op.penetration.depth + _ATOL, pts_3d)
            in_depth || continue
        end
        poly_2d = LibGEOS.difference(poly_2d, _to_geos_polygon(_project_outline(op.profile, op.datum)))
    end

    return _seam_on_slab(poly_2d, pts_3d, 0.0, t)
end

# Validate that `profile` at `datum` lies on the surface of `part::RoundTube` by constructing
# the 2D annular cross-section via `LibGEOS.difference(outer_2d, inner_2d)` and checking
# `_covers_seam` against `annulus_2d` (for sawn end caps) or its 1D boundary (for cylindrical walls).
function _seam_on_part(part::RoundTube, datum::Datum, profile::Union{LineSegment,Circle})
    pts_3d = _seam_points_3d(profile, datum)
    _within_tube_z(part, pts_3d) || return false

    outer_2d = _to_geos_polygon(_outline(Circle(part.outer_diameter)))
    inner_2d = _to_geos_polygon(_outline(Circle(part.outer_diameter - 2.0 * part.thickness)))
    annulus_2d = LibGEOS.difference(outer_2d, inner_2d)
    return _covers_seam(annulus_2d, pts_3d, _on_tube_end_cap(part, pts_3d))
end

# Validate that `profile` at `datum` lies on the surface of `part::RectangularTube` by checking
# either the top/bottom sawn annular end caps or any of the 4 perforated tube walls via `LibGEOS`.
function _seam_on_part(part::RectangularTube, datum::Datum, profile::Union{LineSegment,Circle})
    pts_3d = _seam_points_3d(profile, datum)
    _within_tube_z(part, pts_3d) || return false
    w, h, t = part.width, part.height, part.thickness

    # 1. Check top/bottom sawn annular end caps in XY.
    if _on_tube_end_cap(part, pts_3d)
        annulus_2d = LibGEOS.difference(
            _to_geos_polygon(_rect_outline(0.0, 0.0, w, h)),
            _to_geos_polygon(_rect_outline(t, t, w - t, h - t)),
        )
        _covers_seam(annulus_2d, pts_3d, true) && return true
    end

    # 2. Check each of the 4 tube walls in its local face coordinate frame.
    for face in datums(part)
        fpts_3d = SVector{3,Float64}[inv(face.transform)(p) for p in pts_3d]
        wall_poly_2d = _to_geos_polygon(_tube_wall_outline(part, face))
        for op in operations_of_type(Union{Drill,Cut}, part)
            _pierces_face(op, face) || continue
            wall_poly_2d = LibGEOS.difference(wall_poly_2d, _to_geos_polygon(_project_outline(op.profile, op.datum, face)))
        end

        _seam_on_slab(wall_poly_2d, fpts_3d, -t, 0.0) && return true
    end
    return false
end

# Relative rigid transform placing `part_b` into `part_a`'s frame by gluing `datum_b` onto `datum_a`.
_weld_transform(op::Weld) = op.datum_a.transform ∘ inv(op.datum_b.transform)

function _seam_on_part(part::Weldment, datum::Datum, profile::Union{LineSegment,Circle})
    T_a_b = _weld_transform(part.op)
    return _seam_on_part(part.parent[1], datum, profile) ||
           _seam_on_part(part.parent[2], Datum(inv(T_a_b) ∘ datum.transform), profile)
end

"""
    weld(
        part_a::AbstractPart,
        datum_a::Datum,
        part_b::AbstractPart,
        datum_b::Datum,
        profile::Union{LineSegment,Circle},
    ) -> Weldment

Weld `part_b` (at `datum_b` in `part_b`'s frame) onto `part_a` (at `datum_a` in
`part_a`'s frame) along `profile` (a `LineSegment` along `+xaxis` or a `Circle`
in the datum XY plane).

Validates that `profile` lies on the surface of both `part_a` (at `datum_a`) and
`part_b` (at `datum_b`).
"""
function weld(
    part_a::AbstractPart,
    datum_a::Datum,
    part_b::AbstractPart,
    datum_b::Datum,
    profile::Union{LineSegment,Circle},
)
    op = Weld(datum_a, datum_b, profile)
    _seam_on_part(part_a, datum_a, profile) || throw(
        ArgumentError(
            "Weld seam at $(origin(datum_a)) does not lie on the surface of first part ($(typeof(part_a)))",
        ),
    )
    _seam_on_part(part_b, datum_b, profile) || throw(
        ArgumentError(
            "Weld seam at $(origin(datum_b)) does not lie on the surface of second part ($(typeof(part_b)))",
        ),
    )
    return Weldment(part_a => part_b, op)
end

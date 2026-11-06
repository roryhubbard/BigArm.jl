const _CIRCLE_SEGMENTS = 32

function _signed_area(loop::AbstractVector{Point2d})
    n = length(loop)
    acc = 0.0
    for i in 1:n
        p1 = loop[i]
        p2 = loop[i == n ? 1 : i + 1]
        acc += p1[1] * p2[2] - p2[1] * p1[2]
    end
    return 0.5 * acc
end

function _rect_loop(x0::Real, y0::Real, x1::Real, y1::Real)
    return Point2d[
        Point2d(Float64(x0), Float64(y0)),
        Point2d(Float64(x1), Float64(y0)),
        Point2d(Float64(x1), Float64(y1)),
        Point2d(Float64(x0), Float64(y1)),
    ]
end

"""
    _contour(profile::AbstractProfile) -> Vector{Point2d}

Return a counter-clockwise 2D polygon loop approximating `profile` in its local
datum XY plane centered at `(0, 0)`.
"""
function _contour(profile::Circle; segments::Int=_CIRCLE_SEGMENTS)
    r = profile.diameter / 2.0
    return Point2d[
        Point2d(r * cos(2π * (i - 1) / segments), r * sin(2π * (i - 1) / segments))
        for i in 1:segments
    ]
end

function _project_2d(T, u::Point2d)
    p = T(SVector(u[1], u[2], 0.0))
    return Point2d(p[1], p[2])
end

function _project_hole(op::Drill, onto::Datum)
    T = inv(onto.transform) ∘ op.datum.transform
    loop = [_project_2d(T, u) for u in _contour(op.profile)]
    _signed_area(loop) < 0.0 && reverse!(loop)
    return loop
end

function _pierces_face(op::Drill, face::Datum)
    d = dot(zaxis(op.datum), zaxis(face))
    return d > 1.0 - _ATOL || (op.penetration isa ThroughAll && d < -1.0 + _ATOL)
end

function _face_holes(
    part::AbstractPart,
    face::Datum;
    y_max::Union{Nothing,Real}=nothing,
)
    result = Vector{Point2d}[]
    for op in holes(part)
        _pierces_face(op, face) || continue
        loop = _project_hole(op, face)
        if !isnothing(y_max)
            all(p -> 0.0 < p[2] < y_max, loop) || continue
        end
        push!(result, loop)
    end
    return result
end

struct _MeshBuilder
    points::Vector{Point3d}
    index::Dict{NTuple{3,Int64},Int}
    faces::Vector{TriangleFace{Int}}
end

_MeshBuilder() = _MeshBuilder(Point3d[], Dict{NTuple{3,Int64},Int}(), TriangleFace{Int}[])

function _vertex!(b::_MeshBuilder, p::SVector{3,Float64})
    key = (round(Int64, p[1] / _ATOL), round(Int64, p[2] / _ATOL), round(Int64, p[3] / _ATOL))
    return get!(b.index, key) do
        push!(b.points, Point3d(p[1], p[2], p[3]))
        return length(b.points)
    end
end

function _triangulate_2d(
    outer::AbstractVector{Point2d},
    hole_loops::AbstractVector{<:AbstractVector{Point2d}}=Vector{Point2d}[],
)
    pts = Point2d[]
    segs = Tuple{Int32,Int32}[]

    function add_loop!(loop)
        base = Int32(length(pts))
        n = Int32(length(loop))
        append!(pts, loop)
        for i in Int32(1):n
            push!(segs, (base + i, base + (i == n ? Int32(1) : i + Int32(1))))
        end
    end

    add_loop!(outer)
    for loop in hole_loops
        add_loop!(loop)
    end

    triin = TriangulateIO()
    triin.pointlist = [j == 1 ? p[1] : p[2] for j in 1:2, p in pts]
    triin.segmentlist = [j == 1 ? s[1] : s[2] for j in 1:2, s in segs]
    if !isempty(hole_loops)
        seeds = [sum(loop) / length(loop) for loop in hole_loops]
        triin.holelist = [j == 1 ? c[1] : c[2] for j in 1:2, c in seeds]
    end

    triout, _ = triangulate("pQY", triin)
    out_pts = [
        Point2d(triout.pointlist[1, i], triout.pointlist[2, i])
        for i in axes(triout.pointlist, 2)
    ]
    out_tris = [
        (Int(triout.trianglelist[1, i]), Int(triout.trianglelist[2, i]), Int(triout.trianglelist[3, i]))
        for i in axes(triout.trianglelist, 2)
    ]
    return out_pts, out_tris
end

function _add_planar_face!(
    b::_MeshBuilder,
    datum::Datum,
    outer::Vector{Point2d},
    hole_loops::Vector{Vector{Point2d}}=Vector{Point2d}[];
    z::Real=0.0,
    flip::Bool=false,
)
    pts2d, tris = _triangulate_2d(outer, hole_loops)
    zf = Float64(z)
    vmap = [_vertex!(b, datum.transform(SVector(p[1], p[2], zf))) for p in pts2d]
    for (i1, i2, i3) in tris
        if flip
            push!(b.faces, TriangleFace{Int}(vmap[i1], vmap[i3], vmap[i2]))
        else
            push!(b.faces, TriangleFace{Int}(vmap[i1], vmap[i2], vmap[i3]))
        end
    end
    return nothing
end

function _add_extruded_loop!(
    b::_MeshBuilder,
    datum::Datum,
    loop::AbstractVector{Point2d};
    z_front::Real=0.0,
    z_back::Real,
    outward::Bool=false,
)
    zf = Float64(z_front)
    zb = Float64(z_back)
    n = length(loop)
    front_v = [_vertex!(b, datum.transform(SVector(p[1], p[2], zf))) for p in loop]
    back_v = [_vertex!(b, datum.transform(SVector(p[1], p[2], zb))) for p in loop]
    for i in 1:n
        j = i == n ? 1 : i + 1
        vf1, vf2 = front_v[i], front_v[j]
        vb1, vb2 = back_v[i], back_v[j]
        if outward
            push!(b.faces, TriangleFace{Int}(vf1, vb2, vf2))
            push!(b.faces, TriangleFace{Int}(vf1, vb1, vb2))
        else
            push!(b.faces, TriangleFace{Int}(vf1, vf2, vb2))
            push!(b.faces, TriangleFace{Int}(vf1, vb2, vb1))
        end
    end
    return nothing
end

"""
    _add_slab!(b, datum, outer_front, outer_back, hole_loops, thickness; close_perimeter=false)

Add a 2.5D perforated wall/slab of `thickness` along `-zaxis(datum)` to `b`:
triangulates the front face (`z = 0`) and back face (`z = -thickness`), stitches
internal hole barrels for `hole_loops`, and optionally extrudes `outer_front`
around the perimeter when `close_perimeter=true`.
"""
function _add_slab!(
    b::_MeshBuilder,
    datum::Datum,
    outer_front::Vector{Point2d},
    outer_back::Vector{Point2d},
    hole_loops::Vector{Vector{Point2d}},
    thickness::Real;
    close_perimeter::Bool=false,
)
    t = Float64(thickness)
    _add_planar_face!(b, datum, outer_front, hole_loops; z=0.0, flip=false)
    _add_planar_face!(b, datum, outer_back, hole_loops; z=-t, flip=true)
    if close_perimeter
        _add_extruded_loop!(b, datum, outer_front; z_front=0.0, z_back=-t, outward=true)
    end
    for loop in hole_loops
        _add_extruded_loop!(b, datum, loop; z_front=0.0, z_back=-t, outward=false)
    end
    return nothing
end

"""
    derive_mesh(part::RectangularTube) -> GeometryBasics.Mesh

Derive a closed, watertight 3D triangle mesh for `part` from its dimensions and
the `Drill` operations in its manufacturing history.
"""
function derive_mesh(part::RectangularTube)
    b = _MeshBuilder()
    t = part.thickness
    for face in datums(part)
        w = _face_width(part, face)
        outer_rect = _rect_loop(0.0, 0.0, w, part.length)
        inner_rect = _rect_loop(t, 0.0, w - t, part.length)
        wall_holes = _face_holes(part, face; y_max=part.length)
        _add_slab!(b, face, outer_rect, inner_rect, wall_holes, t)
    end

    cap_datum = Datum(Translation(0.0, 0.0, 0.0) ∘ LinearMap(RotX(0.0)))
    cap_outer = _rect_loop(0.0, 0.0, part.width, part.height)
    cap_cavity = [_rect_loop(t, t, part.width - t, part.height - t)]
    _add_planar_face!(b, cap_datum, cap_outer, cap_cavity; z=0.0, flip=true)
    _add_planar_face!(b, cap_datum, cap_outer, cap_cavity; z=part.length, flip=false)

    return Mesh(b.points, b.faces)
end

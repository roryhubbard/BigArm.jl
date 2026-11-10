function _project_2d(T, u::Point2d)
    p = T(SVector(u[1], u[2], 0.0))
    return Point2d(p[1], p[2])
end

function _project_hole(op::Union{Drill,Cut}, onto::Datum)
    T = inv(onto.transform) ∘ op.datum.transform
    loop = [_project_2d(T, u) for u in _contour(op.profile)]
    _is_ccw(_to_geos_polygon(loop)) || reverse!(loop)
    return loop
end

function _face_holes(
    part::AbstractPart,
    face::Datum,
    bounds_poly::Union{Nothing,AbstractVector{Point2d}}=nothing,
)
    result = Vector{Point2d}[]
    shrunk_bounds =
        isnothing(bounds_poly) ? nothing : LibGEOS.buffer(_to_geos_polygon(bounds_poly), -_ATOL)
    for op in operations_of_type(Union{Drill,Cut}, part)
        _pierces_face(op, face) || continue
        loop = _project_hole(op, face)
        if !isnothing(shrunk_bounds)
            LibGEOS.contains(shrunk_bounds, _to_geos_polygon(loop)) || continue
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
    front_blind::Vector{Tuple{Vector{Point2d},Float64}}=Tuple{Vector{Point2d},Float64}[],
    back_blind::Vector{Tuple{Vector{Point2d},Float64}}=Tuple{Vector{Point2d},Float64}[],
)
    t = Float64(thickness)
    front_holes = isempty(front_blind) ? hole_loops : vcat(hole_loops, first.(front_blind))
    back_holes = isempty(back_blind) ? hole_loops : vcat(hole_loops, first.(back_blind))
    _add_planar_face!(b, datum, outer_front, front_holes; z=0.0, flip=false)
    _add_planar_face!(b, datum, outer_back, back_holes; z=-t, flip=true)
    if close_perimeter
        _add_extruded_loop!(b, datum, outer_front; z_front=0.0, z_back=-t, outward=true)
    end
    for loop in hole_loops
        _add_extruded_loop!(b, datum, loop; z_front=0.0, z_back=-t, outward=false)
    end
    for (loop, depth) in front_blind
        _add_extruded_loop!(b, datum, loop; z_front=0.0, z_back=-depth, outward=false)
        _add_planar_face!(b, datum, loop; z=-depth, flip=false)
    end
    for (loop, depth) in back_blind
        _add_extruded_loop!(b, datum, loop; z_front=-t + depth, z_back=-t, outward=false)
        _add_planar_face!(b, datum, loop; z=-t + depth, flip=true)
    end
    return nothing
end

function _add_annular_cap!(
    b::_MeshBuilder,
    outer_v::AbstractVector{Int},
    inner_v::AbstractVector{Int};
    flip::Bool=false,
)
    n = length(outer_v)
    for i in 1:n
        j = i == n ? 1 : i + 1
        vo1, vo2 = outer_v[i], outer_v[j]
        vi1, vi2 = inner_v[i], inner_v[j]
        if flip
            push!(b.faces, TriangleFace{Int}(vo1, vi2, vo2))
            push!(b.faces, TriangleFace{Int}(vo1, vi1, vi2))
        else
            push!(b.faces, TriangleFace{Int}(vo1, vo2, vi2))
            push!(b.faces, TriangleFace{Int}(vo1, vi2, vi1))
        end
    end
    return nothing
end

"""
    derive_mesh(part::RectangularTube) -> GeometryBasics.Mesh

Derive a closed, watertight 3D triangle mesh for `part` from its dimensions and
the `Saw`, `Drill`, and `Cut` operations in its manufacturing history.
"""
function derive_mesh(part::RectangularTube)
    b = _MeshBuilder()
    t = part.thickness
    for face in datums(part)
        w = _face_width(part, face)
        p_out_0 = face.transform(SVector(0.0, 0.0, 0.0))
        p_out_w = face.transform(SVector(w, 0.0, 0.0))
        zb_out_0, zt_out_0 = _tube_z_bounds(part, p_out_0[1], p_out_0[2])
        zb_out_w, zt_out_w = _tube_z_bounds(part, p_out_w[1], p_out_w[2])
        outer_wall = Point2d[
            Point2d(0.0, zb_out_0),
            Point2d(w,   zb_out_w),
            Point2d(w,   zt_out_w),
            Point2d(0.0, zt_out_0),
        ]

        p_in_0 = face.transform(SVector(t, 0.0, -t))
        p_in_w = face.transform(SVector(w - t, 0.0, -t))
        zb_in_0, zt_in_0 = _tube_z_bounds(part, p_in_0[1], p_in_0[2])
        zb_in_w, zt_in_w = _tube_z_bounds(part, p_in_w[1], p_in_w[2])
        inner_wall = Point2d[
            Point2d(t,     zb_in_0),
            Point2d(w - t, zb_in_w),
            Point2d(w - t, zt_in_w),
            Point2d(t,     zt_in_0),
        ]

        wall_holes = _face_holes(part, face, inner_wall)
        _add_slab!(b, face, outer_wall, inner_wall, wall_holes, t)
    end

    cap_outer = _rect_loop(0.0, 0.0, part.width, part.height)
    cap_inner = _rect_loop(t, t, part.width - t, part.height - t)
    v_ob = [_vertex!(b, SVector(p[1], p[2], _tube_z_bounds(part, p[1], p[2])[1])) for p in cap_outer]
    v_ot = [_vertex!(b, SVector(p[1], p[2], _tube_z_bounds(part, p[1], p[2])[2])) for p in cap_outer]
    v_ib = [_vertex!(b, SVector(p[1], p[2], _tube_z_bounds(part, p[1], p[2])[1])) for p in cap_inner]
    v_it = [_vertex!(b, SVector(p[1], p[2], _tube_z_bounds(part, p[1], p[2])[2])) for p in cap_inner]
    _add_annular_cap!(b, v_ob, v_ib; flip=true)
    _add_annular_cap!(b, v_ot, v_it; flip=false)

    return Mesh(b.points, b.faces)
end

"""
    derive_mesh(part::RoundTube) -> GeometryBasics.Mesh

Derive a closed, watertight 3D triangle mesh for `part` from its outer diameter,
wall thickness, and the `Saw` operations in its manufacturing history, with the
tube axis along `+Z` centered at `(x, y) = (0, 0)`.
"""
function derive_mesh(part::RoundTube)
    b = _MeshBuilder()
    outer = _contour(Circle(part.outer_diameter))
    inner = _contour(Circle(part.outer_diameter - 2.0 * part.thickness))
    n = length(outer)

    v_ob = [_vertex!(b, SVector(p[1], p[2], _tube_z_bounds(part, p[1], p[2])[1])) for p in outer]
    v_ot = [_vertex!(b, SVector(p[1], p[2], _tube_z_bounds(part, p[1], p[2])[2])) for p in outer]
    v_ib = [_vertex!(b, SVector(p[1], p[2], _tube_z_bounds(part, p[1], p[2])[1])) for p in inner]
    v_it = [_vertex!(b, SVector(p[1], p[2], _tube_z_bounds(part, p[1], p[2])[2])) for p in inner]

    for i in 1:n
        j = i == n ? 1 : i + 1
        push!(b.faces, TriangleFace{Int}(v_ot[i], v_ob[j], v_ot[j]))
        push!(b.faces, TriangleFace{Int}(v_ot[i], v_ob[i], v_ob[j]))
        push!(b.faces, TriangleFace{Int}(v_it[i], v_it[j], v_ib[j]))
        push!(b.faces, TriangleFace{Int}(v_it[i], v_ib[j], v_ib[i]))
    end

    _add_annular_cap!(b, v_ot, v_it; flip=false)
    _add_annular_cap!(b, v_ob, v_ib; flip=true)
    return Mesh(b.points, b.faces)
end

"""
    derive_mesh(part::Plate) -> GeometryBasics.Mesh

Derive a closed, watertight 3D triangle mesh for `part` from its dimensions and
the `Saw`, `Drill`, and `Cut` operations in its manufacturing history.
"""
function derive_mesh(part::Plate)
    b = _MeshBuilder()
    top = datum(part, SVector(0, 0, 1))
    outer_poly = _plate_polygon(part)
    shrunk_outer = LibGEOS.buffer(outer_poly, -_ATOL)
    outer = _from_geos_polygon(outer_poly)
    through_holes = Vector{Point2d}[]
    front_blind = Tuple{Vector{Point2d},Float64}[]
    back_blind = Tuple{Vector{Point2d},Float64}[]

    for op in operations(part)
        if op isa Drill
            loop = _project_hole(op, top)
            LibGEOS.contains(shrunk_outer, _to_geos_polygon(loop)) || continue
            if op.penetration isa Blind
                if dot(zaxis(op.datum), zaxis(top)) > 0.0
                    push!(front_blind, (loop, op.penetration.depth))
                else
                    push!(back_blind, (loop, op.penetration.depth))
                end
            else
                push!(through_holes, loop)
            end
        elseif op isa Cut
            loop = _project_hole(op, top)
            LibGEOS.contains(shrunk_outer, _to_geos_polygon(loop)) || continue
            push!(through_holes, loop)
        end
    end

    _add_slab!(
        b,
        top,
        outer,
        outer,
        through_holes,
        part.thickness;
        close_perimeter=true,
        front_blind=front_blind,
        back_blind=back_blind,
    )
    return Mesh(b.points, b.faces)
end

"""
    derive_mesh(part::Weldment) -> GeometryBasics.Mesh

Derive a 3D triangle mesh for `part` by combining `derive_mesh(part.parent[1])`
in `part`'s coordinate frame with `derive_mesh(part.parent[2])` transformed by
`_weld_transform(part.op)`.
"""
function derive_mesh(part::Weldment)
    pts = Point3d[]
    fcs = TriangleFace{Int}[]
    T_b = _weld_transform(part.op)
    for (subpart, transform) in (
        part.parent[1] => IdentityRigidTransform,
        part.parent[2] => T_b,
    )
        submesh = derive_mesh(subpart)
        offset = length(pts)
        for p in coordinates(submesh)
            tp = transform(SVector(p[1], p[2], p[3]))
            push!(pts, Point3d(tp[1], tp[2], tp[3]))
        end
        for f in faces(submesh)
            push!(
                fcs,
                TriangleFace{Int}(
                    Int(f[1]) + offset,
                    Int(f[2]) + offset,
                    Int(f[3]) + offset,
                ),
            )
        end
    end
    return Mesh(pts, fcs)
end

function _face_holes(
    part::AbstractPart,
    face::Datum,
    bounds_poly::AbstractVector{Point2d},
)
    result = Vector{Point2d}[]
    shrunk_bounds = LibGEOS.buffer(_to_geos_polygon(bounds_poly), -_ATOL)
    for op in operations_of_type(Union{Drill,Cut}, part)
        _pierces_face(op, face) || continue
        outline = _project_outline(op.profile, op.datum, face)
        LibGEOS.contains(shrunk_bounds, _to_geos_polygon(outline)) || continue
        push!(result, outline)
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
    hole_outlines::AbstractVector{<:AbstractVector{Point2d}}=Vector{Point2d}[],
)
    pts = Point2d[]
    segs = Tuple{Int32,Int32}[]

    function add_outline!(outline)
        base = Int32(length(pts))
        n = Int32(length(outline))
        append!(pts, outline)
        for i in Int32(1):n
            push!(segs, (base + i, base + (i == n ? Int32(1) : i + Int32(1))))
        end
    end

    add_outline!(outer)
    for outline in hole_outlines
        add_outline!(outline)
    end

    triin = TriangulateIO()
    triin.pointlist = [p[j] for j in 1:2, p in pts]
    triin.segmentlist = [s[j] for j in 1:2, s in segs]
    if !isempty(hole_outlines)
        seeds = [sum(outline) / length(outline) for outline in hole_outlines]
        triin.holelist = [c[j] for j in 1:2, c in seeds]
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
    hole_outlines::Vector{Vector{Point2d}}=Vector{Point2d}[];
    z::Real=0.0,
    flip::Bool=false,
)
    pts2d, tris = _triangulate_2d(outer, hole_outlines)
    zf = Float64(z)
    vmap = [_vertex!(b, datum.frame(SVector(p[1], p[2], zf))) for p in pts2d]
    for (i1, i2, i3) in tris
        if flip
            push!(b.faces, TriangleFace{Int}(vmap[i1], vmap[i3], vmap[i2]))
        else
            push!(b.faces, TriangleFace{Int}(vmap[i1], vmap[i2], vmap[i3]))
        end
    end
    return nothing
end

"""
    _stitch_rings!(b, ring1, ring2; flip=false)

Stitch two equal-length closed vertex-index rings `ring1` and `ring2` into `b`
as a quad strip of `2 * length(ring1)` triangles. Setting `flip=true` reverses
triangle winding to invert the surface normal.
"""
function _stitch_rings!(
    b::_MeshBuilder,
    ring1::AbstractVector{Int},
    ring2::AbstractVector{Int};
    flip::Bool=false,
)
    n = length(ring1)
    for i in 1:n
        j = i == n ? 1 : i + 1
        v1a, v1b = ring1[i], ring1[j]
        v2a, v2b = ring2[i], ring2[j]
        if flip
            push!(b.faces, TriangleFace{Int}(v1a, v2b, v1b))
            push!(b.faces, TriangleFace{Int}(v1a, v2a, v2b))
        else
            push!(b.faces, TriangleFace{Int}(v1a, v1b, v2b))
            push!(b.faces, TriangleFace{Int}(v1a, v2b, v2a))
        end
    end
    return nothing
end

"""
    _add_extruded_outline!(b, datum, outline; z_front=0.0, z_back, outward=false)

Extrude a closed 2D counter-clockwise `outline` in `datum`'s XY plane from `z_front`
to `z_back` and stitch the side wall into `b`. Normals point inward (for hole
barrels) when `outward=false` and outward (for outer perimeters) when `outward=true`.
"""
function _add_extruded_outline!(
    b::_MeshBuilder,
    datum::Datum,
    outline::AbstractVector{Point2d};
    z_front::Real=0.0,
    z_back::Real,
    outward::Bool=false,
)
    zf = Float64(z_front)
    zb = Float64(z_back)
    front_v = [_vertex!(b, datum.frame(SVector(p[1], p[2], zf))) for p in outline]
    back_v = [_vertex!(b, datum.frame(SVector(p[1], p[2], zb))) for p in outline]
    _stitch_rings!(b, front_v, back_v; flip=outward)
    return nothing
end

"""
    _add_slab!(b, datum, outer_front, outer_back, hole_outlines, thickness; close_perimeter=false)

Add a 2.5D perforated wall/slab of `thickness` along `-zaxis(datum)` to `b`:
triangulates the front face (`z = 0`) and back face (`z = -thickness`), stitches
internal hole barrels for `hole_outlines`, and optionally extrudes `outer_front`
around the perimeter when `close_perimeter=true`.
"""
function _add_slab!(
    b::_MeshBuilder,
    datum::Datum,
    outer_front::Vector{Point2d},
    outer_back::Vector{Point2d},
    hole_outlines::Vector{Vector{Point2d}},
    thickness::Real;
    close_perimeter::Bool=false,
    front_blind::Vector{Tuple{Vector{Point2d},Float64}}=Tuple{Vector{Point2d},Float64}[],
    back_blind::Vector{Tuple{Vector{Point2d},Float64}}=Tuple{Vector{Point2d},Float64}[],
)
    t = Float64(thickness)
    front_holes = isempty(front_blind) ? hole_outlines : vcat(hole_outlines, first.(front_blind))
    back_holes = isempty(back_blind) ? hole_outlines : vcat(hole_outlines, first.(back_blind))
    _add_planar_face!(b, datum, outer_front, front_holes; z=0.0, flip=false)
    _add_planar_face!(b, datum, outer_back, back_holes; z=-t, flip=true)
    if close_perimeter
        _add_extruded_outline!(b, datum, outer_front; z_front=0.0, z_back=-t, outward=true)
    end
    for outline in hole_outlines
        _add_extruded_outline!(b, datum, outline; z_front=0.0, z_back=-t, outward=false)
    end
    for (outline, depth) in front_blind
        _add_extruded_outline!(b, datum, outline; z_front=0.0, z_back=-depth, outward=false)
        _add_planar_face!(b, datum, outline; z=-depth, flip=false)
    end
    for (outline, depth) in back_blind
        _add_extruded_outline!(b, datum, outline; z_front=-t + depth, z_back=-t, outward=false)
        _add_planar_face!(b, datum, outline; z=-t + depth, flip=true)
    end
    return nothing
end

"""
    _tube_end_rings!(b, part, outline) -> Tuple{Vector{Int},Vector{Int}}

Register 3D vertices in `b` at the bottom and top sawn axial bounds of `part`
for each 2D cross-section point in `outline`, returning `(v_bot, v_top)`.
"""
function _tube_end_rings!(b::_MeshBuilder, part::AbstractTube, outline::AbstractVector{Point2d})
    n = length(outline)
    v_bot = Vector{Int}(undef, n)
    v_top = Vector{Int}(undef, n)
    for (i, p) in enumerate(outline)
        zb, zt = _tube_z_bounds(part, p[1], p[2])
        v_bot[i] = _vertex!(b, SVector(p[1], p[2], zb))
        v_top[i] = _vertex!(b, SVector(p[1], p[2], zt))
    end
    return v_bot, v_top
end

"""
    _add_tube_caps!(b, part, outer, inner) -> NTuple{4,Vector{Int}}

Add the bottom and top annular end caps of `part` between 2D cross-section outlines
`outer` and `inner` to `b`, and return their vertex-index rings `(v_ob, v_ot, v_ib, v_it)`.
"""
function _add_tube_caps!(
    b::_MeshBuilder,
    part::AbstractTube,
    outer::AbstractVector{Point2d},
    inner::AbstractVector{Point2d},
)
    v_ob, v_ot = _tube_end_rings!(b, part, outer)
    v_ib, v_it = _tube_end_rings!(b, part, inner)
    _stitch_rings!(b, v_ob, v_ib; flip=true)
    _stitch_rings!(b, v_ot, v_it; flip=false)
    return v_ob, v_ot, v_ib, v_it
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
        outer_wall = _tube_wall_outline(part, face)
        inner_wall = _tube_wall_outline(part, face, t)
        wall_holes = _face_holes(part, face, inner_wall)
        _add_slab!(b, face, outer_wall, inner_wall, wall_holes, t)
    end

    _add_tube_caps!(
        b,
        part,
        _rect_outline(0.0, 0.0, part.width, part.height),
        _rect_outline(t, t, part.width - t, part.height - t),
    )

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
    outer = _outline(Circle(part.outer_diameter))
    inner = _outline(Circle(part.outer_diameter - 2.0 * part.thickness))
    v_ob, v_ot, v_ib, v_it = _add_tube_caps!(b, part, outer, inner)
    _stitch_rings!(b, v_ot, v_ob; flip=true)
    _stitch_rings!(b, v_it, v_ib; flip=false)
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

    for op in operations_of_type(Union{Drill,Cut}, part)
        outline = _project_outline(op.profile, op.datum, top)
        LibGEOS.contains(shrunk_outer, _to_geos_polygon(outline)) || continue
        if op isa Drill && op.penetration isa Blind
            target = ispositive(dot(zaxis(op.datum), zaxis(top))) ? front_blind : back_blind
            push!(target, (outline, op.penetration.depth))
        else
            push!(through_holes, outline)
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

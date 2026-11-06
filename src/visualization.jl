function _flat_mesh(mesh::Mesh)
    pts = coordinates(mesh)
    fcs = faces(mesh)
    flat_pts = Vector{Point3d}(undef, 3 * length(fcs))
    flat_fcs = Vector{TriangleFace{Int}}(undef, length(fcs))
    for (k, f) in enumerate(fcs)
        base = 3 * (k - 1)
        flat_pts[base + 1] = pts[f[1]]
        flat_pts[base + 2] = pts[f[2]]
        flat_pts[base + 3] = pts[f[3]]
        flat_fcs[k] = TriangleFace{Int}(base + 1, base + 2, base + 3)
    end
    return Mesh(flat_pts, flat_fcs)
end

"""
    visualize(part::AbstractPart; vis::Visualizer=Visualizer(), open_browser::Bool=isinteractive()) -> Visualizer

Render `derive_mesh(part)` in a MeshCat `Visualizer` with brushed-metal styling,
centered at the scene origin and framed in the default camera.
"""
function visualize(
    part::AbstractPart;
    vis::Visualizer=Visualizer(),
    open_browser::Bool=isinteractive(),
)
    mesh = derive_mesh(part)
    pts = coordinates(mesh)
    xs = [p[1] for p in pts]
    ys = [p[2] for p in pts]
    zs = [p[3] for p in pts]
    lo = SVector(minimum(xs), minimum(ys), minimum(zs))
    hi = SVector(maximum(xs), maximum(ys), maximum(zs))
    center = 0.5 * (lo + hi)
    radius = max(0.5 * norm(hi - lo), 1.0)

    delete!(vis[:part])
    material = MeshPhongMaterial(color=MeshCat.RGBA(0.68, 0.72, 0.76, 1.0))
    setobject!(vis[:part], _flat_mesh(mesh), material)
    settransform!(vis[:part], Translation(-center))

    settransform!(vis["/Cameras/default"], LinearMap(RotZ(-π / 2)))

    open_browser && open(vis)
    return vis
end

"""
    visualize(
        g::ManufacturingGraph;
        path::AbstractString="manufacturing_graph.dot",
        open_viewer::Bool=isinteractive(),
    ) -> String

Write `g` as a Graphviz `.dot` file to `path` and view it with `dot -Tx11` when
Graphviz is installed (or print the DOT text as a terminal fallback).
"""
function visualize(
    g::ManufacturingGraph;
    path::AbstractString="manufacturing_graph.dot",
    open_viewer::Bool=isinteractive(),
)
    dot = to_dot(g)
    write(path, dot)
    dot_cmd = Sys.which("dot")
    if isnothing(dot_cmd)
        print(dot)
    elseif open_viewer
        run(`$dot_cmd -Tx11 $path`; wait=false)
    end
    return dot
end

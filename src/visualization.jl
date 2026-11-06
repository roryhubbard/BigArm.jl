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

    cam = vis["/Cameras/default/rotated/<object>"]
    setprop!(cam, "position", [1.2 * radius, 0.4 * radius, 1.6 * radius])
    setprop!(cam, "far", max(1000.0, 20.0 * radius))

    open_browser && open(vis)
    return vis
end

"""
    visualize(
        g::ManufacturingGraph;
        path::Union{Nothing,AbstractString}=nothing,
        open_browser::Bool=isinteractive(),
        dot_cmd::Union{Nothing,AbstractString}=Sys.which("dot"),
        io::IO=stdout,
    ) -> String

Render `g` to SVG via Graphviz `dot` when installed, or print and return the
DOT representation to `io` as a terminal fallback. Writes the output to `path`
when provided.
"""
function visualize(
    g::ManufacturingGraph;
    path::Union{Nothing,AbstractString}=nothing,
    open_browser::Bool=isinteractive(),
    dot_cmd::Union{Nothing,AbstractString}=Sys.which("dot"),
    io::IO=stdout,
)
    dot = to_dot(g)
    if isnothing(dot_cmd)
        !isnothing(path) && write(path, dot)
        print(io, dot)
        return dot
    end

    svg = read(pipeline(IOBuffer(dot), `$dot_cmd -Tsvg`), String)
    if !isnothing(path)
        write(path, svg)
    elseif open_browser
        svg_path = tempname() * ".svg"
        write(svg_path, svg)
        MeshCat.open_url(svg_path)
    end
    return svg
end

module BigArm

import Base: parent

using CoordinateTransformations: AffineMap, LinearMap, Translation
using GeometryBasics: Mesh, Point2d, Point3d, TriangleFace, coordinates, faces
using LinearAlgebra: dot, norm, normalize
using MeshCat: MeshCat, MeshPhongMaterial, Visualizer, delete!, setobject!, setprop!, settransform!
using Rotations: Rotation, RotX, RotZ
using StaticArrays: SVector
using Triangulate: TriangulateIO, triangulate

export AbstractPart,
    AbstractTube,
    AbstractProfile,
    AbstractDatum,
    AbstractPenetration,
    AbstractOperation,
    RigidTransform,
    RectangularTube,
    Plate,
    Circle,
    Datum,
    ThroughAll,
    ThroughWall,
    Blind,
    Saw,
    Drill,
    Assembly,
    ManufacturingGraph,
    operation,
    isstock,
    origin,
    xaxis,
    yaxis,
    zaxis,
    datums,
    datum,
    holes,
    saw,
    drill,
    operations,
    to_dot,
    derive_mesh,
    visualize

include("primitives.jl")
include("graph.jl")
include("geometry.jl")
include("visualization.jl")

end

module BigArm

import Base: parent

using CoordinateTransformations: AffineMap, LinearMap, Translation
using GeometryBasics: Mesh, Point2d, Point3d, TriangleFace, coordinates, faces
using LibGEOS: LibGEOS
using LinearAlgebra: dot, norm, normalize
using MeshCat: MeshCat, MeshPhongMaterial, Visualizer, delete!, setobject!, setprop!, settransform!
using Rotations: Rotation, RotMatrix3, RotX, RotZ
using StaticArrays: SVector
using Triangulate: TriangulateIO, triangulate

export AbstractPart,
    AbstractTube,
    AbstractProfile,
    AbstractDatum,
    AbstractPenetration,
    AbstractOperation,
    RigidTransform,
    IdentityRigidTransform,
    RectangularTube,
    RoundTube,
    Plate,
    Weldment,
    Circle,
    Polygon,
    Rectangle,
    Datum,
    ThroughAll,
    ThroughWall,
    Blind,
    Saw,
    Drill,
    Cut,
    Weld,
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
    saw,
    drill,
    cut,
    weld,
    operations,
    operations_of_type,
    to_dot,
    derive_mesh,
    visualize

include("primitives.jl")
include("graph.jl")
include("geometry.jl")
include("visualization.jl")

end

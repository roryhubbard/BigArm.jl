module BigArm

import Base: parent

using CoordinateTransformations: AffineMap, LinearMap, Translation
using GeometryBasics: Mesh, Point3d, TriangleFace
using LinearAlgebra: dot, norm, normalize
using Rotations: Rotation, RotX, RotZ
using StaticArrays: SVector
using Triangulate: TriangulateIO, triangulate

export AbstractPart,
    AbstractTube,
    AbstractProfile,
    AbstractDatum,
    AbstractPenetration,
    AbstractOperation,
    RectangularTube,
    Circle,
    Datum,
    ThroughAll,
    ThroughWall,
    Blind,
    Saw,
    Drill,
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
    derive_mesh

include("primitives.jl")
include("graph.jl")
include("geometry.jl")

end

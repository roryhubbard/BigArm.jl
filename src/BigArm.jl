module BigArm

using CoordinateTransformations: AffineMap, LinearMap, Translation
using LinearAlgebra: dot, norm, normalize
using Rotations: Rotation, RotX, RotZ
using StaticArrays: SVector

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
    isstock,
    origin,
    xaxis,
    yaxis,
    zaxis,
    datums,
    datum,
    holes,
    saw,
    drill

include("primitives.jl")

end

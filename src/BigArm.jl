module BigArm

using CoordinateTransformations: AffineMap, LinearMap, Transformation, Translation
using LinearAlgebra: dot, norm
using Rotations: RotMatrix3, RotX, RotY, RotZ, isrotation
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
    x_axis,
    y_axis,
    z_axis,
    translate,
    local_coords,
    datums,
    datum,
    front,
    back,
    right,
    left,
    top,
    bottom,
    holes,
    pierced_walls,
    saw,
    drill

include("primitives.jl")

end

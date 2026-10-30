module BigArm

export AbstractPart,
    AbstractTube,
    AbstractProfile,
    AbstractDatum,
    AbstractPenetration,
    AbstractOperation,
    RectangularTube,
    Circle,
    FaceDatum,
    ThroughAll,
    ThroughWall,
    Blind,
    Saw,
    Drill,
    isstock,
    holes,
    pierced_faces,
    saw,
    drill

include("primitives.jl")

end

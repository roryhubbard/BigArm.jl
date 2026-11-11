const _ATOL = 1e-9

"""
    AbstractDatum

Supertype for coordinate frames on a part.
"""
abstract type AbstractDatum end

const RigidTransform = Union{
    AffineMap{<:Rotation{3},<:SVector{3,<:Real}},
    Translation{<:SVector{3,<:Real}},
    LinearMap{<:Rotation{3}},
}

const IdentityRigidTransform = AffineMap(one(RotMatrix3), zero(SVector{3}))

"""
    Datum(frame::RigidTransform=IdentityRigidTransform)

Right-handed 3D Cartesian coordinate frame (`SE(3)` rigid transformation relative
to the part's 3D origin):
- `origin(datum)` is the 3D origin in part coordinates,
- `xaxis(datum)` and `yaxis(datum)` are the orthonormal 2D axes on the datum surface,
- `zaxis(datum)` is the outward surface normal (tools approach along `-zaxis(datum)`).
"""
struct Datum <: AbstractDatum
    frame::RigidTransform

    function Datum(frame::RigidTransform=IdentityRigidTransform)
        return new(IdentityRigidTransform ∘ frame)
    end
end

"""
    CylindricalDatum(frame::RigidTransform=IdentityRigidTransform)

Right-handed 3D cylindrical coordinate frame (`SE(3)` rigid transformation
relative to the part's 3D origin) whose `zaxis` is the cylinder axis and whose
origin lies on the cylinder centerline (`r = 0`). Used to constrain dispatch to
cylindrical/polar operations and profiles (such as `Circle` welds on `RoundTube`).
"""
struct CylindricalDatum <: AbstractDatum
    frame::RigidTransform

    function CylindricalDatum(frame::RigidTransform=IdentityRigidTransform)
        return new(IdentityRigidTransform ∘ frame)
    end
end

_transform_datum(T::RigidTransform, datum::D) where {D<:AbstractDatum} = D(T ∘ datum.frame)

origin(datum::AbstractDatum) = datum.frame.translation
xaxis(datum::AbstractDatum) = datum.frame.linear[:, 1]
yaxis(datum::AbstractDatum) = datum.frame.linear[:, 2]
zaxis(datum::AbstractDatum) = datum.frame.linear[:, 3]

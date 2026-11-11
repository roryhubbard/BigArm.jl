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
    Datum(transform::RigidTransform=IdentityRigidTransform)

Right-handed 3D coordinate frame (`SE(3)` rigid transformation relative to the
part's 3D origin):
- `origin(datum)` is the 3D origin in part coordinates,
- `xaxis(datum)` and `yaxis(datum)` are the orthonormal 2D axes on the datum surface,
- `zaxis(datum)` is the outward surface normal (tools approach along `-zaxis(datum)`).
"""
struct Datum <: AbstractDatum
    transform::AffineMap{<:Rotation{3},SVector{3,Float64}}

    function Datum(transform::RigidTransform=IdentityRigidTransform)
        return new(IdentityRigidTransform ∘ transform)
    end
end

origin(datum::Datum) = datum.transform.translation
xaxis(datum::Datum) = datum.transform.linear[:, 1]
yaxis(datum::Datum) = datum.transform.linear[:, 2]
zaxis(datum::Datum) = datum.transform.linear[:, 3]

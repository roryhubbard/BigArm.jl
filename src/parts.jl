"""
    AbstractPart

Supertype for the workpiece in the machine (nodes in a manufacturing graph).

Concrete parts carry `parent` and `op` fields accessed via `parent(part)` and
`operation(part)`. Both are `nothing` for raw stock, and both are present when
a shop operation produced the part. `parent` extends `Base.parent`.
"""
abstract type AbstractPart end

"""
    AbstractTube <: AbstractPart

Supertype for tubular parts.
"""
abstract type AbstractTube <: AbstractPart end

"""
    AbstractOperation

Supertype for shop actions applied to parts.
"""
abstract type AbstractOperation end

"""
    RectangularTube(width, height, length, thickness, parent=nothing, op=nothing)

Rectangular-tube part node with outer cross-section `width` (X) and `height` (Y),
tube `length` (Z), and wall `thickness`, all in inches (`Float64`).

A root `RectangularTube` (`parent === nothing` and `op === nothing`) represents
raw stock. Enforces positive dimensions and `2 * thickness < min(width, height)`.
"""
struct RectangularTube <: AbstractTube
    width::Float64
    height::Float64
    length::Float64
    thickness::Float64
    parent::Union{Nothing,AbstractPart}
    op::Union{Nothing,AbstractOperation}

    function RectangularTube(
        width::Real,
        height::Real,
        length::Real,
        thickness::Real,
        parent::Union{Nothing,AbstractPart}=nothing,
        op::Union{Nothing,AbstractOperation}=nothing,
    )
        w = Float64(width)
        h = Float64(height)
        l = Float64(length)
        t = Float64(thickness)
        w > 0.0 || throw(DomainError(width, "RectangularTube width must be positive"))
        h > 0.0 || throw(DomainError(height, "RectangularTube height must be positive"))
        l > 0.0 || throw(DomainError(length, "RectangularTube length must be positive"))
        t > 0.0 || throw(DomainError(thickness, "RectangularTube thickness must be positive"))
        2.0 * t < min(w, h) || throw(
            ArgumentError("RectangularTube thickness ($t) must be strictly less than half of width ($w) and height ($h)"),
        )
        isnothing(parent) == isnothing(op) || throw(
            ArgumentError("RectangularTube parent and op must either both be nothing (raw stock) or both be present"),
        )
        return new(w, h, l, t, parent, op)
    end
end

"""
    RoundTube(outer_diameter, length, thickness, parent=nothing, op=nothing)

Round-tube part node with outer cross-section `outer_diameter`, tube `length` (Z),
and wall `thickness`, all in inches (`Float64`).

A root `RoundTube` (`parent === nothing` and `op === nothing`) represents raw
stock. Enforces positive dimensions and `2 * thickness < outer_diameter`.
"""
struct RoundTube <: AbstractTube
    outer_diameter::Float64
    length::Float64
    thickness::Float64
    parent::Union{Nothing,AbstractPart}
    op::Union{Nothing,AbstractOperation}

    function RoundTube(
        outer_diameter::Real,
        length::Real,
        thickness::Real,
        parent::Union{Nothing,AbstractPart}=nothing,
        op::Union{Nothing,AbstractOperation}=nothing,
    )
        od = Float64(outer_diameter)
        l = Float64(length)
        t = Float64(thickness)
        od > 0.0 || throw(DomainError(outer_diameter, "RoundTube outer_diameter must be positive"))
        l > 0.0 || throw(DomainError(length, "RoundTube length must be positive"))
        t > 0.0 || throw(DomainError(thickness, "RoundTube thickness must be positive"))
        2.0 * t < od || throw(
            ArgumentError("RoundTube thickness ($t) must be strictly less than half of outer_diameter ($od)"),
        )
        isnothing(parent) == isnothing(op) || throw(
            ArgumentError("RoundTube parent and op must either both be nothing (raw stock) or both be present"),
        )
        return new(od, l, t, parent, op)
    end
end

"""
    Plate(width, height, thickness, parent=nothing, op=nothing)

Flat rectangular plate part node with dimensions `width` (X), `height` (Y), and
`thickness` (Z), all in inches (`Float64`).

A root `Plate` (`parent === nothing` and `op === nothing`) represents raw stock.
Enforces positive dimensions.
"""
struct Plate <: AbstractPart
    width::Float64
    height::Float64
    thickness::Float64
    parent::Union{Nothing,AbstractPart}
    op::Union{Nothing,AbstractOperation}

    function Plate(
        width::Real,
        height::Real,
        thickness::Real,
        parent::Union{Nothing,AbstractPart}=nothing,
        op::Union{Nothing,AbstractOperation}=nothing,
    )
        w = Float64(width)
        h = Float64(height)
        t = Float64(thickness)
        w > 0.0 || throw(DomainError(width, "Plate width must be positive"))
        h > 0.0 || throw(DomainError(height, "Plate height must be positive"))
        t > 0.0 || throw(DomainError(thickness, "Plate thickness must be positive"))
        isnothing(parent) == isnothing(op) || throw(
            ArgumentError("Plate parent and op must either both be nothing (raw stock) or both be present"),
        )
        return new(w, h, t, parent, op)
    end
end

"""
    parent(part::AbstractPart) -> Union{Nothing,AbstractPart,Pair{AbstractPart,AbstractPart}}

Return the part(s) `part` was made from: `nothing` for raw stock, the single
input `AbstractPart` for operations such as `saw`, `drill`, and `cut`, or the
`Pair{AbstractPart,AbstractPart}` of input parts for a `Weldment`.
"""
parent(part::AbstractPart) = part.parent

"""
    operation(part::AbstractPart) -> Union{Nothing,AbstractOperation}

Return the shop operation that produced `part`, or `nothing` for raw stock.
"""
operation(part::AbstractPart) = part.op

"""
    isstock(part::AbstractPart) -> Bool

Return `true` if `part` is raw mill stock (`parent(part) === nothing`).
"""
isstock(part::AbstractPart) = isnothing(parent(part))

"""
    datums(part::RectangularTube) -> NTuple{4,Datum}

Return the four outer face `Datum` frames of a `RectangularTube` (front `-Y`,
right `+X`, back `+Y`, left `-X`), each with `(0, 0)` at its bottom-left corner
when looking straight at the face (`-zaxis`) with `+Z` pointing up.
"""
function datums(part::RectangularTube)
    w = part.width
    h = part.height
    return (
        Datum(LinearMap(RotX(π / 2))),                                              # front (-Y)
        Datum(Translation(w,   0.0, 0.0) ∘ LinearMap(RotZ(π / 2) * RotX(π / 2))),   # right (+X)
        Datum(Translation(w,   h,   0.0) ∘ LinearMap(RotZ(π) * RotX(π / 2))),       # back  (+Y)
        Datum(Translation(0.0, h,   0.0) ∘ LinearMap(RotZ(-π / 2) * RotX(π / 2))),  # left  (-X)
    )
end

"""
    datums(part::Plate) -> NTuple{2,Datum}

Return the two flat surface `Datum` frames of a `Plate` (top `+Z` at
`z = thickness` and bottom `-Z` at `z = 0`), each with `(0, 0)` at its
bottom-left corner when looking straight at the face (`-zaxis`) with `+Y`
pointing up.
"""
function datums(part::Plate)
    w = part.width
    t = part.thickness
    return (
        Datum(Translation(0.0, 0.0, t)),                                            # top    (+Z)
        Datum(Translation(w,   0.0, 0.0) ∘ LinearMap(RotZ(π) * RotX(π))),           # bottom (-Z)
    )
end

"""
    datum(part::AbstractPart, normal) -> Datum

Select the `Datum` in `datums(part)` whose outward normal `zaxis(datum)` matches
`normal`.
"""
function datum(part::AbstractPart, normal::SVector{3,<:Real})
    n = normalize(normal)
    for datum in datums(part)
        if dot(zaxis(datum), n) > 1.0 - _ATOL
            return datum
        end
    end
    throw(ArgumentError("Part $(typeof(part)) has no datum with normal $normal"))
end

"""
    Weldment(parent::Pair{<:AbstractPart,<:AbstractPart}, op::AbstractOperation)

Welded `AbstractPart` node produced from two input parts `parent` (`part_a => part_b`)
by `op::Weld`.
"""
struct Weldment <: AbstractPart
    parent::Pair{AbstractPart,AbstractPart}
    op::AbstractOperation
end

"""
    Assembly(parts)

Multi-part 3D assembly positioning each `AbstractPart` in a shared right-handed
3D coordinate frame via a `Pair` `part => transform`, where `transform` is a
`RigidTransform`.
"""
struct Assembly
    parts::Vector{Pair{AbstractPart,RigidTransform}}

    function Assembly(parts::AbstractVector{<:Pair})
        isempty(parts) && throw(ArgumentError("Assembly must contain at least one part"))
        return new(Pair{AbstractPart,RigidTransform}[p.first => p.second for p in parts])
    end
end

Base.length(assembly::Assembly) = length(assembly.parts)
Base.iterate(assembly::Assembly, state...) = iterate(assembly.parts, state...)
Base.getindex(assembly::Assembly, i::Integer) = assembly.parts[i]

function Base.show(io::IO, assembly::Assembly)
    print(io, "Assembly(", length(assembly.parts), " parts)")
    return nothing
end

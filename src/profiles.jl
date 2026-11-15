const _CIRCLE_SEGMENTS = 32

"""
    AbstractProfile

Supertype for pure 2D shapes cut in a datum's XY plane.
"""
abstract type AbstractProfile end

"""
    LineSegment(length)

Linear 1D profile with positive `length` (inches, `Float64`).
"""
struct LineSegment <: AbstractProfile
    length::Float64

    function LineSegment(length::Real)
        l = Float64(length)
        ispositive(l) || throw(DomainError(length, "LineSegment length must be positive"))
        return new(l)
    end
end

"""
    Circle(diameter)

Circular 2D cut profile with positive `diameter` (inches, `Float64`).
"""
struct Circle <: AbstractProfile
    diameter::Float64

    function Circle(diameter::Real)
        d = Float64(diameter)
        ispositive(d) || throw(DomainError(diameter, "Circle diameter must be positive"))
        return new(d)
    end
end

"""
    CircularSegment(chord, height)

Circular-segment 2D cut profile in a `Datum`'s XY plane with positive `chord`
along the datum X-axis (`x ∈ [-chord / 2, chord / 2]`, `y = 0`) and positive
arc `height` into the `+Y` half-plane (apex at `(0, height)`), in inches
(`Float64`). When `height == chord / 2`, the segment is a semicircle.
"""
struct CircularSegment <: AbstractProfile
    chord::Float64
    height::Float64

    function CircularSegment(chord::Real, height::Real)
        c = Float64(chord)
        h = Float64(height)
        ispositive(c) || throw(DomainError(chord, "CircularSegment chord must be positive"))
        ispositive(h) || throw(DomainError(height, "CircularSegment height must be positive"))
        return new(c, h)
    end
end

function _rect_outline(x0::Real, y0::Real, x1::Real, y1::Real)
    return Point2d[
        Point2d(Float64(x0), Float64(y0)),
        Point2d(Float64(x1), Float64(y0)),
        Point2d(Float64(x1), Float64(y1)),
        Point2d(Float64(x0), Float64(y1)),
    ]
end

function _to_geos_polygon(outline::AbstractVector{Point2d})
    coords = Vector{Float64}[[p[1], p[2]] for p in outline]
    push!(coords, [outline[1][1], outline[1][2]])
    return LibGEOS.Polygon([coords])
end

_is_ccw(poly::LibGEOS.Polygon) =
    GC.@preserve poly LibGEOS.isCCW(LibGEOS.getCoordSeq(LibGEOS.exteriorRing(poly)))

function _from_geos_polygon(poly::LibGEOS.Polygon)
    return GC.@preserve poly begin
        seq = LibGEOS.getCoordSeq(LibGEOS.exteriorRing(poly))
        coords = LibGEOS.getCoordinates(seq)
        pts = Point2d[Point2d(c[1], c[2]) for c in @view coords[1:(end - 1)]]
        LibGEOS.isCCW(seq) || reverse!(pts)
        pts
    end
end

"""
    Polygon(points)

Polygonal 2D cut profile in a `Datum`'s XY plane defined by at least 3
non-collinear vertices (inches, `Float64`), normalized to counter-clockwise
winding order.
"""
struct Polygon <: AbstractProfile
    points::Vector{Point2d}

    function Polygon(points::AbstractVector{<:AbstractVector{<:Real}})
        length(points) >= 3 || throw(ArgumentError("Polygon must have at least 3 vertices"))
        pts = Point2d[Point2d(Float64(p[1]), Float64(p[2])) for p in points]
        poly = _to_geos_polygon(pts)
        LibGEOS.area(poly) > _ATOL || throw(ArgumentError("Polygon vertices must not be collinear"))
        _is_ccw(poly) || reverse!(pts)
        return new(pts)
    end
end

"""
    Rectangle(width, height)

Centered rectangular 2D cut profile in a `Datum`'s XY plane with positive
`width` (X) and `height` (Y), in inches (`Float64`).
"""
struct Rectangle <: AbstractProfile
    width::Float64
    height::Float64

    function Rectangle(width::Real, height::Real)
        w = Float64(width)
        h = Float64(height)
        ispositive(w) || throw(DomainError(width, "Rectangle width must be positive"))
        ispositive(h) || throw(DomainError(height, "Rectangle height must be positive"))
        return new(w, h)
    end
end

"""
    _outline(profile::AbstractProfile) -> Vector{Point2d}

Return a counter-clockwise 2D polygon outline approximating `profile` in its
local datum XY plane.
"""
_outline(profile::LineSegment) = Point2d[Point2d(0.0, 0.0), Point2d(profile.length, 0.0)]

function _outline(profile::Circle; segments::Int=_CIRCLE_SEGMENTS)
    r = profile.diameter / 2.0
    return Point2d[
        Point2d(r * cospi(2 * (i - 1) / segments), r * sinpi(2 * (i - 1) / segments))
        for i in 1:segments
    ]
end

function _outline(profile::CircularSegment; segments::Int=_CIRCLE_SEGMENTS ÷ 2)
    a = profile.chord / 2.0
    h = profile.height
    r = (a^2 + h^2) / (2.0 * h)
    yc = h - r
    ϕ_max = 2.0 * atan(h, a)
    pts = Vector{Point2d}(undef, segments + 1)
    pts[1] = Point2d(a, 0.0)
    for i in 1:(segments - 1)
        ϕ = ϕ_max * (2.0 * i / segments - 1.0)
        pts[i + 1] = Point2d(-r * sin(ϕ), yc + r * cos(ϕ))
    end
    pts[segments + 1] = Point2d(-a, 0.0)
    return pts
end

_outline(profile::Rectangle) =
    _rect_outline(-profile.width / 2.0, -profile.height / 2.0, profile.width / 2.0, profile.height / 2.0)

_outline(profile::Polygon) = profile.points

"""
    _project_outline(profile::AbstractProfile, from::Datum, onto::Datum=Datum()) -> Vector{Point2d}

Project `_outline(profile)` from `from`'s local XY plane onto `onto`'s XY plane
(via `inv(onto.frame) ∘ from.frame`) and normalize to counter-clockwise
2D winding order.
"""
function _project_outline(profile::AbstractProfile, from::Datum, onto::Datum=Datum())
    T_onto_from = inv(onto.frame) ∘ from.frame
    outline = Point2d[Point2d(T_onto_from(SVector(u[1], u[2], 0.0))[1:2]) for u in _outline(profile)]
    _is_ccw(_to_geos_polygon(outline)) || reverse!(outline)
    return outline
end

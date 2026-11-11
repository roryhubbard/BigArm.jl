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
        l > 0.0 || throw(DomainError(length, "LineSegment length must be positive"))
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
        d > 0.0 || throw(DomainError(diameter, "Circle diameter must be positive"))
        return new(d)
    end
end

function _rect_loop(x0::Real, y0::Real, x1::Real, y1::Real)
    return Point2d[
        Point2d(Float64(x0), Float64(y0)),
        Point2d(Float64(x1), Float64(y0)),
        Point2d(Float64(x1), Float64(y1)),
        Point2d(Float64(x0), Float64(y1)),
    ]
end

function _to_geos_polygon(loop::AbstractVector{Point2d})
    coords = Vector{Float64}[[p[1], p[2]] for p in loop]
    push!(coords, [loop[1][1], loop[1][2]])
    return LibGEOS.Polygon([coords])
end

_is_ccw(poly::LibGEOS.Polygon) =
    LibGEOS.isCCW(LibGEOS.getCoordSeq(LibGEOS.exteriorRing(poly)))

function _from_geos_polygon(poly::LibGEOS.Polygon)
    seq = LibGEOS.getCoordSeq(LibGEOS.exteriorRing(poly))
    coords = LibGEOS.getCoordinates(seq)
    pts = Point2d[Point2d(c[1], c[2]) for c in @view coords[1:(end - 1)]]
    LibGEOS.isCCW(seq) || reverse!(pts)
    return pts
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
    Rectangle(width, height) -> Polygon

Convenience constructor returning a centered `width × height` rectangular
`Polygon` profile in a `Datum`'s XY plane.
"""
function Rectangle(width::Real, height::Real)
    w = Float64(width)
    h = Float64(height)
    w > 0.0 || throw(DomainError(width, "Rectangle width must be positive"))
    h > 0.0 || throw(DomainError(height, "Rectangle height must be positive"))
    hw = w / 2.0
    hh = h / 2.0
    return Polygon(Point2d[
        Point2d(-hw, -hh),
        Point2d( hw, -hh),
        Point2d( hw,  hh),
        Point2d(-hw,  hh),
    ])
end

"""
    _contour(profile::AbstractProfile) -> Vector{Point2d}

Return a counter-clockwise 2D polygon loop approximating `profile` in its local
datum XY plane.
"""
function _contour(profile::Circle; segments::Int=_CIRCLE_SEGMENTS)
    r = profile.diameter / 2.0
    return Point2d[
        Point2d(r * cos(2π * (i - 1) / segments), r * sin(2π * (i - 1) / segments))
        for i in 1:segments
    ]
end

_contour(profile::Polygon) = profile.points

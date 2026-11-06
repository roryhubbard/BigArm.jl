using BigArm
using CoordinateTransformations: Translation
using StaticArrays: SVector

"""
    UprightSpec(; kwargs...)

Parameters for a single schematic Rogue SM-2.5 squat stand upright (inches).
Published values from `DIMENSIONS.md` are distinguished from placeholder assumptions.
"""
Base.@kwdef struct UprightSpec
    # Published SM-2.5 specs (DIMENSIONS.md)
    width::Float64 = 3.0           # Published: 3×3 in upright section
    height::Float64 = 3.0          # Published: 3×3 in upright section
    thickness::Float64 = 0.120     # Published: 11-gauge steel wall (≈ 0.120 in)
    length::Float64 = 103.0        # Published: 103 in nominal upright height
    hole_spacing::Float64 = 2.0    # Published: 2 in on-center hole spacing

    # Assumed values (not published; to be verified by measurement)
    stock_length::Float64 = 288.0  # Assumed: standard 24 ft (288 in) raw mill stick
    hole_diameter::Float64 = 1.0   # Assumed: nominal 1.0 in cut for 1 in hardware class
    first_hole_y::Float64 = 2.0    # Assumed: first hole center 2.0 in from bottom end
    hole_count::Int = 50           # Assumed: 50 holes spanning y = 2.0:2.0:100.0 in
end

"""
    build_upright(spec::UprightSpec=UprightSpec()) -> RectangularTube

Build the SM-2.5 upright manufacturing graph from raw mill stock through cutoff
and cross-drilling operations.
"""
function build_upright(spec::UprightSpec=UprightSpec())
    stick = RectangularTube(spec.width, spec.height, spec.stock_length, spec.thickness)
    blank = saw(stick, spec.length)

    front = datum(blank, SVector(0, -1, 0))
    hole = Circle(spec.hole_diameter)
    center_x = spec.width / 2.0

    upright = blank
    for i in 0:(spec.hole_count - 1)
        y = spec.first_hole_y + i * spec.hole_spacing
        d = Datum(front.transform ∘ Translation(center_x, y, 0.0))
        upright = drill(upright, hole, d, ThroughAll())
    end
    return upright
end

visualize_upright(spec::UprightSpec=UprightSpec()) = visualize(build_upright(spec))

visualize_upright_graph(spec::UprightSpec=UprightSpec()) =
    visualize(ManufacturingGraph(build_upright(spec)))

using BigArm
using CoordinateTransformations: LinearMap, Translation
using Rotations: RotX, RotY
using StaticArrays: SVector

isdefined(@__MODULE__, :UprightSpec) || include(joinpath(@__DIR__, "upright.jl"))

"""
    MountingPlateSpec(; kwargs...)

Parameters for a schematic GHD side mounting plate (inches).
Published values from `DIMENSIONS.md` are distinguished from placeholder assumptions.
"""
Base.@kwdef struct MountingPlateSpec
    # Published Echo GHD / SM-2.5 specs (DIMENSIONS.md)
    thickness::Float64 = 0.1875                # Published: 3/16 in laser-cut steel mounting plate

    # Assumed values (not published; to be verified by measurement)
    width::Float64 = 12.0                      # Assumed: 12 in horizontal plate span (X)
    height::Float64 = 8.0                      # Assumed: 8 in vertical plate span (Y)
    hole_diameter::Float64 = 1.0               # Assumed: 1.0 in hardware holes
    upright_hole_x::Float64 = 1.5              # Assumed: centered on 3 in upright width
    upright_hole_ys::Tuple{Float64,Float64} = (2.0, 6.0)  # Assumed: 4 in (two 2 in spacings) apart
    arm_hole_xs::Tuple{Float64,Float64} = (6.0, 10.0)     # Assumed: two bolts along horizontal arm
    arm_hole_y::Float64 = 4.0                  # Assumed: centered vertically on plate
end

"""
    ArmSpec(; kwargs...)

Parameters for the horizontal GHD tube arm (inches).
Published values from `DIMENSIONS.md` are distinguished from placeholder assumptions.
"""
Base.@kwdef struct ArmSpec
    # Published specs (DIMENSIONS.md)
    width::Float64 = 3.0                       # Published: 3×3 in tube section
    height::Float64 = 3.0                      # Published: 3×3 in tube section
    thickness::Float64 = 0.120                 # Published: 11-gauge steel tubing (≈ 0.120 in)

    # Assumed values (not published; to be verified by measurement)
    stock_length::Float64 = 288.0              # Assumed: standard 24 ft (288 in) raw mill stick
    length::Float64 = 24.0                     # Assumed: 24 in horizontal arm length
    hole_diameter::Float64 = 1.0               # Assumed: 1.0 in mounting hardware class
    hole_ys::Tuple{Float64,Float64} = (3.0, 7.0)  # Assumed: matches plate arm_hole_xs from upright face
end

"""
    build_mounting_plate(spec::MountingPlateSpec=MountingPlateSpec()) -> Plate

Build a GHD side mounting plate from raw flat plate stock by drilling the
upright-mounting and arm-mounting through-holes.
"""
function build_mounting_plate(spec::MountingPlateSpec=MountingPlateSpec())
    plate = Plate(spec.width, spec.height, spec.thickness)
    top = datum(plate, SVector(0, 0, 1))
    hole = Circle(spec.hole_diameter)

    for y in spec.upright_hole_ys
        d = Datum(top.transform ∘ Translation(spec.upright_hole_x, y, 0.0))
        plate = drill(plate, hole, d, ThroughAll())
    end
    for x in spec.arm_hole_xs
        d = Datum(top.transform ∘ Translation(x, spec.arm_hole_y, 0.0))
        plate = drill(plate, hole, d, ThroughAll())
    end
    return plate
end

"""
    build_arm(spec::ArmSpec=ArmSpec()) -> RectangularTube

Build the horizontal GHD arm tube from raw mill stock through cutoff and
cross-drilling operations.
"""
function build_arm(spec::ArmSpec=ArmSpec())
    stick = RectangularTube(spec.width, spec.height, spec.stock_length, spec.thickness)
    blank = saw(stick, Datum(Translation(0.0, 0.0, spec.length)))
    front = datum(blank, SVector(0, -1, 0))
    hole = Circle(spec.hole_diameter)
    center_x = spec.width / 2.0

    arm = blank
    for y in spec.hole_ys
        d = Datum(front.transform ∘ Translation(center_x, y, 0.0))
        arm = drill(arm, hole, d, ThroughAll())
    end
    return arm
end

"""
    build_ghd_assembly(;
        upright_spec::UprightSpec=UprightSpec(),
        plate_spec::MountingPlateSpec=MountingPlateSpec(),
        arm_spec::ArmSpec=ArmSpec(),
        mount_hole_index::Int=19,
    ) -> Assembly

Assemble the SM-2.5 upright, left and right GHD side mounting plates, and
horizontal arm tube in a shared 3D coordinate frame with `mount_hole_index`
selecting the lower mating upright hole.
"""
function build_ghd_assembly(;
    upright_spec::UprightSpec=UprightSpec(),
    plate_spec::MountingPlateSpec=MountingPlateSpec(),
    arm_spec::ArmSpec=ArmSpec(),
    mount_hole_index::Int=19,
)
    upright = build_upright(upright_spec)
    plate = build_mounting_plate(plate_spec)
    arm = build_arm(arm_spec)

    lower_hole_z = upright_spec.first_hole_y + (mount_hole_index - 1) * upright_spec.hole_spacing
    mount_z = lower_hole_z - plate_spec.upright_hole_ys[1]
    arm_top_z = mount_z + plate_spec.arm_hole_y + arm_spec.width / 2.0

    t_upright = IdentityRigidTransform
    t_left_plate = Translation(0.0, 0.0, mount_z) ∘ LinearMap(RotX(π / 2))
    t_right_plate =
        Translation(0.0, upright_spec.height + plate_spec.thickness, mount_z) ∘
        LinearMap(RotX(π / 2))
    t_arm = Translation(upright_spec.width, 0.0, arm_top_z) ∘ LinearMap(RotY(π / 2))

    return Assembly([
        upright => t_upright,
        plate => t_left_plate,
        plate => t_right_plate,
        arm => t_arm,
    ])
end

using BigArm
using CoordinateTransformations: LinearMap, Translation
using Rotations: RotX, RotY, RotZ
using StaticArrays: SVector

isdefined(@__MODULE__, :UprightSpec) || include(joinpath(@__DIR__, "upright.jl"))

"""
    EchoGHDAttachmentSpec(; kwargs...)

Parameters for the Design B welded Echo GHD footplate and quad-roller attachment
(`DIMENSIONS.md` §2–§3), adapted to the SM-2.5 upright's `1.0 in` holes.
"""
Base.@kwdef struct EchoGHDAttachmentSpec
    # Footplate (DIMENSIONS.md §2–§3)
    footplate_width::Float64 = 24.0            # Photo-estimated: 24.0 in wide
    footplate_height::Float64 = 14.0           # Photo-estimated: 14.0 in tall
    footplate_thickness::Float64 = 0.1875      # Published: 3/16 in steel plate
    window_width::Float64 = 9.0                # Photo-estimated: 9.0 in center window width
    window_height::Float64 = 6.5               # Photo-estimated: 6.5 in center window height (ears & web overlap top/bottom bridges)

    # Upright side mounting ears (2×)
    ear_width::Float64 = 2.5                   # Photo-estimated: 2.5 in deep
    ear_height::Float64 = 8.5                  # Photo-estimated: 8.5 in tall
    ear_thickness::Float64 = 0.1875            # Published: 3/16 in laser-cut steel
    ear_spacing::Float64 = 3.0                 # Published: straddles 3×3 in upright
    ear_hole_diameter::Float64 = 1.0           # Inference: 1.0 in pin holes for SM-2.5
    ear_hole_x::Float64 = 1.5                  # Photo-estimated: 1.5 in from upright front face
    ear_first_hole_y::Float64 = 1.25           # Photo-estimated: 1.25 in from bottom of ear
    ear_hole_spacing::Float64 = 2.0            # Inference: 2.0 in vertical centers matching SM-2.5
    ear_hole_count::Int = 4                    # Inference: 4 holes spanning y = 1.25:2.0:7.25 in

    # Central roller standoff web (1×)
    web_width::Float64 = 6.0                   # Photo-estimated: 6.0 in forward projection
    web_height::Float64 = 10.5                 # Photo-estimated: 10.5 in tall
    web_thickness::Float64 = 0.1875            # Photo-estimated: 3/16 in vertical steel plate
    web_axle_hole_diameter::Float64 = 1.0      # Published/Inference: 1.0 in axle bore
    web_axle_hole_x::Float64 = 5.0             # Photo-estimated: 5.0 in forward from footplate back face
    web_axle_hole_ys::Tuple{Float64,Float64} = (1.0, 9.5)  # Photo-estimated: 8.5 in vertical spacing

    # Roller axle shafts (2×)
    axle_outer_diameter::Float64 = 1.0         # Published: 1.0 in OD shaft
    axle_stock_length::Float64 = 240.0         # Assumed: 20 ft raw round-tube mill stick
    axle_length::Float64 = 20.0                # Photo-estimated: 20.0 in shaft length
    axle_thickness::Float64 = 0.120            # Assumed: 11-gauge (0.120 in) wall

    # Cylindrical foam rollers (4×)
    roller_outer_diameter::Float64 = 4.5       # Photo-estimated: 4.5 in OD foam roller
    roller_stock_length::Float64 = 96.0        # Assumed: 8 ft foam cylinder stock
    roller_length::Float64 = 8.0               # Photo-estimated: 8.0 in roller length
    roller_thickness::Float64 = 1.75           # Published 1.0 in ID -> (4.5 - 1.0) / 2 = 1.75 in wall
    roller_inner_offset_y::Float64 = 0.25      # Photo-estimated: 0.25 in clearance from upright side planes
end

"""
    build_footplate(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec()) -> Plate

Build the windowed `24 × 14 in` Echo GHD footplate from flat plate stock with a
centered rectangular cutout window.
"""
function build_footplate(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec())
    plate = Plate(spec.footplate_width, spec.footplate_height, spec.footplate_thickness)
    top = datum(plate, SVector(0, 0, 1))
    window_center = Datum(
        top.transform ∘ Translation(spec.footplate_width / 2.0, spec.footplate_height / 2.0, 0.0),
    )
    return cut(plate, Rectangle(spec.window_width, spec.window_height), window_center)
end

"""
    build_side_ear(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec()) -> Plate

Build a 7-hole upright side mounting ear from flat plate stock.
"""
function build_side_ear(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec())
    ear = Plate(spec.ear_width, spec.ear_height, spec.ear_thickness)
    top = datum(ear, SVector(0, 0, 1))
    hole = Circle(spec.ear_hole_diameter)
    for i in 0:(spec.ear_hole_count - 1)
        y = spec.ear_first_hole_y + i * spec.ear_hole_spacing
        d = Datum(top.transform ∘ Translation(spec.ear_hole_x, y, 0.0))
        ear = drill(ear, hole, d, ThroughAll())
    end
    return ear
end

"""
    build_standoff_web(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec()) -> Plate

Build the central vertical roller standoff web with upper and lower axle holes.
"""
function build_standoff_web(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec())
    web = Plate(spec.web_width, spec.web_height, spec.web_thickness)
    top = datum(web, SVector(0, 0, 1))
    hole = Circle(spec.web_axle_hole_diameter)
    for y in spec.web_axle_hole_ys
        d = Datum(top.transform ∘ Translation(spec.web_axle_hole_x, y, 0.0))
        web = drill(web, hole, d, ThroughAll())
    end
    return web
end

"""
    build_roller_axle(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec()) -> RoundTube

Build a `1.0 in` OD steel roller axle shaft sawn to length from mill stock.
"""
function build_roller_axle(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec())
    stock = RoundTube(spec.axle_outer_diameter, spec.axle_stock_length, spec.axle_thickness)
    return saw(stock, Datum(Translation(0.0, 0.0, spec.axle_length)))
end

"""
    build_foam_roller(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec()) -> RoundTube

Build a cylindrical foam roller pad sawn to length from tubular foam stock.
"""
function build_foam_roller(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec())
    stock = RoundTube(spec.roller_outer_diameter, spec.roller_stock_length, spec.roller_thickness)
    return saw(stock, Datum(Translation(0.0, 0.0, spec.roller_length)))
end

"""
    build_echo_ghd_weldment(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec()) -> Weldment

Build the welded Echo GHD steel frame joining the windowed footplate, left and
right 4-hole upright side ears, central roller standoff web, and lower and upper
roller axle shafts via explicit joint datums and weld seams.

The weldment's local coordinate frame is the footplate's frame (`x ∈ [0, 24]`
width, `y ∈ [0, 14]` height, `z ∈ [0, 0.1875]` thickness, with `z = 0` on the
upright-facing back surface).
"""
function build_echo_ghd_weldment(spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec())
    footplate = build_footplate(spec)
    ear = build_side_ear(spec)
    web = build_standoff_web(spec)
    axle = build_roller_axle(spec)

    center_x = spec.footplate_width / 2.0
    center_y = spec.footplate_height / 2.0
    ear_y0 = center_y - spec.ear_height / 2.0
    web_y0 = center_y - spec.web_height / 2.0
    ear_bridge_overlap = (spec.ear_height - spec.window_height) / 2.0
    web_bridge_overlap = (spec.web_height - spec.window_height) / 2.0

    # 1. Left and right side ears welded along their lower rear bridge tab
    #    (x=0, y∈[0, ear_bridge_overlap], z=0) to the footplate's lower bridge
    #    straddling the 3.0 in upright.
    d_ear = Datum(LinearMap(RotZ(π / 2)))
    d_fp_left = Datum(
        Translation(center_x - spec.ear_spacing / 2.0 - spec.ear_thickness, ear_y0, 0.0) ∘
        LinearMap(RotZ(π / 2) * RotX(π / 2)),
    )
    d_fp_right = Datum(
        Translation(center_x + spec.ear_spacing / 2.0, ear_y0, 0.0) ∘
        LinearMap(RotZ(π / 2) * RotX(π / 2)),
    )
    frame = weld(footplate, d_fp_left, ear, d_ear, LineSegment(ear_bridge_overlap))
    frame = weld(frame, d_fp_right, ear, d_ear, LineSegment(ear_bridge_overlap))

    # 2. Lower and upper roller axle shafts welded circumferentially (Circle(1.0))
    #    into the standoff web's lower and upper 1.0 in axle holes.
    d_axle = Datum(Translation(0.0, 0.0, spec.axle_length / 2.0 + spec.web_thickness / 2.0))
    axle_seam = Circle(spec.axle_outer_diameter)
    web_assembly = web
    for hole_op in operations_of_type(Drill, web)
        web_assembly = weld(web_assembly, hole_op.datum, axle, d_axle, axle_seam)
    end

    # 3. Central roller standoff web sub-weldment welded along its lower 1.0 in rear
    #    bridge tab (x=0, y∈[0, 1.0], z=0) to the footplate's lower bridge at x = 12.0 + t/2.
    d_web = Datum(LinearMap(RotZ(π / 2)))
    d_fp_web = Datum(
        Translation(center_x + spec.web_thickness / 2.0, web_y0, 0.0) ∘
        LinearMap(RotZ(π / 2) * RotX(-π / 2)),
    )
    return weld(frame, d_fp_web, web_assembly, d_web, LineSegment(web_bridge_overlap))
end

"""
    build_echo_ghd_assembly(;
        upright_spec::UprightSpec=UprightSpec(),
        attachment_spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec(),
        mount_hole_index::Int=19,
    ) -> Assembly

Assemble the SM-2.5 upright, the Design B welded Echo GHD steel frame
(`Weldment`), and the four cylindrical foam rollers in a shared 3D coordinate
frame, with the second hole of the side ears pinned at upright hole
`mount_hole_index`.
"""
function build_echo_ghd_assembly(;
    upright_spec::UprightSpec=UprightSpec(),
    attachment_spec::EchoGHDAttachmentSpec=EchoGHDAttachmentSpec(),
    mount_hole_index::Int=19,
)
    upright = build_upright(upright_spec)
    frame = build_echo_ghd_weldment(attachment_spec)
    roller = build_foam_roller(attachment_spec)

    pin_z = upright_spec.first_hole_y + (mount_hole_index - 1) * upright_spec.hole_spacing
    center_z = attachment_spec.footplate_height / 2.0
    ear_z0 = center_z - attachment_spec.ear_height / 2.0
    pin_ear_z = ear_z0 + attachment_spec.ear_first_hole_y + attachment_spec.ear_hole_spacing
    mount_z = pin_z - pin_ear_z

    upright_front_x = attachment_spec.ear_spacing
    upright_center_y = attachment_spec.ear_spacing / 2.0
    web_z0 = mount_z + center_z - attachment_spec.web_height / 2.0
    lower_axle_z = web_z0 + attachment_spec.web_axle_hole_ys[1]
    upper_axle_z = web_z0 + attachment_spec.web_axle_hole_ys[2]
    axle_x = upright_front_x + attachment_spec.web_axle_hole_x
    left_roller_y0 = -attachment_spec.roller_inner_offset_y - attachment_spec.roller_length
    right_roller_y0 = attachment_spec.ear_spacing + attachment_spec.roller_inner_offset_y

    t_upright = IdentityRigidTransform
    t_frame =
        Translation(
            upright_front_x,
            upright_center_y - attachment_spec.footplate_width / 2.0,
            mount_z,
        ) ∘ LinearMap(RotZ(π / 2) * RotX(π / 2))
    t_lower_left_roller = Translation(axle_x, left_roller_y0, lower_axle_z) ∘ LinearMap(RotX(-π / 2))
    t_lower_right_roller = Translation(axle_x, right_roller_y0, lower_axle_z) ∘ LinearMap(RotX(-π / 2))
    t_upper_left_roller = Translation(axle_x, left_roller_y0, upper_axle_z) ∘ LinearMap(RotX(-π / 2))
    t_upper_right_roller = Translation(axle_x, right_roller_y0, upper_axle_z) ∘ LinearMap(RotX(-π / 2))

    return Assembly([
        upright => t_upright,
        frame => t_frame,
        roller => t_lower_left_roller,
        roller => t_lower_right_roller,
        roller => t_upper_left_roller,
        roller => t_upper_right_roller,
    ])
end

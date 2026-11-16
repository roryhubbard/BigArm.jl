# Implementation Plan

## 1. Interactive Inspection, GUI, & AI Chat Interface
Enable interactive exploration of designs by linking the manufacturing graph and 3D visualization, with a GUI and AI chat interface where you can click a graph node, datum, or face and ask the agent about it (like Hunk, but for CAD review).

## 2. Type- & Dispatch-Enforced Manufacturability
Leverage Julia's type and dispatch system to guarantee valid operations by construction: replace manual runtime validation logic (such as `_seam_on_part` and the checks in `operations.jl`) with types and constructors that enforce validity by construction and constrain `AbstractDatum`, `AbstractProfile`, and `AbstractOperation` combinations via multiple dispatch so only valid `(part, datum, profile, operation)` combinations can be expressed. Consider design restrictions that simplify verification and visualization (for example, marking a modified face as dirty so it cannot be modified again and operations do not need to walk the manufacturing graph to inspect current face state). Additionally, the API should expose the valid design space to the user—for example, when cutting or welding on a cylinder wall, the user should be able to query a part/datum for what geometry is permissible for a given operation, or better yet, have the geometric constraints embedded directly into the coordinate system in which profiles and datums are constructed.

## 3. Fabrication & CAD Exports
Generate standard fabrication files from derived 2D and 3D geometry—including DXF cut profiles and STEP solids—so designed parts can be sent directly to an automated shop.

## 4. DFM & Assembly Verification
Provide automated checks for assembly interference and advisory design-for-manufacturability (DFM) rules.

## 5. Tolerances in the Manufacturing Graph
Attach dimensional and positional tolerances directly to stock nodes and operation edges in the manufacturing graph to verify worst-case mating hardware fit and embed tolerance data in shop exports.

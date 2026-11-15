# Horizon

1. **Leverage Julia's type and dispatch system to guarantee valid operations by construction:**
   Replace manual runtime validation logic (such as `_seam_on_part` and a bunch of the checks in operations.jl) with types and constructors that enforce validity by construction and constrain `AbstractDatum`, `AbstractProfile`, and `AbstractOperation` combinations via multiple dispatch so only valid `(part, datum, profile, operation)` combinations can be expressed.
   a. consider design restrictions that will simplify verification and even visualization code e.g. once a face of a part is cut or drill or modified in any way it is marked as "dirty" or something and cannot be modified again. that way, operations don't need to check the manufacturing graph for what the face currently looks like.
2. GUI + AI chat interface - click a datum or face and ask the agent about it. Kind of like hunk but for CAD review.

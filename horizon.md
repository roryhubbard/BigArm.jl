# Horizon

1. **Leverage Julia's type and dispatch system to guarantee valid operations by construction:**
   Replace manual runtime validation logic (such as `_seam_on_part` and a bunch of the checks in operations.jl) with types and constructors that enforce validity by construction and constrain `AbstractDatum`, `AbstractProfile`, and `AbstractOperation` combinations via multiple dispatch so only valid `(part, datum, profile, operation)` combinations can be expressed.

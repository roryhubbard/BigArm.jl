# Design A Plan — Bolt-Together GHD Attachment

## Goal

Implement the full bolt-together GHD footplate and roller attachment specified in
`DIMENSIONS.md` (§3, *Design A*) and mount it on the SM-2.5 upright as an
`Assembly` in MeshCat without welds.

## Phases

### Phase 1: Cylindrical stock primitive (`RoundTube`)

- Add `RoundTube <: AbstractTube` in `src/primitives.jl` with construction
  validation, `saw` support, and `ManufacturingGraph` summary in `src/graph.jl`.
- Extend `derive_mesh` in `src/geometry.jl` to `RoundTube` and verify
  watertight manifold meshes in `test/primitives.jl`, `test/graph.jl`, and
  `test/geometry.jl`.

### Phase 2: Full bolt-together GHD attachment & assembly

- Update `projects/ghd/attachment.jl` to model the Design A components from
  `DIMENSIONS.md`:
  - Side cheek plates (upright pin column, standoff bolt holes, and upper/lower
    roller axle holes)
  - Central rectangular standoff tube
  - Left and right footplate wings
  - Upper and lower round roller axle shafts and four foam rollers
- Assemble all parts onto the SM-2.5 upright and verify in `test/ghd.jl` that
  every part builds, meshes as a watertight 2-manifold, and aligns coaxially at
  all upright, standoff, and roller-axle holes.

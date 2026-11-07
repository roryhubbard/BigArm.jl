# Design B Plan — Welded Echo GHD Replica

## Goal

Implement the welded Echo GHD footplate and roller attachment specified in
`DIMENSIONS.md` (§3, *Design B*), adapted to the SM-2.5 upright's 1″ holes, and
view the full assembly in MeshCat.

## Phases

### Phase 1: Cylindrical stock (`RoundTube`) and rectangular cutouts (`Rectangle`)

- Add `RoundTube <: AbstractTube` (if not already implemented) and
  `Rectangle <: AbstractProfile` in `src/primitives.jl`.
- Support cutting `Rectangle` through-windows in `Plate` parts and extend
  `derive_mesh` in `src/geometry.jl` accordingly.
- Cover valid/invalid cases and verify watertight manifold meshes in
  `test/primitives.jl`, `test/graph.jl`, and `test/geometry.jl`.

### Phase 2: Weldment primitive and DAG manufacturing graph

- Add a `Weld` operation / `Weldment <: AbstractPart` in `src/primitives.jl`
  that joins multiple `AbstractPart`s at relative `RigidTransform`s.
- Generalize `ManufacturingGraph` and `to_dot` in `src/graph.jl` from a linear
  chain to a multi-parent DAG, and extend `derive_mesh` / `visualize` to
  weldments.
- Verify DAG topology, DOT output, and meshes in `test/graph.jl` and
  `test/geometry.jl`.

### Phase 3: Welded Echo GHD attachment & assembly

- Model the Design B welded attachment from `DIMENSIONS.md` in `projects/ghd/`:
  - Welded steel frame (windowed footplate, 7-hole upright side ears, central
    roller standoff web, and upper/lower axle shafts)
  - Four cylindrical foam rollers mounted on the axles
- Assemble the welded attachment and rollers onto the SM-2.5 upright and verify
  in `test/ghd.jl` that all parts build, produce valid manufacturing DAGs and
  watertight meshes, and align coaxially at the upright pin holes and roller
  axles.

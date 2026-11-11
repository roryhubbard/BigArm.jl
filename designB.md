# Design B Plan — Welded Echo GHD Replica

## Goal

Implement the welded Echo GHD footplate and roller attachment specified in
`DIMENSIONS.md` (§3, *Design B*), adapted to the SM-2.5 upright's 1″ holes, and
view the full assembly in MeshCat.

## Phases

### Phase 1: Cylindrical stock (`RoundTube`), polygonal cutouts (`Polygon` / `Rectangle`), and `Datum`-based `Saw`

- Add `RoundTube <: AbstractTube` and `Polygon <: AbstractProfile` (with
  `Rectangle(width, height)` convenience constructor) in `src/primitives.jl`.
- Generalize `Saw` to take a `Datum` (supporting square cutoffs, miter saw cuts
  on tubes, and bandsaw cuts along arbitrary lines on plates) and support
  cutting `Polygon` profiles in `Plate` and `RectangularTube` parts.
- Extend `derive_mesh` in `src/geometry.jl` and cover valid/invalid cases and
  watertight manifold meshes in `test/primitives.jl`, `test/graph.jl`, and
  `test/geometry.jl`.

### Phase 2: Simplify 2D polygon math with `LibGEOS.jl`

- Add `LibGEOS.jl` to `Project.toml` and replace custom 2D polygon routines
  (`_signed_area`, `_clip_polygon`, `_inside_convex_polygon`) in
  `src/primitives.jl` and `src/geometry.jl` with `LibGEOS` operations
  (intersection, inward `buffer`, `contains`, and `area`).
- Verify all existing primitive, graph, and mesh tests pass with `LibGEOS`.

### Phase 3: Weldment primitive and DAG manufacturing graph

- Add a `Weld` operation / `Weldment <: AbstractPart` in `src/primitives.jl`
  that joins multiple `AbstractPart`s at relative `RigidTransform`s.
- Generalize `ManufacturingGraph` and `to_dot` in `src/graph.jl` from a linear
  chain to a multi-parent DAG, and extend `derive_mesh` / `visualize` to
  weldments.
- Verify DAG topology, DOT output, and meshes in `test/graph.jl` and
  `test/geometry.jl`.

### Phase 4: Welded Echo GHD attachment & assembly

- Model the Design B welded attachment from `DIMENSIONS.md` in `projects/ghd/`:
  - Welded steel frame (windowed footplate, 4-hole upright side ears on 2″
    centers, central roller standoff web, and upper/lower axle shafts)
  - Four cylindrical foam rollers mounted on the axles
- Assemble the welded attachment and rollers onto the SM-2.5 upright and verify
  in `test/ghd.jl` that all parts build, produce valid manufacturing DAGs and
  watertight meshes, and align coaxially at the upright pin holes and roller
  axles.

### Phase 5: Weld joint datum / seam specification and contact validation

- Extend `Weld` and `weld` in `src/primitives.jl` to carry explicit weld joint
  `Datum` / seam parameters and validate that welded parts make physical contact
  along the joint (rejecting floating or disjoint parts).
- Surface weld joint locations and seam lengths in `ManufacturingGraph` /
  `to_dot` in `src/graph.jl` and update the Design B weldment and tests in
  `projects/ghd/` and `test/`.

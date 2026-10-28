# BigArm staged end-to-end implementation plan

## Goal

First build a parameter-driven, schematic Rogue SM-2.5 upright with its circular
through-hole pattern. Then reuse that upright in a schematic model of the complete
stand assembly.

Use reusable BigArm primitives and application-specific model code. MeshCat is a
downstream viewer for derived geometry, not the design environment or geometry
kernel.

Both slices are **schematic**, not fabrication-ready. `DIMENSIONS.md` provides the
upright section, nominal height, circular-hole specification, overall footprint,
and other published product information, but not all dimensions required for an
exact model. `sm_25_monster_squat_stand_2.png` is the visual reference for the
hole pattern and assembly member layout.

## Slice 1: single perforated upright

### Scope

Include:

- One hollow 3×3-inch rectangular-tube upright
- Parameterized upright length and wall thickness
- Nominal 1-inch circular cross-holes on 2-inch centers
- Aligned openings through the front and back tube walls
- Real circular openings in the derived mesh, not just hole markers
- First-class hole features with stable identities
- An in-memory hole table derived from the same features
- Typed Julia design specifications and structural validation
- Interactive MeshCat visualization
- Nonvisual end-to-end tests

Exclude for now:

- Side keyholes, whose exact profile dimensions are not published
- Fabrication-ready hole diameters or drilling files
- Gussets, base feet, crossmembers, pull-up bars, and other assembly members
- DXF, STEP, and other file exports
- A general DFM rules engine
- Exact-fit or fabrication-ready claims

The published 1-inch hardware class is not necessarily the actual manufactured
hole diameter. The modeled diameter, first-hole offset, and hole count must be
explicit fields in the Julia design specification; any value not established by
the published documentation is identified as an assumption in the application
code.

## Slice 2: complete schematic assembly

Reuse the perforated upright definition from slice 1 and add:

- A second instance of the same upright part
- Two longitudinal base feet, one beneath each upright
- One transverse lower crossmember connecting the left and right assemblies
- Fat/skinny top pull-up bars
- Hollow rectangular-tube and solid round-bar geometry
- Explicit instance placements and assembly transforms
- Assembly-level validation and nonvisual smoke tests

The resulting schematic has seven instances: two uprights, two base feet, one
lower crossmember, and two pull-up bars. Gussets, rubber feet, base-hole patterns,
and keyholes remain excluded unless separately added with suitable dimensions.

## Architecture

### Slice 1 data flow

```text
UprightSpec(...) in projects/ghd/design.jl
              │
              ▼
       build_upright(spec)
       tube blank + hole features
              │
              ▼
             Part
              │
       ┌──────┴───────────┐
       ▼                  ▼
  validation       derived 2D profiles
                          │
                    triangulated walls
                          │
                   ┌──────┴──────┐
                   ▼             ▼
              local mesh      hole table
                   │
                   ▼
                MeshCat
```

### Slice 2 expansion

```text
perforated upright Part ──► two Instances
base-foot Parts ──────────► two Instances
lower-crossmember Part ───► one Instance
pull-up-bar Parts ────────► two Instances
                              │
                              ▼
                           Assembly
                              │
                    poses + derived local meshes
                              │
                              ▼
                           MeshCat
```

### Reusable package code

Nothing under `src/` should mention Rogue, GHD, or SM-2.5.

Initial concepts:

- **`Part`**
  - Stable identity
  - Manufacturing features rather than hand-authored mesh data
- **`TubeSection`**
  - Rectangular tube stock and saw-cut length
- **`Hole`**
  - Diameter, position, and drilling axis
  - Stable identity shared by geometry generation and the hole table
- **`RoundBar`**, added in slice 2
  - Solid round stock and saw-cut length
- **`Instance` and `Assembly`**, added in slice 2
  - Instances reference parts and assign assembly-specific names and poses
  - Assemblies preserve part identity rather than merging all geometry

One upright cross-hole feature drives aligned openings in both the front and back
walls. It must not become two unrelated hole features when geometry is derived.

Geometry is derived locally from each part. Assembly visualization reuses that
geometry and applies instance transforms, avoiding separate meshes for identical
left and right parts. Keep the type hierarchy shallow.

### GHD application code

The application owns everything Rogue-specific:

- Concrete, typed Julia values for `UprightSpec` in slice 1 and `StandSpec` in
  slice 2
- `build_upright` and `build_stand`
- Hole count, first-hole offset, and other layout decisions
- Base-foot, lower-crossmember, and pull-up-bar dimensions
- Assembly placement logic
- Design/build scripts and application-level tests
- Comments or source notes distinguishing published values from assumptions

BigArm supplies the modeling vocabulary; the application supplies product
knowledge. Design directly in Julia for these slices. Do not introduce a generic
parameter type, TOML loader, or parallel input schema yet.

### Coordinates and units

For the upright in slice 1:

- Inches and `Float64`
- Right-handed local coordinates
- `Z`: upright length, with the base at `Z = 0`
- `X`: upright width
- `Y`: upright depth and circular cross-hole drilling axis
- Local XY origin at the center of the tube cross-section

For the assembly in slice 2:

- `X`: stand width
- `Y`: stand depth
- `Z`: vertical
- Origin at the center of the footprint at floor level
- Each part uses local coordinates and is positioned by its instance

### Validation versus DFM

Slice 1 validation is limited to structural and geometric checks:

- Positive section dimensions, length, and wall thickness
- Wall thickness compatible with the section size
- Positive hole diameter and spacing
- Holes contained within the upright length and face width
- Valid first-hole offset and hole count
- Finite values throughout the typed specification

Slice 2 adds unique instance names and finite placement values. Visualization and
the reference hole table remain available with unmeasured values. Provenance
metadata and the fabrication measurement gate can be introduced with real
fabrication outputs later.

## Repository structure

```text
src/
  BigArm.jl
  model.jl            Part, TubeSection, Hole; RoundBar later
  geometry.jl         drilled profiles, triangulation, local meshes
  visualization.jl    generic MeshCat adapter
  assembly.jl         Instance and Assembly; added in slice 2

projects/ghd/
  upright.jl          UprightSpec and build_upright
  stand.jl            StandSpec and build_stand; added in slice 2
  design.jl           concrete Julia specs and interactive entry point

test/
  runtests.jl
  model.jl
  geometry.jl
  ghd.jl              nonvisual end-to-end smoke tests
```

Slice 1 dependencies:

- `GeometryBasics`
- `Triangulate`
- `MeshCat`
- `Test`

Add `CoordinateTransformations` and `Rotations` for assembly placement in slice 2.
Do not add LibGEOS, MeshIO, or STEP tooling until the corresponding functionality
is implemented.

## Implementation steps

### Slice 1

1. **Julia design specification**
   - Define a typed, application-specific `UprightSpec` containing the upright
     dimensions and hole-pattern values from `DIMENSIONS.md` and the reference
     image.
   - Instantiate it directly in `design.jl`; use comments or source notes to
     identify assumed diameter, first-hole offset, hole count, and wall thickness.
   - Test construction and validation without file I/O.

2. **Feature model**
   - Implement tube blanks, parts, and cross-hole features.
   - Establish local-part coordinates and drilling-axis conventions.
   - Test feature identity and parameter-driven pattern generation.

3. **Derived drilled geometry**
   - Generate rectangular front/back wall profiles containing circular holes.
   - Triangulate the profiles and derive the hollow-tube mesh.
   - Generate aligned openings in both walls from each cross-hole feature.
   - Test hole count, spacing, alignment, and overall mesh bounds.

4. **Upright application and hole table**
   - Implement `build_upright(spec)` using the typed Julia specification.
   - Derive an in-memory hole table from the upright's hole features.
   - Keep all product-specific dimensions and pattern decisions out of `src/`.

5. **MeshCat visualization**
   - Render the derived upright mesh with visible through-holes.
   - Add basic material and camera framing without making MeshCat a modeling
     dependency.
   - Make `projects/ghd/design.jl` suitable for `includet` in a Revise session.

6. **Slice 1 verification**
   - Add a nonvisual smoke test: `UprightSpec` → upright → validation → drilled
     mesh and hole table.
   - Verify that changing diameter, spacing, offset, or count in the Julia spec
     updates both geometry and hole table.
   - Document the REPL workflow and schematic-model limitations.

### Slice 2

7. **Assembly model**
   - Implement round-bar blanks, instances, assemblies, and poses.
   - Test part reuse and assembly placement.

8. **Complete stand application**
   - Implement `StandSpec` and `build_stand`.
   - Reuse the perforated upright part for both upright instances.
   - Add two base feet, one lower crossmember, and two pull-up bars.

9. **Slice 2 verification**
   - Add a nonvisual smoke test: `StandSpec` → assembly → validation → placed
     meshes.
   - Verify the seven-instance member layout against the reference image.
   - Verify that identical parts share derived local geometry.

## Acceptance criteria

### Slice 1

The first slice is complete when this workflow displays one perforated upright:

```sh
julia --project=.
```

```julia
using Revise
using BigArm
includet("projects/ghd/design.jl")
```

Then:

- The upright has real circular openings through its front and back walls.
- The hole table agrees with the hole features and derived openings.
- Editing upright or hole values in the Julia specification and rerunning the
  design script updates the geometry and hole table.
- Tests validate the nonvisual pipeline.
- The application clearly reports which values are assumptions.
- No side-keyhole, fabrication-ready, or exact-fit claim is produced.

### Slice 2

- MeshCat displays the complete schematic stand layout from the reference image.
- Both uprights reuse the slice 1 part definition and hole pattern.
- Editing the typed Julia specification updates the assembly.
- Assembly and nonvisual end-to-end tests pass.
- The model remains explicitly schematic and excludes undocumented joint and
  keyhole details.

## Later work

After these slices, choose a fabrication-oriented part to prove the measurement
gate and a real output path: feature model → DFM checks → DXF/hole table or other
manufacturing artifact. If file-driven design becomes useful, add TOML only as an
input adapter that constructs the same typed Julia specifications; do not make it
a second source of modeling truth. Add exact keyhole geometry only after obtaining
suitable profile dimensions or treating it as an explicitly assumed design study.

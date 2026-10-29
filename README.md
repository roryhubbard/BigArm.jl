# BigArm.jl

[![Build Status](https://github.com/roryhubbard/BigArm.jl/actions/workflows/CI.yml/badge.svg?branch=master)](https://github.com/roryhubbard/BigArm.jl/actions/workflows/CI.yml?query=branch%3Amaster)

**Manufacturability-first, code-based design and fabrication tooling — in Julia.**

## What this is

BigArm is a minimal Julia package for designing manufacturable parts and generating
the artifacts needed to fabricate them: cut files, drawings, drill plans, cost
estimates, and inspection sheets.

The first applied project is a custom glute-ham developer (GHD) extension for a
Rogue SM-2.5 Monster Squat Stand 2.0 — adapter plates, brackets, and tube sections
that mount a roller assembly to the stand. That project grounds the reusable tools
in an immediate use case; it is an application of BigArm, not the package's sole
purpose.

The design philosophy in one sentence: **the model is a list of manufacturing
operations, and geometry is derived from it.** Parts are not sculpted as free-form
solids and analyzed afterward; they are composed from the operations of a specific
shop (saw, drill, cut, weld). If a design composes in the package's vocabulary, it
can be built with those operations.

The initial scope is deliberately narrow: prismatic ("2.5D") parts — flat profiles
with a constant thickness, holes and pockets running perpendicular to the profile.
This covers the GHD assembly's parts while keeping the tools useful for similar
plates, brackets, and tube sections.

## What this is not

- Not a general-purpose CAD system and not a general B-rep or f-rep kernel. Not
  arbitrary 3D booleans, no 3D rolling-ball fillets, no sculpted surfaces.
- No STEP *import*, no CAD GUI, and no notebook-driven design environment. Design
  happens in Julia scripts and the REPL, with MeshCat for interactive inspection.
- Out-of-scope operations are borrowed, not built: STEP export will use an
  OCCT-backed bridge

## Principles

1. **Manufacturability by construction.** The modeling vocabulary is the shop's
   operation vocabulary. Unmanufacturable-by-these-means geometry is
   unrepresentable.
2. **Features are the model; geometry is derived.** A `Hole` is a first-class
   object with a stable identity, not something reverse-engineered from faces.
   The hole table, drill plan, and inspection sheet are *reads* of the model.
3. **Tolerances are data.** Fit against mating hardware is checked across tolerance
   ranges, not assumed at nominal dimensions.
4. **Dead simple Julia.** Structs + functions + multiple dispatch.

## Architecture

```
params.toml ──► feature model ──► derived geometry ──► DFM checks ──► outputs
 (project dims    (operations:      (2D profiles +      (rules as      (STEP, DXF,
  + provenance)    plates, tubes,    tessellated mesh)   tests)        drawings, …)
                    holes, welds)
```

- **Feature model** — parts built from operations: `Plate`, `TubeSection`,
  `Hole`, `Pocket`, `Weld`, each carrying its manufacturing parameters
  (stock, tool, tolerance) alongside its dimensions.
- **Derived geometry** — exact 2D profile polygons for dimensions and cutting
  layouts; a tessellated mesh for display and assembly interference checks.
  No face/edge/vertex topology is maintained — the initial 2.5D scope doesn't need it.
- **Process layer** — the operation list *is* the process plan. Cycle-time and
  cost estimation, drill sequences, and inspection points are generated from it.

## Stack

These are working choices, not commitments. Keep the core small and add each
dependency only when the corresponding functionality is implemented.

| Concern            | Working choice                                  |
| ------------------ | ----------------------------------------------- |
| Language           | Julia 1.13+                                     |
| Parameters         | TOML (stdlib)                                   |
| 2D polygon ops     | LibGEOS.jl — union, difference, offset          |
| Triangulation      | Triangulate.jl — profiles with holes → meshes   |
| Mesh interchange   | GeometryBasics.jl — MeshCat/MeshIO-compatible meshes |
| Mesh export        | MeshIO.jl / FileIO.jl — STL initially           |
| Visualization      | MeshCat.jl — interactive 3D in a browser        |
| STEP export        | OCCT-backed bridge; evaluate Gmsh.jl before M3  |

LibGEOS is the initial polygon backend because it works directly with floating-
point coordinates and provides the required booleans and offsets. A Clipper
backend remains an option if it earns its place; it should not be a second initial
dependency. GeometryBasics is used at visualization/export boundaries rather than
making the feature model depend on a viewer or mesh package.

Internal unit convention: **inches**, Float64 (the first project's hardware, US
stock, and drill sizes are all imperial). The STEP bridge converts on export.

## Development workflow

The package scaffold was created with PkgTemplates.jl. Revise.jl and
PkgTemplates.jl are development tools, not BigArm runtime dependencies; install
them in Julia's shared environment rather than in this package.

Start a package REPL from the repository root:

```sh
julia --project=.
```

Load Revise before BigArm. Use `includet` rather than `include` for design scripts
that Revise should track, and rerun the test file after edits:

```julia
using Revise
using BigArm

includet("projects/ghd/design.jl") # once the applied project script exists
include("test/runtests.jl")
```

Run the package tests in a clean root environment with:

```sh
julia --project=. -e 'using Pkg; Pkg.test()'
```

## First applied project: GHD extension

The GHD design lives in `projects/ghd/`, separately from BigArm's reusable source.
Its parameter file holds everything known about the mating hardware: published
SM-2.5 specs (3×3″ 11-gauge uprights, 1″ holes on 2″ spacing) and — once taken —
measured values (upright spacing, first-hole height, actual hole positions).
Each entry records its source and whether it has been physically measured.

Echo-side mounting geometry is not published by Rogue; it enters the project
parameters only by measurement of a purchased assembly or vendor confirmation.
Placeholders are permitted for design exploration and are visibly gated
(Principle 4) until replaced. `DIMENSIONS.md` is the current dimension reference;
it is not fabrication-ready on its own.

## Outputs

| Artifact                    | Producer        | Used for                              |
| --------------------------- | --------------- | ------------------------------------- |
| STEP (AP242)                | OCCT bridge     | Shop quoting, CNC machining           |
| DXF                         | native          | Laser / plasma / waterjet plate work  |
| STL                         | MeshIO / FileIO | Printed prototypes, jigs, templates   |
| SVG → PDF drawings          | native          | Shop drawings, 1:1 drill templates    |
| Hole table (CSV)            | native          | Drill planning, inspection            |
| Process plan + cost estimate| native          | Build planning, quote comparison      |
| Inspection sheet            | native          | First-article check against nominal   |

The package is not done when the model renders; it is done when every part has
a file a stranger could manufacture from.

## Example (API sketch — not final)

```julia
using BigArm

params = load_params("projects/ghd/params/stand.toml") # measured SM-2.5 dimensions
stand  = build_stand(params)                    # the mating hardware, as a part

plate = Plate(10.0, 3.0, 0.375)                 # L × W × thickness, inches
plate = drill(plate, Hole(1.000, (1.5, 1.5)))   # 1″ pin hole, Monster side
plate = drill(plate, Hole(0.625, (8.5, 1.5)))   # 5/8″ hole, Echo side

check_dfm(plate)            # edge distance, drill access, standard sizes — as tests
fits(stand, plate)          # clearance + hole alignment across tolerances

export_dxf(plate, "adapter_plate.dxf")
hole_table(plate, "adapter_plate_holes.csv")
```

## Roadmap

- **M0** — Package skeleton, parameter model, and GHD stand rendering in MeshCat.
- **M1** — Feature API for plates, tubes, holes; hole table and DXF export.
- **M2** — DFM checks as a test suite; process plan + cost estimator;
  SVG drawings and 1:1 templates.
- **M3** — STEP export via the selected OCCT bridge; assembly fit check across
  tolerances; generated inspection sheet.

## Repository layout

```
src/           BigArm.jl and reusable design/manufacturing tools
projects/ghd/  GHD design scripts, parameters, and generated artifacts
examples/      small examples of the reusable APIs
test/          unit tests and DFM assertions
```

## Background

The SM-2.5's published specs are sufficient to model the stand side of this
assembly; the Echo roller assembly's mounting geometry is not published and
must be measured. An earlier Build123d (Python) model of the stand, built from
published specs only, serves as the reference implementation for M0.

## License

MIT. See `LICENSE`.

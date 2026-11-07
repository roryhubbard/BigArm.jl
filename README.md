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

The design philosophy in one sentence: **a part is the sequence of shop operations
that makes it, and geometry is derived from that sequence.**

The initial scope is deliberately narrow: prismatic ("2.5D") parts — flat profiles
with a constant thickness, holes and pockets running perpendicular to the profile.
This covers the GHD assembly's parts while keeping the tools useful for similar
plates, brackets, and tube sections.

## What this is not

- Not a general-purpose CAD system and not a general B-rep or f-rep kernel. No
  arbitrary 3D booleans, no 3D rolling-ball fillets, no sculpted surfaces.
- No STEP *import*, no CAD GUI, and no notebook-driven design environment. Design
  happens in Julia scripts and the REPL, with MeshCat for interactive inspection.
- Out-of-scope functionality is borrowed, not built: STEP export will use an
  OCCT-backed bridge.

## Principles

1. **Manufacturability by construction.** The modeling vocabulary is the shop's
   operation vocabulary. Unmanufacturable-by-these-means geometry is
   unrepresentable.
2. **Operations are the model; geometry is derived.** A drilled hole is a
   first-class operation in the graph with a stable identity, not something
   reverse-engineered from faces. The hole table, drill plan, and inspection
   sheet are *reads* of the model.
3. **Tolerances are data.** Fit against mating hardware is checked across tolerance
   ranges, not assumed at nominal dimensions.
4. **A small set of primitives.** BigArm is built up from a few carefully
   reasoned-about primitives. Types form a shallow hierarchy with few roots and
   plain structs at the leaves: structs say what a thing is, and methods say what
   can be done to it. Multiple dispatch across the hierarchy turns a handful of
   types and verbs into an expressive design language that stays dead simple.
   Primitives are named for what they are in CAD/CAM, not for the project that
   first needed them.

## Designing with shop operations

BigArm's main departure from conventional CAD is that a part is never sculpted as
a free-form solid and checked for manufacturability afterward. Its design is the
sequence of operations a shop performs on it — saw, drill, cut, weld — and each
operation takes a part and returns a new one. Parametric CAD keeps a history too,
but its steps are mostly geometric (sketch, extrude, boolean), so the history
describes a shape. BigArm's history is the process plan.

Just as [CVX](https://cvxr.com/cvx/) guarantees convexity by composing known atoms
under strict rules, BigArm guarantees manufacturability by starting with raw mill
stock and evolving parts solely through valid shop operations. Julia's type
hierarchy and multiple dispatch enforce the rules: if an operation dispatches and
constructs a new part, that part is manufacturable.

- **Dispatch** decides which operations apply to which kinds of part. If the shop
  can't do it, there is no method.
- **Construction** checks values. A hole wider than the face it passes through
  never constructs.

The guarantee is only as strong as the rules: *manufacturable* means
manufacturable by the shop the rules describe. Like CVX's rules, BigArm's are
sufficient, not necessary, so some makeable parts can't be expressed yet. The fix
is the same as in CVX: add an atom, which here means an operation and its rules.
Rules about cost or risk rather than possibility, such as edge distance or tight
tolerances, belong to DFM checks: warnings, not errors.

## The manufacturing graph

Every operation takes one or more parts and returns a new one, so a design forms a
directed acyclic graph: parts are the nodes and operations are the edges. A single
part's history is a path from mill stock to finished part; a weld is where two
paths join. The graph plays the role of CVX's expression tree. Every node passed
the rules when it was built, so every node is a valid part that can be rendered,
and walking the graph replays the build.

Ideas the graph opens up, to be tested rather than promised:

- **Complexity as structure.** Operation count, setup changes, and the longest
  chain of dependent operations are graph quantities that may track fabrication
  cost and lead time. Raw counts treat a ¼″ hole and a 10′ weld alike, so useful
  measures likely need shop data such as time and cost on the edges.

## End to end process

```
design script ──► manufacturing graph ──┬──► derived geometry ──► MeshCat, DXF, STEP, …
                  stock + operations,   ├──► graph view
                  checked as built      ├──► DFM checks
                                        └──► process plan, hole table, cost
```

- **Manufacturing graph** — the model: mill stock and the shop operations applied
  to it, each operation carrying its manufacturing parameters (tool, tolerance)
  alongside its dimensions.
- **Derived geometry** — exact 2D profile polygons for dimensions and cutting
  layouts; a tessellated mesh for display and assembly interference checks.
  No face/edge/vertex topology is maintained — the initial 2.5D scope doesn't need it.
- **Graph view** — the graph itself, with the part at any node viewable in MeshCat.
- **DFM checks** — advisory rules, run as tests.
- **Process layer** — the graph *is* the process plan. Cycle-time and cost
  estimation, drill sequences, and inspection points are generated from it.

## Stack

These are working choices, not commitments. Keep the core small and add each
dependency only when the corresponding functionality is implemented.

| Concern            | Working choice                                  |
| ------------------ | ----------------------------------------------- |
| Language           | Julia 1.13+                                     |
| Coordinate frames  | CoordinateTransformations.jl + Rotations.jl + StaticArrays.jl |
| Parameters         | TOML (stdlib)                                   |
| 2D polygon ops     | LibGEOS.jl — union, difference, offset          |
| Triangulation      | Triangulate.jl — profiles with holes → meshes   |
| Mesh interchange   | GeometryBasics.jl — MeshCat/MeshIO-compatible meshes |
| Mesh export        | MeshIO.jl / FileIO.jl — STL initially           |
| Visualization      | MeshCat.jl — interactive 3D in a browser        |
| Graph drawing      | Graphviz — DOT export, layered layout for DAGs  |
| STEP export        | OCCT-backed bridge; evaluate Gmsh.jl first      |

## First applied project: GHD extension

The GHD design lives in `projects/ghd/`, separately from BigArm's reusable source.
Its parameter file holds everything known about the mating hardware: published
SM-2.5 specs (3×3″ 11-gauge uprights, 1″ holes on 2″ spacing) and — once taken —
measured values (upright spacing, first-hole height, actual hole positions).
Each entry records its source and whether it has been physically measured.

Echo-side mounting geometry is not published by Rogue; it enters the project
parameters only by measurement of a purchased assembly or vendor confirmation.
Placeholders are permitted for design exploration and are visibly gated until
replaced. `DIMENSIONS.md` is the current dimension reference; it is not
fabrication-ready on its own.

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

## Example (API sketch — not final)

One SM-2.5 upright, as mill stock and shop operations:

```julia
using BigArm
using CoordinateTransformations: Translation
using StaticArrays: SVector

stick = RectangularTube(3.0, 3.0, 288.0, 0.120)  # 3×3″ 11-gauge tube, 24′ mill stick
blank = saw(stick, 103.0)                        # cut to length, inches

front = datum(blank, SVector(0, -1, 0))          # front wall datum (-Y normal)
hole  = Circle(1.0)
upright = blank
for y in 2.0:2.0:100.0                           # (x, y) from bottom-left corner of front wall
    d = Datum(front.transform ∘ Translation(1.5, y, 0.0))
    upright = drill(upright, hole, d, ThroughAll())
end

drill(upright, Circle(4.0), Datum(front.transform ∘ Translation(1.5, 51.0, 0.0)), ThroughAll())  # throws

visualize(upright)               # the drilled tube in MeshCat
visualize(graph(upright))        # stock → saw → 50 drills → upright
```

## Roadmap

The current horizon is a single part: one SM-2.5 upright. It comes first because
it is simple, mostly covered by published specs, and the mating hardware every GHD
part must fit.

- **Primitives** — define the initial type hierarchy and the operations that
  dispatch on it.
- **One upright** — model a 3×3″ 11-gauge upright as mill stock sawn to length and
  cross-drilled with 1″ holes on 2″ centers. The model is schematic; unpublished
  details such as the side keyholes are left out.
- **Visualization** — render part geometry in MeshCat, for the finished upright or
  any intermediate part, and render the manufacturing graph that produces it.

The design language grows only when the GHD project demands it, and each addition
is named and placed as a general CAD/CAM concept. Everything else in this README
— exports, DFM checks, cost estimates, assemblies — is direction, not schedule.

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
must be measured.

## Looking forward

What if your shop provided the API you designed parts with? If a design compiled
against their stock and operations, you would get manufacturable geometry, a
process plan, and a cost and lead-time estimate on the spot. Pair that with
lights-out automation — the kind shops like Hadrian are building — and you could
submit a script at night and pick up finished parts in the morning.

## License

MIT. See `LICENSE`.

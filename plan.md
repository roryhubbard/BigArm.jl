# BigArm implementation plan

## Goal

Define BigArm's initial set of CAD/CAM primitives and use them to model a single
Rogue SM-2.5 squat stand upright as raw mill stock evolved through shop
operations, with visualization of both the derived part geometry and the
manufacturing graph that produces it.

Everything beyond this single upright — full-stand assemblies, adapter plates,
DXF/STEP exports, DFM test suites, and cost estimation — is out of scope for
this plan.

## Scope

### Include

- A small, general-purpose set of primitives in `src/` with a shallow type
  hierarchy and plain structs at the leaves
- Manufacturability rules enforced by multiple dispatch and struct construction
- Raw rectangular-tube mill stock (`RectangularTube`), a cutoff operation (`saw`), and
  a cross-drilling operation (`drill` with `Hole`)
- Immutable part nodes where each operation takes a part and returns a new part,
  forming a directed acyclic manufacturing graph
- Derived 3D mesh geometry for any part node in the graph (raw stick, sawn
  blank, partially drilled tube, or finished upright), with real circular
  openings through both front and back walls
- Interactive 3D geometry visualization in MeshCat (`visualize(part)`)
- Manufacturing graph extraction and Graphviz visualization
  (`visualize(graph(part))`)
- A schematic SM-2.5 upright script in `projects/ghd/` built from those
  primitives, with published values distinguished from assumptions
- Nonvisual unit and end-to-end tests

### Exclude

- Other stand members (base feet, crossmembers, pull-up bars, gussets) and
  multi-part assemblies
- Side keyholes (exact profile dimensions are not published)
- Plate stock, round bar, pockets, and welds (added later only when demanded by
  subsequent GHD parts)
- TOML parameter files and loaders (design directly in Julia first)
- DXF, STEP, STL, and drawing file exports
- Advisory DFM checks and cost/cycle-time estimation

## Core architecture

```text
stick = RectangularTube(...)
        │
        ▼  saw(stick, L)
      blank
        │
        ▼  drill(blank, Hole(...))   × N
     upright
        │
        ├──► derive_mesh(part) ──► MeshCat
        ├──► graph(part)       ──► Graphviz (DOT / SVG)
        └──► hole_table(part)  ──► in-memory inspection
```

### 1. Primitives and type hierarchy

Nothing under `src/` mentions Rogue, GHD, or SM-2.5. Primitives are named for
what they are in CAD/CAM, with two root abstract types and plain structs at the
leaves:

```text
AbstractPart
└── AbstractTube
    └── RectangularTube   # width, height, length, wall, parent, op

AbstractOperation
├── Saw                   # target cut length
└── Drill                 # Hole applied to a part

Hole                      # diameter, z position
```

Raw mill stock is not a separate type; it is simply a root `AbstractPart` whose
`parent` and `op` are `nothing` (`isstock(p) = isnothing(p.parent)`). Because
`saw` and `drill` take a `RectangularTube` and return a new `RectangularTube`
linked to the previous one, every intermediate and final node keeps its concrete
part type without needing wrapper structs or type parameters.

### 2. Manufacturability rules (dispatch + construction)

Following the CVX analogy in `README.md`, rules are enforced at two layers:

- **Dispatch** determines which operations apply to which parts. Only methods
  representing valid shop operations exist (`saw(::AbstractTube, length)` and
  `drill(::RectangularTube, ::Hole)`).
- **Construction** validates physical dimensions and shop constraints before a
  new part node can be created:
  - `RectangularTube`: finite positive width, height, length, and wall thickness;
    wall thickness strictly less than half of width and height
    (`2 * wall < min(w, h)`).
  - `saw(part, length)`: finite positive target length strictly less than or
    equal to the current part's length; cannot saw across an existing hole.
  - `drill(part, Hole(diameter, z))`: finite positive hole diameter and position;
    hole must fit within the inner tube wall width
    (`diameter < width - 2 * wall`); hole must lie wholly within the tube ends
    (`diameter / 2 < z < length - diameter / 2`); hole must not overlap an
    existing hole along the tube.

If any rule fails, the operation throws an `ArgumentError` or `DomainError`
immediately and no node is constructed.

### 3. The manufacturing graph

Because every part node stores its `parent::Union{Nothing,AbstractPart}` and the
`op::Union{Nothing,AbstractOperation}` that produced it, a part *is* a handle to
its own manufacturing history — no global mutable graph is needed.

- `graph(part)` (or `graph(parts...)`) walks parent links to collect the unique
  part nodes and directed operation edges `(parent ──op──► child)` in
  topological order.
- Every node in the graph is a valid `AbstractPart` that can be passed directly
  to `derive_mesh` or `visualize`.
- Applying two different operations to the same `blank` naturally creates a fork
  sharing the upstream nodes.

### 4. Coordinates and units

- Units: **inches**, `Float64`.
- Right-handed local coordinates on `RectangularTube`:
  - `Z`: along the tube length, from `0.0` at the base to `length` at the top.
  - `X`: across the front/back face width (`-width/2` to `+width/2`).
  - `Y`: across the tube cross-section height (`-height/2` to `+height/2`),
    parallel to the cross-hole drilling axis.
- One `Drill(Hole(diameter, z))` operation represents a single cross-drilled
  through-hole along the `Y` axis and produces aligned circular openings in both
  the front (`+Y`) and back (`-Y`) walls.

### 5. Derived geometry

Geometry is a read of a part node, never stored as the source of truth:

- From any `RectangularTube`, collect its dimensions and all `Drill` operations
  in its ancestor chain.
- Build the 2D outer/inner wall profiles, perforating the front and back outer
  and inner wall faces with circular hole polygons at each drilled `z`.
- Triangulate the perforated wall faces with `Triangulate.jl` and connect the
  inner cylindrical hole barrels and end caps into a closed
  `GeometryBasics.Mesh`.

## Repository layout and dependencies

```text
src/
  BigArm.jl
  primitives.jl       AbstractPart, AbstractTube, AbstractOperation,
                      RectangularTube, Hole, Saw, Drill, isstock, saw, drill
  graph.jl            ManufacturingGraph, graph, hole_table, to_dot
  geometry.jl         2D perforated wall profiles, triangulation, derive_mesh
  visualization.jl    visualize(part) via MeshCat, visualize(graph) via Graphviz

projects/ghd/
  upright.jl          UprightSpec and build_upright(spec)
  design.jl           REPL entry point for building and visualizing the upright

test/
  runtests.jl
  primitives.jl       rule enforcement, valid/invalid construction and dispatch
  graph.jl            DAG traversal, node/edge identity, forks, DOT output
  geometry.jl         mesh bounds, hole openings, intermediate-node meshing
  ghd.jl              end-to-end upright smoke tests
```

Runtime dependencies to add when implementing Tasks 3 and 4:

- `GeometryBasics`
- `Triangulate`
- `MeshCat`

Do not add `LibGEOS`, `MeshIO`, `FileIO`, `CoordinateTransformations`, or OCCT
bridges during these tasks; rectangular walls with non-overlapping circular
holes can be triangulated directly with `Triangulate.jl`.

## Tasks

### Task 1: Primitives and manufacturability rules

- Define `AbstractPart`, `AbstractTube`, and `AbstractOperation`.
- Implement `RectangularTube`, `Hole`, `Saw`, `Drill`, and `isstock`.
- Implement `saw(part::AbstractTube, length)` and
  `drill(part::RectangularTube, hole)` with all dimensional and shop-rule checks.
- Write unit tests in `test/primitives.jl` covering valid chains and verifying
  that invalid stock dimensions, oversized holes, out-of-bounds holes,
  overlapping holes, and cuts across existing holes throw errors.

### Task 2: Manufacturing graph and inspection reads

- Implement `ManufacturingGraph` and `graph(parts...)` by walking parent links
  and deduplicating shared ancestor nodes.
- Implement `operations(part)` and `hole_table(part)` as reads of the graph.
- Implement `to_dot(g::ManufacturingGraph)` to serialize nodes (stock/part
  summaries) and edges (operation labels) in Graphviz DOT format.
- Write unit tests in `test/graph.jl` verifying node counts, edge ordering,
  shared-ancestor forks, hole-table readbacks, and DOT string generation.

### Task 3: Derived 2D/3D geometry

- Implement 2D boundary and hole-loop construction for the four outer and four
  inner faces of a `RectangularTube` part node.
- Triangulate the perforated front and back faces with `Triangulate.jl`, mesh
  the unperforated side walls and top/bottom end caps, and stitch the cylindrical
  through-hole barrels connecting each outer wall to its inner wall.
- Verify in `test/geometry.jl` that `derive_mesh` succeeds on the raw `stick`,
  the sawn `blank`, an intermediate 1-hole part, and a multi-hole part, with
  expected bounding boxes and watertight/valid triangle indices.

### Task 4: Geometry and graph visualization

- Implement `visualize(part::AbstractPart)` to render `derive_mesh(part)` in
  MeshCat with clear metal styling and camera framing.
- Implement `visualize(g::ManufacturingGraph)` to render the DOT graph (to SVG
  via `dot` when Graphviz is installed, with a clean text/DOT fallback in the
  terminal).
- Keep visualization decoupled from the primitive and graph definitions.

### Task 5: SM-2.5 upright model

- In `projects/ghd/upright.jl`, define `UprightSpec` and `build_upright(spec)`
  using `RectangularTube`, `saw`, and `drill`.
- Record published SM-2.5 values from `DIMENSIONS.md` (3×3″ section, 11-gauge
  ≈ 0.120″ wall, 103″ height, 1″ hole class on 2″ centers) and explicitly mark
  assumed values (raw stick length, exact drilled diameter, first-hole offset,
  hole count) in code comments.
- Create `projects/ghd/design.jl` mirroring the `README.md` example so it can be
  loaded with `includet("projects/ghd/design.jl")` in a Revise session.

### Task 6: End-to-end verification

- Add `test/ghd.jl` to run the full upright pipeline without opening a browser:
  `UprightSpec` → `build_upright` → `graph` → `hole_table` → `derive_mesh` →
  `to_dot`.
- Confirm that `julia --project=. -e 'using Pkg; Pkg.test()'` passes cleanly.

## Acceptance criteria

The plan is complete when the README example runs end to end:

```julia
using Revise
using BigArm

stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
blank = saw(stick, 103.0)

upright = blank
for z in 2.0:2.0:100.0
    upright = drill(upright, Hole(1.0, z))
end

drill(upright, Hole(4.0, 51.0))  # throws

visualize(upright)               # renders the perforated 3×3 tube in MeshCat
visualize(graph(upright))        # renders stock → saw → 50 drills → upright
```

And:

- `visualize(stick)`, `visualize(blank)`, and `visualize(upright)` each render
  the geometry at that point in the manufacturing sequence.
- Nothing in `src/` references Rogue, GHD, or SM-2.5.
- Every invalid shop operation tested in `test/primitives.jl` fails at
  construction or dispatch before any node is added to the graph.
- All tests pass via `Pkg.test()`.

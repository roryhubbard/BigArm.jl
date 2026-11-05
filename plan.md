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
- Raw rectangular-tube mill stock (`RectangularTube`), a 2D circular profile
  (`Circle`), a homogeneous coordinate datum (`Datum` via `datums(part)`),
  discrete penetration modes (`ThroughAll`, `ThroughWall`, `Blind`), a cutoff
  operation (`saw`), and a drilling operation (`drill`)
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
        ▼  drill(blank, Circle(d), Datum(...), ThroughAll())   × N
     upright
        │
        ├──► derive_mesh(part) ──► MeshCat
        ├──► graph(part)       ──► Graphviz (DOT / SVG)
        └──► hole_table(part)  ──► in-memory inspection
```

### 1. Primitives and type hierarchy

Nothing under `src/` mentions Rogue, GHD, or SM-2.5. Primitives are named for
what they are in CAD/CAM, organized around a small set of orthogonal root
abstract types with plain structs at the leaves:

1. **`AbstractPart`** — what workpiece is in the machine
2. **`AbstractProfile`** — what 2D shape is being cut
3. **`AbstractDatum`** — where on the part (a homogeneous coordinate frame)
4. **`AbstractPenetration`** — discrete cut penetration / termination mode
5. **`AbstractOperation`** — what shop action is performed

```text
AbstractPart
└── AbstractTube
    └── RectangularTube   # width, height, length, thickness, parent, op

AbstractProfile           # pure 2D shape in a datum's XY plane
└── Circle                # diameter

AbstractDatum             # SE(3) coordinate frame relative to the part origin
└── Datum                 # CoordinateTransformations.AffineMap(RotMatrix3, SVector3)

AbstractPenetration       # discrete cut penetration mode
├── ThroughAll            # singleton: through the entire cross-section (both walls)
├── ThroughWall           # singleton: through the datum wall into the cavity only
└── Blind                 # depth::Float64 (for solid stock; not valid on thin-wall tube)

AbstractOperation         # shop action applied to a part
├── Saw                   # length::Float64
└── Drill                 # profile::Circle, datum::Datum, penetration::AbstractPenetration
```

Raw mill stock is not a separate type; it is simply a root `AbstractPart` whose
`parent` and `op` are `nothing` (`isstock(p) = isnothing(p.parent)`). Because
`saw` and `drill` take a `RectangularTube` and return a new `RectangularTube`
linked to the previous one, every intermediate and final node keeps its concrete
part type without needing wrapper structs or type parameters.

Separating `Circle` (`AbstractProfile`), `Datum` (`AbstractDatum`), and
`AbstractPenetration` keeps cut shapes reusable across parts and faces, and
models penetration the way drawings and shops actually specify it (`ThroughAll()`,
`ThroughWall()`, `Blind(d)`) without any type parameters on `Drill`.

### 2. Manufacturability rules (dispatch + construction)

Following the CVX analogy in `README.md`, rules are enforced at two layers:

- **Dispatch** decides which operations, profiles, datums, and penetrations
  compose. For example, `drill` only accepts `Circle` profiles, and a
  thin-walled `RectangularTube` only accepts `ThroughAll` or `ThroughWall`
  penetration — passing a non-circular profile or `Blind` fails by dispatch.
- **Construction** validates physical dimensions and shop constraints before a
  new part node can be created:
  - Stock dimensions and cut parameters must be physically valid (e.g., tube wall
    thickness fits inside the outer cross-section).
  - `saw` can only shorten a part (`target <= part.length`).
  - `drill` requires a datum lying on a valid wall of the part and a hole that
    fits inside the wall's inner cavity and tube ends.

If any rule fails, the operation throws immediately and no node is constructed.

### 3. How datums are defined and evolve across operations

A `Datum` is a right-handed 3D coordinate frame — a rigid transformation
$T \in SE(3)$ relative to the part origin, backed by
`CoordinateTransformations.AffineMap` and `Rotations.RotMatrix3` — with no
`width`, `height`, or face symbols stored on it:

- **Each concrete `AbstractPart` implements `datums(part)`**, returning its
  surface `Datum` frames, and `datum(part, normal)` selects a surface by its
  outward normal $\hat{z}$. Offsetting a frame by $(x, y)$ on that surface is
  standard `CoordinateTransformations` composition:
  `Datum(wall.transform ∘ Translation(x, y, 0.0))`.
- **Stock bounds update on the part node:** `saw` returns a new `RectangularTube`
  with the shortened `length`, updating the valid axial range for subsequent
  operations.
- **Setup tracking:** two operations share a 3-axis setup when their `Datum`s
  share the same outward normal $\hat{z}$.

### 4. The manufacturing graph

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

### 5. Coordinates and units

- Units: **inches**, `Float64`.
- **3D part frame (corner origin):** right-handed **Z-up** coordinates
  `(X, Y, Z)` with `(0, 0, 0)` at the bottom-front-left corner of the part, so
  all on-part coordinates are non-negative (`X ∈ [0, width]`,
  `Y ∈ [0, height]`, `Z ∈ [0, length]`):
  - **`:front`** (`n̂ = (0, -1, 0)`, at `Y = 0`) and **`:back`**
    (`n̂ = (0, +1, 0)`, at `Y = height`), each `width × length`.
  - **`:right`** (`n̂ = (+1, 0, 0)`, at `X = width`) and **`:left`**
    (`n̂ = (-1, 0, 0)`, at `X = 0`), each `height × length`.
  - **`:top`** (`n̂ = (0, 0, +1)`, at `Z = length`) and **`:bottom`**
    (`n̂ = (0, 0, -1)`, at `Z = 0`), each `width × height`.
- **2D datum frame (ASME drawing / vise corner origin):** when looking straight
  at a surface (down $-\hat{z}$), each `Datum` has a right-handed frame
  $(\hat{x}, \hat{y}, \hat{z})$ with $\hat{x} \times \hat{y} = \hat{z}$ and
  origin at its bottom-left corner:
  - On the four vertical tube walls, $\hat{y} = (0, 0, 1)$ (along the tube
    length) and $\hat{x} = \hat{y} \times \hat{z}$ points right across the wall
    width, so `wall.transform ∘ Translation(x, y, 0.0)` maps $(x, y)$ to 3D via
    $p + x\hat{x} + y\hat{y}$.
- `Drill(Circle(diameter), d, ThroughAll())` represents a cross-drilled hole
  entering the datum wall and exiting the opposite wall, producing aligned
  circular openings in both walls; with `ThroughWall()`, it pierces only the
  entry wall into the tube cavity.

### 6. Derived geometry

Geometry is a read of a part node, never stored as the source of truth:

- From any `RectangularTube`, collect its dimensions and all `Drill` operations
  in its ancestor chain.
- Build the 2D outer/inner wall profiles, perforating each wall pierced by a
  `Drill` with circular hole polygons at their local $(x, y)$ wall coordinates.
- Triangulate the perforated wall faces with `Triangulate.jl` and connect the
  inner cylindrical hole barrels and end caps into a closed
  `GeometryBasics.Mesh`.

## Repository layout and dependencies

```text
src/
  BigArm.jl
  primitives.jl       AbstractPart, AbstractTube, AbstractProfile,
                      AbstractDatum, AbstractPenetration, AbstractOperation,
                      RectangularTube, Circle, Datum, ThroughAll, ThroughWall,
                      Blind, Saw, Drill, origin, xaxis, yaxis, zaxis, datums,
                      datum, isstock, holes, saw, drill
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

Task 1 runtime dependencies (for `Datum` $SE(3)$ frames and vector math):

- `CoordinateTransformations`
- `Rotations`
- `StaticArrays`
- `LinearAlgebra`

Runtime dependencies to add when implementing Tasks 3 and 4:

- `GeometryBasics`
- `Triangulate`
- `MeshCat`

Do not add `LibGEOS`, `MeshIO`, `FileIO`, or OCCT bridges during these tasks;
rectangular walls with non-overlapping circular holes can be triangulated
directly with `Triangulate.jl`.

## Tasks

### Task 1: Primitives and manufacturability rules

- Define the five root abstract types (`AbstractPart`, `AbstractProfile`,
  `AbstractDatum`, `AbstractPenetration`, `AbstractOperation`) and the initial
  leaf structs (`RectangularTube`, `Circle`, `Datum`, `ThroughAll`,
  `ThroughWall`, `Blind`, `Saw`, `Drill`).
- Implement `datums(part)`, `datum(part, normal)`, `saw`, and `drill` with their
  dispatch and construction checks.
- Test valid operation chains and verify that invalid dimensions, unsupported
  dispatch combinations, and out-of-bounds holes fail immediately.

### Task 2: Manufacturing graph and inspection reads

- Implement `operations(part)` and `hole_table(part)` as reads of the graph.
  - TODO: is hole_table necessary right now? what is it used for?
- Implement `to_dot(g::ManufacturingGraph)` to serialize nodes (stock/part
  summaries) and edges (operation labels) in Graphviz DOT format.
  - TODO: consider using the GraphViz.jl package

### Task 3: Derived 2D/3D geometry

- implement `derive_mesh` so that we can render the parts (Task 4).
  The solution should be general enough to work with other 2.5D parts that outer
  library will support.
- Leverage `Triangulate.jl` if it simplifies the code.
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
  using `RectangularTube`, `Circle`, `Datum`, `ThroughAll`, `saw`, and `drill`.
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
using CoordinateTransformations: Translation
using StaticArrays: SVector

stick = RectangularTube(3.0, 3.0, 288.0, 0.120)
blank = saw(stick, 103.0)

front = datum(blank, SVector(0, -1, 0))
hole  = Circle(1.0)
upright = blank
for y in 2.0:2.0:100.0
    d = Datum(front.transform ∘ Translation(1.5, y, 0.0))
    upright = drill(upright, hole, d, ThroughAll())
end

drill(upright, Circle(4.0), Datum(front.transform ∘ Translation(1.5, 51.0, 0.0)), ThroughAll())  # throws

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

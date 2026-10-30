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
  (`Circle`), a face datum (`FaceDatum`), discrete penetration modes
  (`ThroughAll`, `ThroughWall`, `Blind`), a cutoff operation (`saw`), and a
  drilling operation (`drill`)
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
        ▼  drill(blank, Circle(d), FaceDatum(:front, 0.0, z), ThroughAll())   × N
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
3. **`AbstractDatum`** — where on the part (face/axis and coordinates)
4. **`AbstractPenetration`** — discrete cut penetration / termination mode
5. **`AbstractOperation`** — what shop action is performed

```text
AbstractPart
└── AbstractTube
    └── RectangularTube   # width, height, length, wall, parent, op

AbstractProfile           # pure 2D shape in a datum face's plane
└── Circle                # diameter

AbstractDatum             # location on a part
└── FaceDatum             # face::Symbol, x::Float64, z::Float64

AbstractPenetration       # discrete cut penetration mode
├── ThroughAll            # singleton: through the entire cross-section (both walls)
├── ThroughWall           # singleton: through the datum wall into the cavity only
└── Blind                 # depth::Float64 (for solid stock; not valid on thin-wall tube)

AbstractOperation         # shop action applied to a part
├── Saw                   # length::Float64
└── Drill                 # profile::Circle, datum::FaceDatum, penetration::AbstractPenetration
```

Raw mill stock is not a separate type; it is simply a root `AbstractPart` whose
`parent` and `op` are `nothing` (`isstock(p) = isnothing(p.parent)`). Because
`saw` and `drill` take a `RectangularTube` and return a new `RectangularTube`
linked to the previous one, every intermediate and final node keeps its concrete
part type without needing wrapper structs or type parameters.

Separating `Circle` (`AbstractProfile`), `FaceDatum` (`AbstractDatum`), and
`AbstractPenetration` keeps cut shapes reusable across parts and faces, and
models penetration the way drawings and shops actually specify it (`ThroughAll()`,
`ThroughWall()`, `Blind(d)`) without any type parameters on `Drill`.

### 2. Manufacturability rules (dispatch + construction)

Following the CVX analogy in `README.md`, rules are enforced at two layers:

- **Dispatch** determines which operations, profiles, datums, and penetrations
  compose:
  - `saw(::AbstractTube, length)`
  - `drill(::RectangularTube, ::Circle, ::FaceDatum, ::Union{ThroughAll,ThroughWall})`
    (with convenience `drill(part, circle, datum)` defaulting to `ThroughAll()`).
    Twist drills only cut `Circle` profiles, and thin-walled `RectangularTube`
    only permits through-wall penetration (`ThroughAll` or `ThroughWall`) —
    passing a non-circular profile or `Blind(depth)` fails by dispatch
    (`MethodError`).
- **Construction** validates physical dimensions, datum bounds, and shop
  constraints before a new part node can be created:
  - `RectangularTube`: finite positive width, height, length, and wall thickness;
    wall thickness strictly less than half of width and height
    (`2 * wall < min(w, h)`).
  - `Circle(diameter)`: finite positive diameter.
  - `FaceDatum(face, x, z)` (with convenience `FaceDatum(face, z)` defaulting
    `x = 0.0`): valid datum face symbol (`:front`, `:back`, `:left`, `:right`)
    and finite coordinates.
  - `Blind(depth)`: finite positive depth.
  - `saw(part, length)`: finite positive target length strictly less than or
    equal to the current part's length; cannot saw across an existing hole.
  - `drill(part, circle, datum, penetration)`: the datum face must exist on
    `part`; the circle centered at `(datum.x, datum.z)` must fit strictly within
    the inner cavity width of that face
    (`abs(datum.x) + diameter / 2 < face_width / 2 - wall`); the hole must lie
    wholly within the tube ends
    (`diameter / 2 < datum.z < length - diameter / 2`); and the hole must not
    overlap any existing hole on any wall it pierces (`ThroughWall` pierces only
    `datum.face`; `ThroughAll` pierces both `datum.face` and the opposite wall).

If any rule fails, the operation throws an `ArgumentError` or `DomainError`
immediately and no node is constructed.

### 3. How datums evolve across operations

`FaceDatum` is a stateless coordinate specification; the `AbstractPart` node and
its ancestor chain track how valid datum bounds and surfaces change as
operations are applied:

- **Stock bounds update on the part node:** `saw(stick, 103.0)` returns a new
  `RectangularTube` with `length = 103.0`, which immediately updates the valid
  `z ∈ (0, 103.0)` range of `:front`, `:back`, `:left`, and `:right` for all
  subsequent operations.
- **Material removal is tracked in the ancestor chain:** `drill` walks
  `part.parent` to check existing `Drill` operations on the same axis so new
  cuts cannot collide with prior cuts, and `saw` walks `part.parent` so a cutoff
  cannot slice through an existing hole.
- **Setup tracking:** because each `Drill` records its `FaceDatum`, the
  manufacturing graph can read setup changes directly from the sequence of datum
  faces.

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
- Right-handed local coordinates on `RectangularTube`:
  - `Z`: along the tube length, from `0.0` at the base to `length` at the top.
  - `X`: across the `:front` / `:back` face width (`-width/2` to `+width/2`).
  - `Y`: across the tube cross-section height (`-height/2` to `+height/2`,
    spanning the `:left` and `:right` faces).
- On a `FaceDatum(face, x, z)`, `x` is the horizontal offset from the face
  centerline and `z` is the axial position from the base (`Z = 0`).
- `Drill(Circle(diameter), FaceDatum(:front, x, z), ThroughAll())` represents a
  cross-drilled hole entering `:front` (`+Y`) and exiting `:back` (`-Y`),
  producing aligned circular openings in both walls; with `ThroughWall()`, it
  pierces only the `:front` wall into the tube cavity.

### 6. Derived geometry

Geometry is a read of a part node, never stored as the source of truth:

- From any `RectangularTube`, collect its dimensions and all `Drill` operations
  in its ancestor chain.
- Build the 2D outer/inner wall profiles, perforating each wall pierced by a
  `Drill` (`datum.face` for `ThroughWall`; both `datum.face` and its opposite
  face for `ThroughAll`) with circular hole polygons at `(x, z)`.
- Triangulate the perforated wall faces with `Triangulate.jl` and connect the
  inner cylindrical hole barrels and end caps into a closed
  `GeometryBasics.Mesh`.

## Repository layout and dependencies

```text
src/
  BigArm.jl
  primitives.jl       AbstractPart, AbstractTube, AbstractProfile,
                      AbstractDatum, AbstractPenetration, AbstractOperation,
                      RectangularTube, Circle, FaceDatum, ThroughAll,
                      ThroughWall, Blind, Saw, Drill, isstock, holes, saw, drill
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

- Define the root abstract types: `AbstractPart` (with `AbstractTube`),
  `AbstractProfile`, `AbstractDatum`, `AbstractPenetration`, and
  `AbstractOperation`.
- Implement `RectangularTube`, `Circle`, `FaceDatum`, `ThroughAll`,
  `ThroughWall`, `Blind`, `Saw`, `Drill`, `isstock`, and `holes`.
- Implement `saw(part::AbstractTube, length)` and
  `drill(part::RectangularTube, profile::Circle, datum::FaceDatum, penetration::Union{ThroughAll,ThroughWall}=ThroughAll())`
  with all dimensional, datum-bound, and shop-rule checks.
- Write unit tests in `test/primitives.jl` covering valid chains (`ThroughAll`
  and `ThroughWall`) and verifying that invalid stock dimensions, invalid faces,
  `Blind` penetration on a tube (`MethodError`), oversized or off-center holes,
  out-of-bounds holes, overlapping holes, and cuts across existing holes throw
  errors.

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
  using `RectangularTube`, `Circle`, `FaceDatum`, `ThroughAll`, `saw`, and
  `drill`.
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

hole = Circle(1.0)
upright = blank
for z in 2.0:2.0:100.0
    upright = drill(upright, hole, FaceDatum(:front, 0.0, z), ThroughAll())
end

drill(upright, Circle(4.0), FaceDatum(:front, 0.0, 51.0), ThroughAll())  # throws

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

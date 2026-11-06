# Phase 2 plan — GHD attachment & assembly

## Goal

Design the bolt-together GHD attachment (side mounting plates + horizontal tube
arm) and view it mounted on the SM-2.5 upright as an assembly in MeshCat,
extending `src/` only with the general-purpose plate and assembly primitives
needed to express it. No welds are used in this phase.

## Tasks

### Task 1: Plate primitive and manufacturability rules

- Add a 2.5D flat plate part primitive in `src/` with surface datums, drilling
  support, and manufacturing graph integration.
- Enforce physical validity at construction and operation time, and cover valid
  and invalid cases in `test/`.

### Task 2: Plate derived geometry

- Extend `derive_mesh` to plate parts (reusing the existing 2.5D slab meshing
  pipeline) and verify watertight manifold meshes in `test/geometry.jl`.

### Task 3: Assembly representation and visualization

- Add a general assembly primitive in `src/` that positions multiple parts in a
  shared 3D coordinate frame and extend `visualize` to render assemblies in
  MeshCat.

### Task 4: GHD attachment and upright assembly

- Model the GHD side mounting plates and horizontal arm tube in `projects/ghd/`,
  assemble them onto the SM-2.5 upright, and add a helper to visualize the full
  assembly in MeshCat.
- Verify in `test/ghd.jl` that all parts build, mesh, and align coaxially at
  their mating holes.

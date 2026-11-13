# Dimension Reference & Design Spec — Echo GHD Roller Attachment & SM-2.5 Monster Squat Stand 2.0

Compiled for Rory's custom GHD design work in `BigArm.jl`. Every value is labelled:
- **Published** = from Rogue product pages, support guides, or official assembly instructions (`IS0183`)
- **Photo-estimated** = scaled from `Echo GHD 1.png`–`8.png` using the `3×3 in` upright and `2.0 in` on-center hole grid as an in-frame ruler
- **Inference** = derived from mating hardware classes
- **Missing** = not yet physically measured on hardware

---

## 1. SM-2.5 Monster Squat Stand 2.0 — Published Specs (`sm_25_monster_squat_stand_2.png`)

| Item | Value | Status |
|---|---|---|
| Uprights | `3.0 × 3.0 in`, 11-gauge (`0.120 in` wall) steel tube | **Published** |
| Upright height | `103.0 in` nominal (`103.375 in` true height) | **Published** |
| Front/back holes | `1.0 in` diameter class ("1 in hardware"), `2.0 in` on-center vertical spacing | **Published** |
| Side keyhole pattern | Laser-cut keyholes on upright sides, `2.0 in` vertical spacing | **Published** |
| Footprint | `50.0 in` deep × `54.0 in` wide | **Published** |
| Weight / capacity | `225 lb` / `1000+ lb` | **Published** |

In `projects/ghd/upright.jl`, the upright occupies `X ∈ [0.0, 3.0]`, `Y ∈ [0.0, 3.0]`, `Z ∈ [0.0, 103.0]`, with `1.0 in` `ThroughAll` holes drilled along the `Y`-axis (`front` `-Y` to `back` `+Y`) at `X = 1.5 in` and `Z = 2.0:2.0:100.0 in`.

---

## 2. Echo GHD Roller & Footplate Attachment — Published & Photo-Derived Dimensions

Sources: Rogue Echo GHD product page, Rogue `IS0183` customer instruction sheet (`ML ECHO GHD — FOOT PLATE/ROLLER ASSEMBLY`), and `Echo GHD 1.png`–`8.png`.

| Component / Feature | Dimension | Status & Source |
|---|---|---|
| **Pad** | `30.0 in` L × `24.0 in` W × `10.0 in` H | **Published** (Rogue spec) |
| **Plyo / Games Box** | `30.0 in` L × `24.0 in` W × `20.0 in` H | **Published** (Rogue Games Box) |
| **Side mounting ears** (`2×`) | `3/16 in` (`0.1875 in`) laser-cut steel | **Published** (Rogue spec) |
| Side ear outer dimensions | `2.5 in` deep × `8.5 in` tall (stock); **`2.5 in` deep × `9.5 in` tall** (SM-2.5 adaptation for `1.0 in` holes) | **Photo-estimated** (`Echo GHD 3.png`, `GHD_back_view.png`) |
| Side ear inside spacing | `3.0 in` (straddles `3×3 in` upright) | **Published** (`3×3` compatibility) |
| Side ear hole pattern | `7` holes on `1.0 in` centers (`5/8 in` stock Monster Lite); adapted to **`4` holes on `2.0 in` vertical centers** (`y = 1.75:2.0:7.75 in`, `x = 1.5 in` from upright front face) for `1.0 in` SM-2.5 Monster holes | **Photo-estimated** (`Echo GHD 3.png`, `GHD_back_view.png`) / **Inference** (`2.0 in` spacing required so `1.0 in` holes have `1.0 in` web instead of touching, and all 4 holes sit inside the `7.5 in` window) |
| Detent pin diameter | `5/8 in` (`0.625 in`) on stock Monster Lite; **`1.0 in`** for SM-2.5 Monster adaptation | **Published** (`IS0183`: `5/8" × 4" Detent Pin`) / **Inference** (`1.0"` for SM-2.5) |
| **Footplate** (`1×`) | `3/16 in` (`0.1875 in`) steel plate | **Published** (Rogue spec) |
| Footplate outer dimensions | `24.0 in` wide × `14.0 in` tall | **Photo-estimated** (`Echo GHD 2.png`–`4.png`: ~7 upright hole spaces tall, matches 24 in pad width) |
| Footplate center cutout window | `9.0 in` wide × `7.5 in` tall (centered horizontally; leaves `3.0 in` upright + `3.0 in` ankle opening on each side, and `3.25 in` top/bottom bridges overlapped by the side ears and central web) | **Photo-estimated** (`Echo GHD 2.png`–`4.png`, `GHD_back_view.png`) |
| **Central roller standoff web** (`1×`) | `3/16 in` (`0.1875 in`) contoured vertical steel plate (dogbone / sideways-`I` profile with `3.0 in` central neck and `2.0 in` wide front vertical axle post) | **Photo-estimated** (`Echo GHD 4.png`, `GHD_back_view.png`) |
| Standoff web outer dimensions | `6.0 in` forward projection × `10.5 in` tall | **Photo-estimated** (`Echo GHD 2.png`, `4.png`) |
| Roller axle center offset | `5.0 in` forward from footplate front face (`8.5 in` vertical center-to-center spacing between upper and lower axles) | **Photo-estimated** (`Echo GHD 3.png`, `4.png`) |
| **Roller axle shafts** (`2×`) | `1.0 in` OD × `20.0 in` long round steel tube/bar (`11-gauge` ≈ `0.120 in` wall) | **Published** OD (`IS0183` specifies `1" Flat Washers` [8×] and `1-1/16" Shaft Collars` [4×] on the roller shafts) / **Photo-estimated** length (`20.0 in`) |
| **Foam rollers** (`4×`) | `4.5 in` OD × `1.0 in` ID × `8.0 in` long cylindrical foam pads (2 upper, 2 lower, split left/right of central web) | **Published** count & bore (`IS0183`: 4× rollers on 1" shafts) / **Photo-estimated** OD & length (`Echo GHD 2.png`–`4.png`) |
| **Stabilizer Kit** (optional box link) | Two `3/16 in` straps (`2.0 in` × `18.0 in`, `3.0 in` inside spacing) with `12` holes on `1.0 in` centers, hinged to a rear Games Box handle-slot hook bracket | **Published** (`12 settings, 1 in on centre`) / **Photo-estimated** (`Echo GHD 5.png`) |

---

## 3. Architecture & Design Specification for Next Agent

We want to model **both** versions of the full GHD Footplate + Roller Attachment mounted on the SM-2.5 upright (`X ∈ [0, 3]`, `Y ∈ [0, 3]`, `Z ∈ [0, 103]`):

1. **Design A — Bolt-Together Attachment (No Welds)**: Uses bolted tube/plate joints so every part can be cut/drilled as a flat `Plate` or `RectangularTube` / `RoundTube` and assembled with fasteners.
2. **Design B — Welded Echo GHD Replica (Adapted to SM-2.5 1″ Upright)**: Matches Rogue's welded construction in `Echo GHD 2.png`–`4.png` with `1.0 in` upright pin holes.

### Shared 3D Assembly Coordinate Frame

- **Upright (`build_upright`)**:
  - `X ∈ [0.0, 3.0]` (front of attachment projects in `+X`), `Y ∈ [0.0, 3.0]` (upright cross-hole axis runs along `Y`, so lateral left/right across the footplate is along `Y`), `Z ∈ [0.0, 103.0]` (vertical height).
  - Mating upright holes are at `X = 1.5`, `Y ∈ [0.0, 3.0]`, `Z = z_pin` (e.g., `mount_hole_index = 19` → `Z = 38.0 in`).

---

### Design A: Full Bolt-Together GHD Attachment (No Welds)

Evolves `projects/ghd/attachment.jl` from a bare 4-part T-joint into the complete bolt-together footplate + quad-roller attachment without requiring welds:

```text
                       [ Left Footplate Wing ]   [ Right Footplate Wing ]
                                  │                         │
   [Upright 3×3] ◄── [ Left Side Plate ] ── [ 3×3 Standoff Tube ] ── [ Right Side Plate ]
                                  │                                         │
                        (Upper & Lower 1" Roller Axles bolted through Side Plates)
                                  │                                         │
                       [ 2× Left Foam Rollers ]                 [ 2× Right Foam Rollers ]
```

#### Parts to Model in Design A

1. **Left & Right Side Mounting / Cheek Plates (`2 × Plate`, `3/16 in` thick)**:
   - Dimensions: `width = 9.0 in` (along `X`, spanning `[0.0, 9.0]` from the back of the upright to the roller axles), `height = 10.5 in` (along `Z`), `thickness = 0.1875 in`.
   - Placed at `Y ∈ [-0.1875, 0.0]` (`left_plate`) and `Y ∈ [3.0, 3.1875]` (`right_plate`).
   - Hole pattern (all `ThroughAll`):
     - **Upright pin column** at `x = 1.5 in`: `7` holes of diameter `1.0 in` on `1.0 in` vertical centers (`y = 2.25:1.0:8.25 in`), so the center hole (`y = 5.25 in`) aligns coaxially with the upright's `1.0 in` hole (and holes `y = 3.25, 5.25, 7.25 in` align simultaneously with three consecutive upright holes on `2.0 in` spacing).
     - **Standoff tube bolt holes** at `x = 4.5 in` and `x = 6.5 in`, `y = 5.25 in` (`1.0 in` diameter): clamp the central `3×3 in` horizontal standoff tube between the two plates.
     - **Upper & lower roller axle holes** at `x = 7.75 in`, `y = 1.0 in` (lower) and `y = 9.5 in` (upper) (`1.0 in` diameter, `8.5 in` apart vertically): support the two horizontal `1.0 in` OD roller axle shafts!
2. **Central Standoff Tube (`1 × RectangularTube`, `3.0 × 3.0 × 6.0 in`, `0.120 in` wall)**:
   - Sawn to `length = 6.0 in` from `288.0 in` stock; extends along `X ∈ [3.0, 9.0]`, `Y ∈ [0.0, 3.0]`, centered vertically on `y = 5.25 in` of the side plates.
   - Cross-drilled along `Y` (`ThroughAll`) with two `1.0 in` holes at `z_local = 1.5 in` (`X = 4.5 in`) and `z_local = 3.5 in` (`X = 6.5 in`) coaxial with the side plates' standoff bolt holes.
   - Also cross-drilled vertically (`left`/`right` walls in tube local frame, which lie at the top/bottom faces in assembly `Z`) near `X = 3.0 + 1.5 = 4.5 in` if bolting a top/bottom footplate flange, or the footplate bolts through flange tabs.
3. **Footplate (`1 × Plate` or `2 × Plate` wings, `3/16 in` thick)**:
   - Flush at the front of the upright (`X ∈ [3.0, 3.1875]`, or two `8.5 in × 14.0 in × 0.1875 in` left/right footplate wings on `Y ∈ [-8.5, 0.0]` and `Y ∈ [3.0, 11.5]`, leaving the `3.0 in` central channel open, or a single `24.0 in × 14.0 in × 0.1875 in` plate once rectangular window cutout support is added).
4. **Upper & Lower Roller Axle Shafts (`2 × RoundTube` or `Cylindrical` part, `1.0 in` OD × `20.0 in` long)**:
   - Horizontal shafts running along `Y ∈ [-8.5, 11.5]` (centered on `Y = 1.5 in`), passing coaxially through the upper (`y = 9.5 in`) and lower (`y = 1.0 in`) axle holes of both side plates at `X = 7.75 in`.
5. **Four Foam Rollers (`4 × RoundTube`, `4.5 in` OD × `1.0 in` ID × `8.0 in` long)**:
   - Mounted coaxially on the upper and lower axle shafts outboard of the left and right side plates (`Y ∈ [-8.1875, -0.1875]` on the left and `Y ∈ [3.1875, 11.1875]` on the right).

---

### Design B: Welded Echo GHD Replica (Adapted to SM-2.5 1″ Upright)

Models the exact factory Echo GHD geometry shown in `Echo GHD 2.png`–`4.png`, adapted to `1.0 in` SM-2.5 upright holes.

#### New Reusable `BigArm` Primitives Required in `src/` for Design B

1. **`RoundTube <: AbstractTube`** (`outer_diameter`, `length`, `thickness`, `parent`, `op`):
   - Supports `saw(tube, length)` and `derive_mesh(tube)` (annular cylinder mesh).
   - Used for both the `1.0 in` OD steel roller axle shafts (`RoundTube(1.0, 20.0, 0.120)`) and the `4.5 in` OD foam rollers (`RoundTube(4.5, 8.0, 1.75)`).
   - *(Note: `RoundTube` is also used by Design A for the roller axles and foam rollers.)*
2. **`Rectangle <: AbstractProfile`** (`width`, `height`) + **`cut(part::Plate, profile::Rectangle, datum::Datum, ::ThroughAll)`** (or generalizing `drill`/profile cutting on `Plate`):
   - Cuts a rectangular `width × height` window through a `Plate` centered at `datum` (`_contour(rect::Rectangle)` returns the 4-vertex CCW rectangle loop in `_add_slab!`).
   - Used to cut the `9.0 in × 8.5 in` center window in the `24.0 in × 14.0 in × 0.1875 in` Footplate.
3. **`Weld` operation / `Weldment <: AbstractPart`** (or assembling the welded steel sub-assembly):
   - Joins multiple `AbstractPart`s with relative `RigidTransform`s into a single rigid weldment node in the `ManufacturingGraph` (so `ManufacturingGraph` becomes a true DAG where paths from multiple mill-stock roots join at `Weld` operations, matching `README.md` § *The manufacturing graph*).

#### Parts & Transforms in Design B (in Attachment Local Frame, Upright Front Face at `X = 3.0`)

1. **Footplate (`Plate(24.0, 14.0, 0.1875)`)**:
   - Cut with a centered `Rectangle(9.0, 7.5)` through-window at `(x = 12.0, y = 7.0)`.
   - Oriented vertically at `X ∈ [3.0, 3.1875]`, centered on the upright (`Y ∈ [-10.5, 13.5]`, `Z ∈ [mount_z, mount_z + 14.0]`).
2. **Left & Right Upright Side Ears (`2 × Plate(2.5, 9.5, 0.1875)`)**:
   - Each drilled with `4` holes (`Circle(1.0)`) on `2.0 in` vertical centers at `x = 1.5 in`, `y = 1.75:2.0:7.75 in` (all 4 holes inside the `7.5 in` window opening).
   - Welded to the back of the Footplate (`X = 3.0`) at `Y ∈ [-0.1875, 0.0]` and `Y ∈ [3.0, 3.1875]`, spanning `X ∈ [0.5, 3.0]` and `Z ∈ [mount_z + 2.25, mount_z + 11.75]`.
3. **Central Roller Standoff Web (`Plate(6.0, 10.5, 0.1875)`)**:
   - Contoured with top and bottom perimeter cutouts leaving a `1.0 in` rear base, `1.0 in` gusset taper, `3.0 in` tall central horizontal neck, and `2.0 in` wide front vertical axle post.
   - Drilled with `2` axle holes (`Circle(1.0)`) at `x = 5.0 in`, `y = 1.0 in` (lower) and `y = 9.5 in` (upper).
   - Welded vertically to the **front** face of the Footplate (`X = 3.1875`) at the center (`Y ∈ [1.5 - 0.09375, 1.5 + 0.09375]`), projecting forward along `X ∈ [3.1875, 9.1875]` and `Z ∈ [mount_z + 1.75, mount_z + 12.25]`.
4. **Upper & Lower Roller Axle Shafts (`2 × saw(RoundTube(1.0, 240.0, 0.120), 20.0)`)**:
   - Welded coaxially through the upper and lower `1.0 in` holes of the Central Roller Standoff Web (`X = 8.1875 in`, `Z = mount_z + 2.75 in` and `Z = mount_z + 11.25 in`, spanning `Y ∈ [-8.5, 11.5]`).
5. **Four Foam Rollers (`4 × saw(RoundTube(4.5, 96.0, 1.75), 8.0)`)**:
   - Slid coaxially onto the left (`Y ∈ [-8.25, -0.25]`) and right (`Y ∈ [3.25, 11.25]`) sides of the upper and lower axle shafts.

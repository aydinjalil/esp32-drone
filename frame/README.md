# ESP32-S3 Drone Frame — Parametric (OpenSCAD)

A fully parametric 5" 6S quad frame for this project's flight controller. Every
dimension lives in [`params.scad`](params.scad); change a number, re-run
`./export.sh`, get fresh STLs. Designed around the known hardware: EMAX ECO III
1900KV motors (16×16 mount), a 132×43×25mm pack, the ICM-20948 IMU on a
vibration-isolated mount, a down-facing VL53L4CX, a GPS mast, and a future micro
FPV camera.

## Layout

```
frame/
  params.scad        # all dimensions + tolerances (edit here)
  lib/util.scad      # reusable modules (rounded plates, slots, bolt grids, dovetail)
  parts/*.scad       # one printable part per file
  assembly.scad      # visual fit check (not printed)
  export.sh          # render every part -> stl/ + png/
```

## Rendering

Requires OpenSCAD (found on this machine at
`~/Applications/OpenSCAD.app`). From this directory:

```bash
./export.sh          # writes stl/<part>.stl and png/<part>.png for all parts
```

The script auto-locates the OpenSCAD CLI on `PATH`, `/Applications`, or
`~/Applications`. STLs/PNGs are git-ignored (regenerate any time).

## Parts

| # | Part | Size (mm) | Print qty |
|---|------|-----------|-----------|
| 1 | `bottom_plate` | 120×95×3 | 1 |
| 2 | `top_plate` | 120×95×3 | 1 |
| 3 | `arm` | 135×18×6 | **4** |
| 4 | `battery_tray` | 145×50 | 1 |
| 5 | `esp32_carrier` | 60×40×2.5 (for a 40×60 perfboard) | 1 |
| 6a | `imu_lower_plate` | 35×35×2 | 1 |
| 6b | `imu_floating_plate` | 30×30×2 | 1 |
| 7 | `tof_bracket` | 25×25 (+5 shroud) | 1 |
| 8a | `gps_plate` | 32×32×2 | 1 |
| 8b | `gps_mast_adapters` | top + bottom | 1 (both) |
| 9 | `camera_mount` | 35×30×2.5 | 1 (L+R) |

## Arm mounting — dovetail seat (the key design decision)

Arms are **captured by geometry, not bolts**. Each arm has a wider root (24mm,
tapering to 22mm — a dovetail) that drops into a **1.5mm-deep mating pocket in
both plates**. Four M3×20 bolts (16×16 pattern) pass through top plate → arm →
bottom plate into nylon lock nuts.

- **Landing loads** push the arm against the pocket floors (compression), not the
  bolt shanks.
- **Motor torque** is reacted by the pocket walls (the dovetail taper self-centers
  and stops rocking), not bolt shear.
- **Crashes** break the cheap printed arm first; the center body survives.

> Note: the pocket is a **mating dovetail trapezoid** (root + 0.2mm clearance),
> not a plain rectangle. A tapered root in a rectangular pocket would only touch
> at one edge and still rock — the taper only works if the pocket tapers too.
> The arm clamps the two plates 3mm apart (6mm arm − 2×1.5mm pocket engagement);
> the electronics stack sits above the top plate and below the bottom plate.

## Print settings

| Part(s) | Material | Orientation | Perimeters | Infill |
|---------|----------|-------------|-----------|--------|
| Arms | **PA-CF / Nylon-CF** (PETG ok) | Flat (as modelled) — extrusion runs along the arm | 4–5 | 40–60% |
| Plates | PA-CF or PETG | Flat; **flip `top_plate` pocket-side up** | 4 | 30–40% |
| Tray / carrier / GPS / IMU plates | PETG | Flat | 3 | 25% |
| ToF bracket / mast adapters | PETG | As modelled (shroud/collar up) | 3 | 30% |

The IMU floating plate uses VHB tape for the ICM-20948; the lower plate and
floating plate are joined by silicone damper balls.

## Hardware (BOM)

- **16× M3×20** socket head + **16× M3 nylon lock nuts** — arm clamps (4/arm).
- **4× M3** + standoffs/nuts — FC/PDB & top↔bottom stack (30.5×30.5 pattern).
- **4× M3 + ~6mm standoffs** — perfboard to carrier (53×33 corner pattern). The
  ESP32-S3 sockets into female headers on the perfboard (stands ~8.5mm off it).
- **4× M3 + standoffs** — carrier down to the main stack (30.5×30.5).
- **4× silicone damper balls** (stem dia `damper_hole`, default 3.0mm) — IMU mount.
- **2× M3** — IMU lower plate to stack; **2× M3** — GPS plate / mast / top plate.
- **2× M2** — ToF bracket to bottom plate.
- **1× carbon tube, 10mm OD** cut to ~80mm — GPS mast (recommended over printing).

## Confirm before printing (flagged parameters)

These were not pinned down by the spec; sensible defaults are set — verify against
your actual parts and adjust in `params.scad`:

- `standoff_x/standoff_y` — currently reuses the 30.5 FC pattern for the stack.
- `pb_inset` (2.5) — perfboard corner-hole inset from each edge (→ 55×35 pattern);
  `pb_hole_d` (3.2 ≈ 0.125") — carrier corner-hole diameter for the M3 bolt.
- `damper_hole` (3.0) — match your silicone ball stems.
- `imu_stack_sep` (20) — the 2 holes that bolt the IMU lower plate down.
- `tof_mount_sep` (18), `gps_mast_sep` (20) — match your sensor breakouts.

## Assembly order

1. Seat each arm root in the bottom-plate pockets.
2. Lower the top plate so its pockets capture the arm roots; the dovetails
   self-center.
3. Run the 16× M3×20 through top → arm → bottom; tighten into lock nuts.
4. Standoffs on the 30.5 pattern; mount FC/PDB.
5. ESP32 carrier + IMU damper stack above the top plate; battery tray + ToF
   bracket below the bottom plate; GPS mast at the rear.

// =============================================================================
// params.scad — single source of truth for the ESP32-S3 drone frame (all mm)
//
// Every dimension lives here. Tweak a number, re-run ./export.sh, get new STLs.
// Frame class: 5" 6S quad. Motors: EMAX ECO III 1900KV (16x16 mount).
// Battery: 132 x 43 x 25 mm. See README.md for the full parameter discussion.
// =============================================================================

$fn = 64;       // circle smoothness for rendered/exported geometry
eps = 0.1;      // small over-cut so boolean subtractions punch cleanly through

// ---- Fastener clearance holes -----------------------------------------------
m2_clear = 2.4; // M2 clearance
m3_clear = 3.2; // M3 clearance

// ---- Center plates (Part 1 bottom, Part 2 top) ------------------------------
plate_w       = 120;   // X, left-right
plate_d       = 95;    // Y, front-back
bottom_t      = 3;
top_t         = 3;
plate_outer_r = 4;     // outer corner fillet
slot_corner_r = 2;     // slot / cutout corner fillet

// 30.5 x 30.5 is the standard FC/PDB pattern. The same pattern doubles as the
// top<->bottom standoff stack (FLAGGED: change standoff_* if your standoffs use
// a different pitch than the PDB).
pdb_x = 30.5;
pdb_y = 30.5;
standoff_x = 30.5;
standoff_y = 30.5;

// Replaceable arms seat in mating DOVETAIL POCKETS at 4 diagonal (X-quad)
// positions. The arm root is captured by geometry (compression surfaces +
// tapered walls) so crash/landing loads go through the pocket floors and walls,
// not the bolts. Both plates get an identical pocket on their arm-facing side.
root_w_in    = 24;   // root width at the center-body (inboard) end
root_w_out   = 22;   // root width at the arm (outboard) end -> dovetail taper
root_len     = 25;   // root seat length, radial
root_pos     = 40;   // radial distance of pocket/root center from plate center
pocket_depth = 1.5;  // pocket depth in EACH plate (arm captured top + bottom)
pocket_clear = 0.2;  // per-side clearance for nylon-CF print tolerance
arm_bolt     = 16;   // 4x M3 through-bolt pattern (16x16) at the root

// Battery strap slots on the bottom plate (strap runs front-to-back over pack).
batt_strap_l   = 25; // slot length (across, lets a ~20mm strap pass)
batt_strap_w   = 5;
batt_strap_sep = 76; // front/rear separation along Y (clear of the arm pockets)

// VL53L4CX downward window through the bottom plate.
tof_window     = 15;
tof_window_fwd = 22; // forward (+Y) offset from plate center

// GPS mast holes at rear of top plate.
gps_mast_sep = 20;   // 2x M3, 20mm apart
gps_mast_y   = 38;   // rear (-Y) offset

// Camera side-plate holes at front of top plate.
cam_side_sep   = 19; // micro FPV cam width
cam_side_inset = 8;  // from front edge

// ---- Arm (Part 3) -----------------------------------------------------------
arm_l        = 135;
arm_w        = 18;
arm_t        = 6;
arm_long_r   = 2;    // long-edge fillet
arm_root_r   = 4;    // concave fillet where the arm body meets the clamp root
motor_pad    = 32;   // square motor pad at the outboard end
motor_pad_r  = 3;
motor_bolt   = 16;   // 16 x 16 motor bolt circle
motor_shaft  = 9;    // center bell/shaft clearance hole
esc_pocket_l = 32;   // shallow ESC seat on top of the arm
esc_pocket_w = 16;
esc_pocket_d = 1;
ziptie_l     = 10;   // zip-tie slots (one each side of the ESC)
ziptie_w     = 3;
ziptie_sep   = 26;   // along the arm, straddling the ESC pocket

// ---- Battery bed (Part 4) — pack rides on TOP in a fully-walled, snug bed -----
// The pack is captured on all 4 sides (no lengthwise slide) and held down by a
// strap; walls are tall enough to contain it, not just a token lip.
batt_l   = 132.58;  // MEASURED pack length
batt_w   = 43;      // pack width
batt_h   = 25;      // pack height
batt_fit   = 1.0;   // length clearance (Y): pack drops in but can't slide
batt_fit_w = 4.0;   // width clearance (X): widened +3.0mm per revision (was 1.0)

tray_wall    = 2.5; // perimeter wall thickness
tray_wall_h  = 14;  // wall height -> contains the pack (was a 3mm lip => it slid)
tray_base_t  = 2.5; // floor thickness
tray_strap_l = 20;  // strap slot length (across the pack, ~20mm velcro)
tray_strap_w = 4;   // strap slot width
tray_strap_sep = 80;// along the pack, straddling the CG

// TWO battery-lead exit cutouts on ONE end wall (the pack's twin power wires exit
// the same side). Open-topped, fully-radiused U-slots placed near the L/R edges so
// each wire exits straight without pinching or rubbing.
tray_lead_w    = 10; // width of EACH cutout (per thick power wire)
tray_lead_edge = 3;  // gap from the cavity side wall to a cutout's outer edge

// Battery bed mounts DIRECTLY TO THE FRAME on its FOUR EXISTING perimeter
// standoff holes (the stock Top_Plate mount box) — already present in the printed
// Main_Body, so NO new holes and no frame reprint. Tall standoffs rise from these
// holes, beside the electronics, to the tray. The 30.5 central column can't be
// reused because the ESP32 perfboard caps it (no holes there). The holes are
// slightly trapezoidal (front pair wider than rear); given as explicit offsets
// from the bed center (10.05, 51.35). World coords (measured from Main_Body.stl):
//   front (-12.0, 26.1) & (32.1, 26.1);  rear (-9.0, 76.6) & (29.1, 76.6).
tray_mount_front_x = 22.05;  // front pair: x = +/- this from bed center
tray_mount_rear_x  = 19.05;  // rear pair:  x = +/- this (narrower)
tray_mount_y       = 25.25;  // both pairs: y = -/+ this (front -y, rear +y)

// ---- ESP32 / sensor carrier (Part 5) ----------------------------------------
// The ESP32-S3 sockets into female headers on a 40 x 60mm perfboard; the
// perfboard (not the bare module) bolts to this carrier at its 4 corner holes.
carrier_l    = 60;   // matches the 40 x 60mm perfboard footprint
carrier_w    = 40;
carrier_t    = 2.5;
carrier_r    = 0.8;  // near-sharp corners to match the perfboard's square edges
pb_inset     = 2.5;  // perfboard corner-hole CENTER, distance from each edge -> 55x35
pb_hole_d    = 3.2;  // ~0.125" board hole; an M3 bolt clears through carrier + board
pb_mount_x   = carrier_l - 2 * pb_inset; // -> corner-hole pattern, derived from inset
pb_mount_y   = carrier_w - 2 * pb_inset;
carrier_stack_x = 30.5; // 4x M3 down to the main stack standoffs
carrier_stack_y = 30.5;
header_h     = 8.5;  // ESP32 stands this far off the perfboard on female headers
qt_slot_l    = 8;    // STEMMA QT / I2C cable pass-through
qt_slot_w    = 3;

// ---- Common sensor breakout (ICM-20948 / BMP581 / VL53L4CX, Adafruit 1.0x0.7")
// Rounded-corner boards with 4 corner mounting holes + STEMMA QT ports on the
// ends and pin headers on the long edges -> mounts must stay OPEN at the edges.
sb_l   = 25.7;   // board length
sb_w   = 17.7;   // board width
sb_r   = 2.5;    // board corner radius (rounded)
sb_hole_d = 2.4; // M2 clearance in the carrier (board holes are 2.1mm = M2)
sb_mx  = 20.1;   // measured 4-corner hole pattern along length (17.97 edge + 2.1)
sb_my  = 12.4;   // measured 4-corner hole pattern across width (10.3 edge + 2.1)

// ---- Isolated IMU mount (ICM-20948 on a TPU pad; baro lives on the perfboard)
// Open plate: the board sits raised on a central TPU pad so its STEMMA ports,
// SDO/address jumper, and pin headers all stay accessible. Soft pad isolates
// the gyro from motor vibration and keeps it off the ESP32's EMI.
im_l   = 36;     // plate (fits within the 54.5mm-wide body)
im_w   = 36;
im_t   = 2.5;
im_r   = 3;
im_mount = 30.5; // 30.5 x 30.5 -> reference frame standoffs
imu_pad_l = 16;  // central TPU pad: smaller than the board so edges stay open
imu_pad_w = 12;
imu_pad_h = 3;
imu_pad_seat = 1; // recess that locates the pad on the plate
imu_wire_w = 4;   // notch to route the STEMMA + SDO jumper off the board

// ---- IMU vibration-isolation mount (LEGACY 2-plate silicone mount) -----------
// Superseded by the sensor deck + TPU pad; kept for reference only.
imu_lower    = 35;
imu_floating = 30;
imu_mount_t  = 2;
damper_pattern = 25; // 25 x 25 damper-ball hole pattern (shared by both plates)
damper_hole  = 3.0;  // FLAGGED: set to your silicone ball stem diameter
imu_stack_sep = 20;  // FLAGGED: 2x M3 holes that bolt the lower plate to the stack

// ---- ToF down-bracket (VL53L4CX, same board family + 20.1x12.4 M2 pattern) ---
// Holds the ToF facing DOWN under the body; open top so STEMMA + XSHUT wires
// route up to the ESP32. Adjustable end tabs bolt to the body underside.
tf_l        = 30;   // bracket frame length
tf_w        = 22;   // bracket frame width
tf_t        = 2.5;
tf_open_l   = 16;   // central optical opening
tf_open_w   = 12;
tf_shroud_h = 4;    // downward light shroud around the optical path
tf_shroud_w = 2;    // shroud wall thickness
tf_tab_l    = 9;    // end mounting tabs
tf_tab_slot = 9;    // slot length -> adjustable position under the body
tf_wire_w   = 4;    // XSHUT + STEMMA wire notch

// ---- GPS plate + carbon-tube mast adapters (Part 8) -------------------------
gps_plate     = 32;
gps_plate_t   = 2;
gps_board     = 25.5;
gps_zip_l     = 10;  // zip-tie slots to retain the GPS module
gps_zip_w     = 3;
gps_tube_d    = 10;  // carbon tube OUTER diameter (mast)
gps_tube_wall = 2.5; // adapter wall around the tube
gps_adapter_base_l = 25;
gps_adapter_base_w = 15;
gps_adapter_base_t = 5;
gps_adapter_collar_h = 12; // how far the tube seats into each adapter

// ---- Camera mount (Part 9) --------------------------------------------------
cam_plate_l = 35;
cam_plate_h = 30;
cam_plate_t = 2.5;
cam_lens_sep = 19;   // micro FPV camera lens-bolt spacing
cam_angles   = [0, 15, 30, 45]; // selectable tilt holes
cam_pivot_y  = 10;   // pivot hole height above the plate base
cam_angle_r  = 14;   // radius of the angle-selector holes from the pivot

// ---- Battery skid (Part 6) — landing gear around the under-slung pack -------
// The pack is strapped UNDER the frame and the drone was balancing on it like
// a ridge: it teetered at partial thrust and struck props on every liftoff
// attempt. The skid turns the pack into a 4-point landing base: the battery
// rides in a ladder cradle whose two transverse skis end in wide feet. The
// existing velcro straps (frame slots at batt_strap_sep) wrap pack + skid
// together — no new fasteners, straps recessed so the feet sit flat.
skid_span       = 140;  // ski tip-to-tip stance across X (roll axis)
skid_bar_w      = 16;   // each ski bar's width along Y
skid_slab_t     = 6;    // slab thickness; the pack rests on its top face
skid_wall_h     = 8;    // side-rail wall rising above the slab (guides pack)
skid_rail_w     = 5;    // side-rail width (X)
skid_ski_sep    = 76;   // ski centers along Y — matches batt_strap_sep slots
skid_relief_l   = 70;   // central under-ski relief span (strap passage): the
skid_relief_d   = 2;    //   skid stands on 4 tip pads, strap sits recessed
skid_tip_ch     = 3;    // 45deg chamfer on the ski tips' leading bottom edges

// Rigid mount: 4 corner posts rise from the side rails up to the frame belly,
// each capped with an M3 heat-set insert. Placed at the frame's FULL-WIDTH
// bands (body y~24 front, y~80 rear) where ~6mm of plate sits beside the pack.
// Posts sit ON the rails (x flush with the pack-side wall, so they don't
// intrude on the pack channel). Assembly: drop pack in, hold skid to the
// belly, mark through the 4 post bores, drill the plate 3.2mm, press inserts
// into the post tops, bolt DOWN from the plate top face. The posts are their
// own drill template — no measuring. Needs 4x M3 screws ~12mm (6mm plate +
// insert) and 4 M3 heat-set inserts.
skid_post_fy    = -26;  // front post pair: y from pack center (-> body y~25)
skid_post_ry    =  29;  // rear post pair:  y from pack center (-> body y~80)
skid_post_w     = 12;   // post width along Y
// post_t and post_x are derived in the part from the rail so the post's inner
// face stays flush with the pack-channel wall (no pack intrusion).

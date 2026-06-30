// Modified Main_Body — adds reinforced 30.5 x 30.5 standoff bosses, each cored
// for an M3 HEAT-SET INSERT (installed from the boss top), at TWO locations:
//   * CENTRAL bay (Y=49): the PDB + ESP32 carrier tower (PDB stays centered)
//   * FRONT bay  (Y=4):  the isolated IMU mount, kept low and away from the
//                        PDB's high-current traces (better for the magnetometer)
// Standoffs/screws thread into solid brass so everything clamps down securely.
// For the ToF it adds only two FLAT through-holes in the belly (NO protruding
// bosses — those needed support and ruined the print): the user screws their
// own standoffs straight into these. Original STL is untouched.
//
// The battery bed does NOT get its own bosses: it reuses the frame's four
// EXISTING perimeter standoff holes (the stock Top_Plate mount box) so there's
// no invented bolt pattern — tall brass standoffs rise from those holes to the
// top bed. See assembly_frame.scad / battery_tray.scad for the matching coords.
$fn = 48;
ref = "/Users/aydinjalil/dev/esp32-drone/frame/FPV Drone (5 Inch) - 6167128/files/Main_Body.stl";

// --- tweakable placement (body top face is z=6) ---
stack_cx   = 10.3;   // central tower center, X (body centerline)
stack_cy   = 49;     // central tower center, Y (native CENTRAL mount — PDB + ESP carrier)
imu_cx     = 10.3;   // IMU mount center, X
imu_cy     = 4;      // IMU mount center, Y (front bay — isolated, low)
stack_pat  = 30.5;   // mounting pattern (matches esp32_carrier / imu_mount)
body_top   = 6;

// Boss sized so a heat-set insert grips fresh, thick-walled material.
boss_d     = 8;      // boss outer diameter (>= insert_d + ~2mm wall for heat-set)
boss_h     = 6;      // boss height above the body (contains the whole insert)

// --- M3 heat-set insert (pressed in from the boss top) ---
// Defaults assume a ~5mm-long M3 insert wanting a ~4.0mm pilot. CHECK YOURS and
// adjust: typical pilot 3.9–4.2mm; length 4.0 / 5.0 / 5.7mm are all common.
insert_d   = 4.0;    // pilot bore for the insert
insert_h   = 5.5;    // pocket depth = insert length (~5mm) + a little overflow room
relief_d   = 2.5;    // relief below the insert: screw-tip clearance + plastic venting

// --- ToF down-bracket under-mount (front bay) -------------------------------
// The VL53L4CX bracket hangs BELOW the body facing straight down (no optical
// window needed). Mounting is just two FLAT through-holes in the belly — the
// user screws their own standoffs straight into them, bracket bolts to the
// standoff bottoms. No bosses => the belly stays flat and prints cleanly.
tof_cx      = 10.3;  // ToF center, X (body centerline)
tof_cy      = 4;     // ToF center, Y (front bay)
tof_tab_sep = 37;    // bracket tab spacing = 2 * tab_x in tof_bracket.scad
tof_screw_d = 2.6;   // pilot for self-tapping an M3 standoff straight into the frame
                     // (use 4.0 instead if you'd rather heat-set an insert here)

module bosses(cx, cy, px = stack_pat, py = stack_pat) {
    for (x = [-px/2, px/2], y = [-py/2, py/2])
        translate([cx + x, cy + y, 0]) {
            // boss with a small filleted base for strength
            translate([0, 0, body_top]) cylinder(d = boss_d, h = boss_h);
            translate([0, 0, body_top]) cylinder(d1 = boss_d + 2, d2 = boss_d, h = 1.2);
        }
}

module holes(cx, cy, px = stack_pat, py = stack_pat) {
    for (x = [-px/2, px/2], y = [-py/2, py/2])
        translate([cx + x, cy + y, 0]) {
            // heat-set insert pocket, cut down from the boss top into solid material
            translate([0, 0, body_top + boss_h - insert_h])
                cylinder(d = insert_d, h = insert_h + 1);
            // relief / vent hole the rest of the way through the body
            translate([0, 0, -1])
                cylinder(d = relief_d, h = body_top + boss_h + 2);
        }
}

// Two FLAT through-holes for the ToF standoffs — no bosses, belly stays flat.
module tof_holes() {
    for (x = [-tof_tab_sep/2, tof_tab_sep/2])
        translate([tof_cx + x, tof_cy, -1])
            cylinder(d = tof_screw_d, h = body_top + 2);
}

difference() {
    union() {
        import(ref);
        bosses(stack_cx, stack_cy);                 // central PDB + ESP tower
        bosses(imu_cx, imu_cy);                     // front-bay IMU mount
    }
    holes(stack_cx, stack_cy);
    holes(imu_cx, imu_cy);
    tof_holes();                                    // flat belly through-holes
}

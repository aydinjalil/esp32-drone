// Part 4 — Battery Bed (pack rides on TOP, fully contained)
// Length runs along Y (front-back) to match the body. A 4-walled cradle: inner
// cavity = pack + batt_fit so the 132.58mm pack drops in but can't slide (end
// walls capture the length — the old tray had only 3mm side lips and no ends).
// Walls tray_wall_h tall contain it; two velcro straps through the floor hold it
// down; the twin power leads exit two radiused cutouts on one end. Bolts directly
// to the frame's four EXISTING perimeter standoff holes via tall standoffs.
include <../params.scad>
use <../lib/util.scad>

// Countersunk M3 flat-head: head sinks flush with the floor so the LiPo rests
// flat over it (the mount holes fall inside the cavity, under the pack).
cs_head_d = 6.0;                     // flat-head top diameter
cs_depth  = (cs_head_d - m3_clear)/2; // 90deg cone -> 1.4mm, fits the 2.5mm floor

module tray_mount_hole(x, y) {
    translate([x, y, 0]) {
        translate([0, 0, -eps])               // through clearance hole
            cylinder(d = m3_clear, h = tray_base_t + 2*eps);
        translate([0, 0, tray_base_t - cs_depth])  // countersink opening at floor top
            cylinder(d1 = m3_clear, d2 = cs_head_d, h = cs_depth + eps);
    }
}

// Open-topped, fully-radiused wire-exit U-slot cut through an end wall at x=xc.
// Stadium profile (semicircular bottom + straight sides) extruded through the
// wall; the top circle sits above the wall rim so the slot is open at the top.
module wire_notch(xc, y_end) {
    r = tray_lead_w / 2;
    translate([xc, y_end, 0])
        rotate([90, 0, 0])
            linear_extrude(height = 2 * tray_wall + 4 * eps, center = true)
                hull() {
                    translate([0, tray_base_t + r])                     circle(r = r);
                    translate([0, tray_base_t + tray_wall_h + r + 1])   circle(r = r);
                }
}

module battery_tray() {
    inW  = batt_w + batt_fit_w;        // cavity width  (X) — widened +3mm
    inL  = batt_l + batt_fit;          // cavity length (Y) — snug on the pack
    outW = inW + 2 * tray_wall;
    outL = inL + 2 * tray_wall;

    difference() {
        union() {
            rounded_plate(outW, outL, tray_base_t, plate_outer_r);   // floor
            translate([0, 0, tray_base_t])                           // perimeter walls
                difference() {
                    rounded_plate(outW, outL, tray_wall_h, plate_outer_r);
                    translate([0, 0, -eps])
                        rounded_plate(inW, inL, tray_wall_h + 2*eps, slot_corner_r);
                }
        }

        // Two velcro straps: each loops over the pack through a pair of floor
        // slots just inside the long walls.
        for (y = [-tray_strap_sep/2, tray_strap_sep/2])
            for (x = [-1, 1])
                translate([x * (inW/2 - 3), y, 0])
                    rotate([0, 0, 90]) slot_cut(tray_strap_l, tray_strap_w, tray_base_t);

        // Two battery-lead exit cutouts through the rear (+Y) end wall, near the
        // left and right edges, so the pack's twin power wires exit cleanly.
        for (s = [-1, 1])
            wire_notch(s * (inW/2 - tray_lead_edge - tray_lead_w/2), outL/2);

        // Bolts to the frame's four EXISTING perimeter standoff holes (trapezoidal:
        // front pair wider than rear). Placed explicitly; countersunk flush.
        tray_mount_hole(-tray_mount_front_x, -tray_mount_y);  // front-left
        tray_mount_hole( tray_mount_front_x, -tray_mount_y);  // front-right
        tray_mount_hole(-tray_mount_rear_x,   tray_mount_y);  // rear-left
        tray_mount_hole( tray_mount_rear_x,   tray_mount_y);  // rear-right
    }
}

battery_tray();

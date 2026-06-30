// Part 1 — Bottom Center Plate
// 120 x 95 x 3mm. FC/PDB + standoff holes, 4 diagonal dovetail arm pockets
// (on the TOP/arm-facing face) with 4x M3 through-bolts each, front/rear
// battery-strap slots, and a downward VL53L4CX window.
include <../params.scad>
use <../lib/util.scad>

module bottom_plate() {
    difference() {
        rounded_plate(plate_w, plate_d, bottom_t, plate_outer_r);

        // FC / PDB mount = top<->bottom standoff stack (shared 30.5 pattern)
        bolt_grid(pdb_x, pdb_y, m3_clear, bottom_t);

        // 4 diagonal arm seats: dovetail pocket (top face) + 4x M3 through-bolts
        for (a = [45, 135, 225, 315]) rotate([0, 0, a]) translate([root_pos, 0, 0]) {
            recess_top(pocket_depth, bottom_t)
                trapezoid_2d(root_w_in + 2*pocket_clear,
                             root_w_out + 2*pocket_clear, root_len);
            bolt_grid(arm_bolt, arm_bolt, m3_clear, bottom_t);
        }

        // Battery strap slots (front +Y / rear -Y), length across the strap
        for (y = [-batt_strap_sep/2, batt_strap_sep/2])
            translate([0, y, 0]) slot_cut(batt_strap_l, batt_strap_w, bottom_t);

        // VL53L4CX downward optical window, near the front
        translate([0, tof_window_fwd, 0])
            rect_cut(tof_window, tof_window, bottom_t, slot_corner_r);
    }
}

bottom_plate();

// Part 2 — Top Center Plate
// 120 x 95 x 3mm. Matching standoff holes, 4 diagonal dovetail arm pockets (on
// the BOTTOM/arm-facing face) with 4x M3 through-bolts each, rear GPS-mast
// holes, and front camera side-plate holes.
include <../params.scad>
use <../lib/util.scad>

module top_plate() {
    difference() {
        rounded_plate(plate_w, plate_d, top_t, plate_outer_r);

        // Standoff stack holes (match bottom plate FC/PDB pattern)
        bolt_grid(standoff_x, standoff_y, m3_clear, top_t);

        // 4 diagonal arm seats: dovetail pocket (bottom face) + 4x M3 bolts
        for (a = [45, 135, 225, 315]) rotate([0, 0, a]) translate([root_pos, 0, 0]) {
            recess_bottom(pocket_depth)
                trapezoid_2d(root_w_in + 2*pocket_clear,
                             root_w_out + 2*pocket_clear, root_len);
            bolt_grid(arm_bolt, arm_bolt, m3_clear, top_t);
        }

        // GPS mast mounting holes at rear
        translate([0, -gps_mast_y, 0]) bolt_pair(gps_mast_sep, m3_clear, top_t);

        // Camera side-plate holes at front (M3, 19mm apart)
        translate([0, plate_d/2 - cam_side_inset, 0])
            bolt_pair(cam_side_sep, m3_clear, top_t);
    }
}

top_plate();

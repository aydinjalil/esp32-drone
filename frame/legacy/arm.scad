// Part 3 — Replaceable Arm (print 4x)
// Dovetail root (24mm inboard -> 22mm outboard, 25mm seat) that drops into the
// matching plate pockets, necking to an 18 x 6mm arm, out to a 32x32 motor pad
// (16x16 bolts + 9mm shaft). 4x M3 through-bolts at the root clamp it between
// the plates. Strength-critical: see README for print orientation.
include <../params.scad>
use <../lib/util.scad>

root_cx  = root_len / 2;            // root seat center (inboard end at x=0)
motor_cx = arm_l - motor_pad / 2;   // motor pad / motor center along the arm
esc_cx   = 70;                      // ESC seat / zip-tie group center

// Top-view outline: dovetail root -> arm body -> motor pad, concave junctions
// filled for fatigue strength.
module arm_profile() {
    fillet_2d(arm_root_r)
        union() {
            translate([root_cx, 0]) trapezoid_2d(root_w_in, root_w_out, root_len);
            translate([(root_len + arm_l) / 2, 0])
                rounded_rect(arm_l - root_len, arm_w, arm_long_r);
            translate([motor_cx, 0]) rounded_rect(motor_pad, motor_pad, motor_pad_r);
        }
}

module arm() {
    difference() {
        linear_extrude(height = arm_t) arm_profile();

        // Root clamp: 4x M3 through-bolts in a 16x16 pattern
        translate([root_cx, 0, 0]) bolt_grid(arm_bolt, arm_bolt, m3_clear, arm_t);

        // Motor mount: 16x16 bolt pattern + center shaft clearance
        translate([motor_cx, 0, 0]) {
            bolt_grid(motor_bolt, motor_bolt, m3_clear, arm_t);
            hole(motor_shaft, arm_t);
        }

        // Shallow ESC seat on the top face
        translate([esc_cx, 0, 0])
            pocket(esc_pocket_l, esc_pocket_w, esc_pocket_d, arm_t, slot_corner_r);

        // Zip-tie slots straddling the ESC (length across the arm)
        for (x = [esc_cx - ziptie_sep/2, esc_cx + ziptie_sep/2])
            translate([x, 0, 0]) rect_cut(ziptie_w, ziptie_l, arm_t);
    }
}

arm();

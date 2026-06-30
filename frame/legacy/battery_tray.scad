// Part 4 — Battery Tray (mounts under the bottom plate)
// 145 x 50 base with raised side lips, a recessed grip-pad pocket, two strap
// slots, and a 30.5 bolt pattern up to the bottom plate. Sized for a
// 132 x 43 x 25mm pack.
include <../params.scad>
use <../lib/util.scad>

module battery_tray() {
    union() {
        difference() {
            rounded_plate(tray_l, tray_w, tray_base_t, plate_outer_r);

            // Recessed grip-pad pocket on top (battery sits here)
            pocket(tray_recess_l, tray_recess_w, tray_recess_d, tray_base_t, slot_corner_r);

            // Strap slots (run across the pack), spaced along its length
            for (x = [-tray_strap_sep/2, tray_strap_sep/2])
                translate([x, 0, 0]) rotate([0, 0, 90])
                    slot_cut(tray_strap_l, tray_strap_w, tray_base_t);

            // Bolts up to the bottom plate (reuse FC pattern)
            bolt_grid(tray_mount_x, tray_mount_y, m3_clear, tray_base_t);
        }

        // Raised side lips that retain the battery width
        for (s = [-1, 1])
            translate([0, s * (tray_recess_w/2 + tray_lip_w/2), tray_base_t + tray_lip_h/2])
                cube([tray_recess_l, tray_lip_w, tray_lip_h], center = true);
    }
}

battery_tray();

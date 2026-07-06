// Part 6 — Battery Skid (landing gear around the under-slung pack)
// The pack rides in a ladder cradle: two transverse SKIS (the landing feet)
// joined by two longitudinal side rails whose walls guide the pack's sides.
// The frame's existing velcro straps (through the bottom-plate slots, spaced
// batt_strap_sep apart) wrap the pack AND this skid together — the ski centers
// sit exactly under the straps, and the central span of each ski's underside
// is relieved so the strap recesses and the skid stands on its four tip pads.
//
// Stance: skid_span (140) across roll x ~92 across pitch, versus the bare
// 43mm-wide battery it replaces as the ground contact. Prints flat, no
// supports (relief + chamfers face up when printed upside down — see below).
// PRINT: upside down (ski bottoms up) or as-is; both are support-free since
// the only overhangs are the 2mm relief and 45-degree tip chamfers.
include <../params.scad>
use <../lib/util.scad>

batt_chan = batt_w + 1.5;              // pack drop-in clearance across X
rail_x0   = batt_chan / 2;             // rail inner face
ski_hy    = skid_bar_w / 2;

module ski(y_c) {
    difference() {
        // bar with rounded tip corners
        translate([0, y_c, skid_slab_t / 2])
            linear_extrude(height = skid_slab_t, center = true)
                rounded_rect(skid_span, skid_bar_w, 4);
        // central underside relief: strap passage + guarantees 4-point stance
        translate([0, y_c, skid_relief_d / 2 - eps])
            cube([skid_relief_l, skid_bar_w + 2 * eps, skid_relief_d + 2 * eps],
                 center = true);
        // 45-degree chamfers on the tips' bottom edges (sled-style lead-in)
        for (sx = [-1, 1])
            translate([sx * skid_span / 2, y_c, 0])
                rotate([0, sx * 45, 0])
                    cube([2 * skid_tip_ch, skid_bar_w + 2 * eps, 2 * skid_tip_ch],
                         center = true);
    }
}

module rail(x_c) {
    // slab-level rail joining the skis, plus the pack-guide wall above it
    translate([x_c, 0, (skid_slab_t + skid_wall_h) / 2])
        cube([skid_rail_w, skid_ski_sep + skid_bar_w, skid_slab_t + skid_wall_h],
             center = true);
}

module battery_skid() {
    union() {
        for (sy = [-1, 1]) ski(sy * skid_ski_sep / 2);
        for (sx = [-1, 1]) rail(sx * (rail_x0 + skid_rail_w / 2));
    }
}

battery_skid();

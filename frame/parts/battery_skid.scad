// Part 6 — Battery Skid (landing gear + rigid mount for the under-slung pack)
// The pack rides in a ladder cradle: two transverse SKIS (the landing feet)
// joined by two longitudinal side rails whose walls guide the pack's sides.
// The frame's existing velcro straps still wrap pack + skid (ski centers sit
// under the strap slots, undersides relieved so the strap recesses), but the
// LOAD path is now four corner POSTS that bolt to the frame plate: the straps
// only assist. This kills the pre-liftoff lean the compliant straps allowed.
//
// The pack is as wide as the frame through the bay, so there is no frame
// material beside it there — the posts reach up at the frame's FULL-WIDTH
// bands (by the arm roots, body y~24 / y~80) where ~6mm of plate sits beside
// the pack. M3 heat-set inserts cap the posts; screws enter from the plate
// TOP through holes drilled using the posts themselves as the template.
//
// PRINT: as-is (feet down). Support-free: the only overhangs are the 2mm ski
// relief and the 45-degree tip chamfers.
include <../params.scad>
use <../lib/util.scad>

batt_chan = batt_w + 1.5;              // pack drop-in clearance across X
rail_x0   = batt_chan / 2;             // rail inner face (pack-channel wall)
rail_xc   = rail_x0 + skid_rail_w / 2; // rail centerline
post_top  = skid_slab_t + batt_h + 1;  // post height = flush with the belly
post_t    = skid_rail_w;               // post as thick as the rail (stays flush
                                       //   with the pack wall -> no intrusion)

module ski(y_c) {
    difference() {
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
    translate([x_c, 0, (skid_slab_t + skid_wall_h) / 2])
        cube([skid_rail_w, skid_ski_sep + skid_bar_w, skid_slab_t + skid_wall_h],
             center = true);
}

// One corner mount post at (x_c, y_c), rising from the rail to the belly with
// a heat-set insert bore in its top and a screw-tip relief below it.
module post(x_c, y_c) {
    difference() {
        translate([x_c, y_c, post_top / 2])
            cube([post_t, skid_post_w, post_top], center = true);
        translate([x_c, y_c, post_top - insert_h])
            cylinder(d = insert_d, h = insert_h + eps);
        translate([x_c, y_c, post_top - insert_h - 6])
            cylinder(d = relief_d, h = 6 + eps);
    }
}

module battery_skid() {
    union() {
        for (sy = [-1, 1]) ski(sy * skid_ski_sep / 2);
        for (sx = [-1, 1]) rail(sx * rail_xc);
        for (sx = [-1, 1]) {
            post(sx * rail_xc, skid_post_fy);
            post(sx * rail_xc, skid_post_ry);
        }
    }
}

battery_skid();

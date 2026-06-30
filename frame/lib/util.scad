// =============================================================================
// lib/util.scad — reusable 2D/3D helpers. All "cutter" modules are sized to
// over-shoot in Z so callers can subtract them cleanly; callers position them.
// Self-contained: `use <util.scad>` imports only modules, not variables, so we
// define our own over-cut constant here rather than relying on params.scad.
// =============================================================================

eps = 0.05;   // local over-cut for clean boolean subtractions

// ---- 2D primitives ----------------------------------------------------------

// Rounded rectangle centered at origin. r=0 -> sharp corners.
module rounded_rect(w, d, r = 0) {
    if (r > 0)
        offset(r = r) square([max(0.01, w - 2*r), max(0.01, d - 2*r)], center = true);
    else
        square([w, d], center = true);
}

// Isosceles trapezoid centered at origin: width w1 at the -X end, w2 at the +X
// end, length l along X. Used for the dovetail arm root + its mating pocket so
// the two always share one profile (call with +clearance for the pocket).
module trapezoid_2d(w1, w2, l) {
    polygon([[-l/2, -w1/2], [-l/2, w1/2], [l/2, w2/2], [l/2, -w2/2]]);
}

// Stadium slot: length l along X, width w along Y, fully rounded ends.
module slot_2d(l, w) {
    r = w / 2;
    hull() {
        translate([-(l/2 - r), 0]) circle(r = r);
        translate([ (l/2 - r), 0]) circle(r = r);
    }
}

// Inset-fillet a 2D shape: rounds BOTH convex and concave corners by radius r.
// Used for the arm root where the body blends into the clamp pad.
module fillet_2d(r) {
    offset(r = r) offset(delta = -r) children();
}

// ---- 3D solids --------------------------------------------------------------

// Flat plate: rounded rectangle, centered in XY, base on z=0.
module rounded_plate(w, d, h, r = 0) {
    linear_extrude(height = h) rounded_rect(w, d, r);
}

// ---- Cutters (subtract these) ----------------------------------------------

// Single through-hole, base on z=0, height h (caller lifts by -eps and pads h).
module hole(d, h) {
    translate([0, 0, -eps]) cylinder(d = d, h = h + 2*eps);
}

// 4-hole rectangular bolt pattern, spacing dx by dy, through a part of height h.
module bolt_grid(dx, dy, d, h) {
    for (x = [-dx/2, dx/2], y = [-dy/2, dy/2])
        translate([x, y, 0]) hole(d, h);
}

// 2-hole pair separated by `sep` along X (used for arm clamp / mast / cam).
module bolt_pair(sep, d, h) {
    for (x = [-sep/2, sep/2]) translate([x, 0, 0]) hole(d, h);
}

// Rounded-end through slot, length l along X.
module slot_cut(l, w, h) {
    translate([0, 0, -eps]) linear_extrude(height = h + 2*eps) slot_2d(l, w);
}

// Rectangular through cutout (window / strap slot), optional corner radius.
module rect_cut(l, w, h, r = 0) {
    translate([0, 0, -eps]) linear_extrude(height = h + 2*eps) rounded_rect(l, w, r);
}

// Shallow rectangular pocket cut DOWNWARD from the top face at z = top_z.
module pocket(l, w, depth, top_z, r = 0) {
    translate([0, 0, top_z - depth])
        linear_extrude(height = depth + eps) rounded_rect(l, w, r);
}

// Recess an arbitrary 2D child profile DOWN from the top face at z = top_z.
module recess_top(depth, top_z) {
    translate([0, 0, top_z - depth])
        linear_extrude(height = depth + eps) children();
}

// Recess an arbitrary 2D child profile UP from the bottom face (z = 0).
module recess_bottom(depth) {
    translate([0, 0, -eps])
        linear_extrude(height = depth + eps) children();
}

// Sensor-board seat: a rounded recess (captures the board by geometry, matching
// its rounded corners) plus two through mounting holes on the centerline.
// Subtract from a carrier of total thickness `h_total`, top face at `top_z`.
module board_cradle(l, w, r, depth, top_z, hole_d, hole_sep, h_total) {
    recess_top(depth, top_z) rounded_rect(l, w, r);
    for (x = [-hole_sep/2, hole_sep/2]) translate([x, 0, 0]) hole(hole_d, h_total);
}

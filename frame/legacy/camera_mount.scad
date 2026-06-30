// Part 9 — Camera Side Mounts (future micro FPV cam; L + R, printed together)
// 35 x 30 x 2.5 side plates. A pivot hole plus a 0/15/30/45-degree selector arc
// sets camera tilt; 19mm spacing matches a micro FPV camera; base holes bolt to
// the top plate's front mount holes.
include <../params.scad>
use <../lib/util.scad>

module camera_side() {
    difference() {
        linear_extrude(height = cam_plate_t)
            rounded_rect(cam_plate_l, cam_plate_h, 3);

        // Base mount holes to the top plate (front)
        translate([0, -cam_plate_h/2 + 5, 0])
            bolt_pair(cam_side_sep, m3_clear, cam_plate_t);

        // Camera pivot
        translate([0, cam_pivot_y, 0]) hole(m3_clear, cam_plate_t);

        // Tilt selector holes on an arc above the pivot
        for (ang = cam_angles)
            translate([cam_angle_r * sin(ang), cam_pivot_y + cam_angle_r * cos(ang), 0])
                hole(m3_clear, cam_plate_t);
    }
}

camera_side();                                   // right side
translate([0, cam_plate_h + 6, 0]) mirror([1, 0, 0]) camera_side();  // left side

// Part 8a — GPS Plate (mounts on top of the mast)
// 32 x 32 x 2. Zip-tie slots retain the GPS module; 2x M3 bolt it to the top
// mast adapter.
include <../params.scad>
use <../lib/util.scad>

module gps_plate() {
    difference() {
        rounded_plate(gps_plate, gps_plate, gps_plate_t, slot_corner_r);

        // Zip-tie slots straddling the GPS module
        for (s = [-1, 1])
            translate([0, s * (gps_board/2 - 1), 0])
                rect_cut(gps_zip_l, gps_zip_w, gps_plate_t, 1);

        // Bolts to the top mast adapter
        bolt_pair(gps_mast_sep, m3_clear, gps_plate_t);
    }
}

gps_plate();

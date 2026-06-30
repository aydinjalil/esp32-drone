// Part 5 — ESP32 / Perfboard Carrier Plate
// 60 x 40 x 2.5, sized to a 40 x 60mm perfboard. The ESP32 sockets into female
// headers on the perfboard; the perfboard bolts to this carrier at its 4 corner
// holes (on short standoffs to clear the header solder tails). The carrier in
// turn bolts down to the main stack. Side slots pass the I2C / STEMMA QT cabling
// to the IMU, baro, and ToF.
include <../params.scad>
use <../lib/util.scad>

// The battery tray rides on tall standoffs from the FRAME's four perimeter holes,
// which pass at/just inside this carrier's four corners. A scalloped notch at each
// corner lets them through (rear pair graze ~2.5mm, front pair ~0.2-0.7mm). Local
// coords: the carrier sits at (10.3,49) rotated +90deg, so a frame hole (px,py)
// maps to local (py-49, 10.3-px):
//   rear  (-9,76.6)->(27.6,19.3)   (29.1,76.6)->(27.6,-18.8)
//   front (-12,26.1)->(-22.9,22.3) (32.1,26.1)->(-22.9,-21.8)
bed_post_local   = [[27.6, 19.3], [27.6, -18.8], [-22.9, 22.3], [-22.9, -21.8]];
bed_post_clear_d = 6;    // ~5mm standoff + clearance

module esp32_carrier() {
    difference() {
        rounded_plate(carrier_l, carrier_w, carrier_t, carrier_r);

        // Perfboard corner mounting holes
        bolt_grid(pb_mount_x, pb_mount_y, pb_hole_d, carrier_t);

        // 4x M3 down to the main stack standoffs
        bolt_grid(carrier_stack_x, carrier_stack_y, m3_clear, carrier_t);

        // I2C / STEMMA QT cable pass-throughs at the short ends
        for (s = [-1, 1])
            translate([s * (carrier_l/2 - 4), 0, 0])
                rect_cut(qt_slot_w, qt_slot_l, carrier_t, 1);

        // Corner clearance notches for the four battery-tray standoffs
        for (p = bed_post_local)
            translate([p[0], p[1], 0]) hole(bed_post_clear_d, carrier_t);
    }
}

esp32_carrier();

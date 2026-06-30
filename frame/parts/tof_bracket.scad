// ToF Down-Bracket — holds the VL53L4CX facing DOWN under the main body.
// The board screws on top by its 4 corner holes (same 20.1x12.4 M2 pattern as
// the ICM), sensor looking down through the central opening + light shroud.
// Open top + a wire notch take the STEMMA cable and the XSHUT wire up to the
// ESP32. End tabs with slots bolt to the body underside (slide to position).
include <../params.scad>
use <../lib/util.scad>

tab_x = tf_l/2 + tf_tab_l/2 - 1;   // tab center, slightly overlapping the frame

module tof_bracket() {
    difference() {
        union() {
            rounded_plate(tf_l, tf_w, tf_t, 3);                 // board frame

            // Downward light shroud around the optical opening
            translate([0, 0, -tf_shroud_h])
                difference() {
                    rounded_plate(tf_open_l + 2*tf_shroud_w,
                                  tf_open_w + 2*tf_shroud_w, tf_shroud_h, 2);
                    translate([0, 0, -eps])
                        rounded_plate(tf_open_l, tf_open_w, tf_shroud_h + 2*eps, 1);
                }

            // End mounting tabs
            for (s = [-1, 1])
                translate([s * tab_x, 0, 0])
                    rounded_plate(tf_tab_l, tf_w - 4, tf_t, 2);
        }

        // Central optical opening through the frame
        rect_cut(tf_open_l, tf_open_w, tf_t, 1);

        // 4 board mount holes (M2, 20.1 x 12.4)
        bolt_grid(sb_mx, sb_my, sb_hole_d, tf_t);

        // Adjustable mounting slots in the end tabs
        for (s = [-1, 1])
            translate([s * tab_x, 0, 0]) slot_cut(tf_tab_slot, m3_clear, tf_t);

        // XSHUT + STEMMA wire notch
        translate([0, tf_w/2 - 1, 0]) rect_cut(tf_wire_w, 6, tf_t, 1);
    }
}

tof_bracket();

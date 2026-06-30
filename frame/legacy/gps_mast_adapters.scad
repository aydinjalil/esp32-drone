// Part 8b — GPS Mast Adapters (carbon-tube mast, recommended over a printed mast)
// Bottom adapter bolts to the top plate's rear mast holes and sockets the tube;
// top adapter caps the tube and mounts the GPS plate. Both printed together.
// Use a 10mm OD carbon tube cut to your desired mast height (~80mm).
include <../params.scad>
use <../lib/util.scad>

collar_d = gps_tube_d + 2 * gps_tube_wall;

// Bottom adapter: base + upward collar, tube sockets in from the top.
module mast_adapter_bottom() {
    difference() {
        union() {
            rounded_plate(gps_adapter_base_l, gps_adapter_base_w, gps_adapter_base_t, 2);
            translate([0, 0, gps_adapter_base_t])
                cylinder(d = collar_d, h = gps_adapter_collar_h);
        }
        // Tube socket (blind: leaves a 1mm floor in the base)
        translate([0, 0, 1])
            cylinder(d = gps_tube_d, h = gps_adapter_base_t + gps_adapter_collar_h);
        // Base mount holes to the top plate
        bolt_pair(gps_mast_sep, m3_clear, gps_adapter_base_t);
    }
}

// Top adapter: downward collar + top flat, tube enters from below.
module mast_adapter_top() {
    difference() {
        union() {
            cylinder(d = collar_d, h = gps_adapter_collar_h);
            translate([0, 0, gps_adapter_collar_h])
                rounded_plate(gps_adapter_base_l, gps_adapter_base_w, gps_adapter_base_t, 2);
        }
        // Tube socket from below (blind: 1mm cap under the flat)
        translate([0, 0, -eps])
            cylinder(d = gps_tube_d, h = gps_adapter_collar_h);
        // Holes to bolt the GPS plate on top
        translate([0, 0, gps_adapter_collar_h])
            bolt_pair(gps_mast_sep, m3_clear, gps_adapter_base_t);
    }
}

mast_adapter_bottom();
translate([0, gps_adapter_base_l + 5, 0]) mast_adapter_top();

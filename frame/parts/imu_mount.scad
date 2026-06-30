// Isolated IMU Mount — carries the ICM-20948, mounts on the frame's 30.5 stack.
// The board sits raised on a central TPU pad (separate part), so every edge
// stays open: STEMMA QT ports on the ends, pin headers + the SDO->3V3 address
// jumper on the sides all remain reachable. The soft pad decouples the gyro
// from motor vibration and keeps it away from the ESP32's EMI.
include <../params.scad>
use <../lib/util.scad>

module imu_mount() {
    difference() {
        rounded_plate(im_l, im_w, im_t, im_r);

        // 4x M3 to the frame's 30.5 standoffs
        bolt_grid(im_mount, im_mount, m3_clear, im_t);

        // Central recess that locates the TPU damping pad
        pocket(imu_pad_l + 1, imu_pad_w + 1, imu_pad_seat, im_t, 1);

        // Optional 4-corner retention holes matching the ICM (light nylon screws
        // or small grommets); primary mount is the TPU pad + VHB tape
        bolt_grid(sb_mx, sb_my, sb_hole_d, im_t);

        // Wire-routing notch for the STEMMA cable / SDO jumper to exit the board
        translate([0, im_w/2 - 1, 0]) rect_cut(imu_wire_w, 6, im_t, 1);
    }
}

imu_mount();

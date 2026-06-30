// Part 6b — IMU Vibration Mount, Floating Plate
// 30 x 30 x 2. Carries the ICM-20948 (VHB tape) and hangs on the 4 silicone
// damper balls (25 x 25 pattern) above the lower plate. Decoupling the IMU this
// way targets the motor-EMI / vibration risks noted in DEVLOG.md.
include <../params.scad>
use <../lib/util.scad>

module imu_floating_plate() {
    difference() {
        rounded_plate(imu_floating, imu_floating, imu_mount_t, slot_corner_r);

        // Damper-ball holes (shared pattern with the lower plate)
        bolt_grid(damper_pattern, damper_pattern, damper_hole, imu_mount_t);
    }
}

imu_floating_plate();

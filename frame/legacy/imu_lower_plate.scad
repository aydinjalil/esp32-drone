// Part 6a — IMU Vibration Mount, Lower (Damper) Plate
// 35 x 35 x 2. Bolts to the stack; carries the 4 silicone damper balls on a
// 25 x 25 pattern. Floating plate (Part 6b) sits above on the same dampers.
include <../params.scad>
use <../lib/util.scad>

module imu_lower_plate() {
    difference() {
        rounded_plate(imu_lower, imu_lower, imu_mount_t, slot_corner_r);

        // Damper-ball holes
        bolt_grid(damper_pattern, damper_pattern, damper_hole, imu_mount_t);

        // 2x M3 to bolt the plate down to the stack
        bolt_pair(imu_stack_sep, m3_clear, imu_mount_t);
    }
}

imu_lower_plate();

// IMU TPU Damping Pad — print in flexible TPU (~95A).
// Sits in the IMU mount's central recess; the ICM-20948 VHB-tapes to its top.
// Soft TPU decouples the IMU from frame/motor vibration (the standard FC trick),
// and being smaller than the board it leaves the STEMMA ports / pins exposed.
include <../params.scad>
use <../lib/util.scad>

module imu_tpu_pad() {
    rounded_plate(imu_pad_l, imu_pad_w, imu_pad_h, 2);
}

imu_tpu_pad();

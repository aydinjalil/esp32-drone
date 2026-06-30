// Assembly preview — LAYOUT B: central PDB+ESP tower, IMU at the front bay.
// VISUAL FIT/SCALE CHECK ONLY. Standoff gaps driven by real measurements.
use <parts/esp32_carrier.scad>
use <parts/imu_mount.scad>
use <parts/imu_tpu_pad.scad>
use <parts/tof_bracket.scad>
use <parts/battery_tray.scad>

body = "/Users/aydinjalil/dev/esp32-drone/frame/modified/Main_Body_modified.stl";

// --- centres (mirror modify_body.scad) ---------------------------------------
stack_cx = 10.3; stack_cy = 49;   // central tower: PDB + ESP32 carrier
imu_cx   = 10.3; imu_cy   = 4;    // front bay: isolated IMU
tof_cx   = 10.3; tof_cy   = 4;    // front bay belly: ToF
body_top = 6;                     // body top face
boss_top = 12;                    // body top (6) + boss height (6)
tof_standoff = 8;                 // user's standoffs hang the ToF below the flat belly

// plate thicknesses + the measured ESP32 height
pdb_th    = 1.6;
carrier_t = 2.5;
imu_t     = 2.5;
pb_standoff = 3;       // carrier top -> perfboard bottom (clears header tails)
esp32_h     = 17.38;   // MEASURED: perfboard bottom -> top of USB-C

// --- central tower gaps (each = air gap between two plates) -------------------
g1 = 6;    // boss top -> PDB   (underside solder clearance)
g2 = 8;    // PDB -> carrier    (clears the flat PDB connector)
z_pdb     = boss_top + g1;                  // 18.0
z_carrier = z_pdb + pdb_th + g2;            // 27.6
esp_top_z = z_carrier + carrier_t + pb_standoff + esp32_h;  // ~50.5  (tower's true top)

// --- IMU (front bay) ---------------------------------------------------------
g_imu = 4;                         // boss top -> IMU plate
z_imu = boss_top + g_imu;          // 16.0
// IMU soft-mount part dims (mirror params.scad — assembly uses <use> so vars
// aren't imported): plate -> TPU pad seated 1mm in a recess -> ICM board on VHB.
im_t         = 2.5;                // IMU plate thickness
imu_pad_seat = 1;                  // recess depth that locates the pad
imu_pad_h    = 3;                  // TPU pad height (2mm proud above the plate)
sb_l         = 25.7;              // ICM-20948 board length
sb_w         = 17.7;             // ICM-20948 board width

module pdb() {
    difference() {
        linear_extrude(pdb_th) union() {
            square([35.57, 36.14], center = true);
            translate([0, (36.14/2 + 15.7/2) - 0.2, 0]) square([16.93, 15.7], center = true); // tongue -> rear
        }
        for (x = [-15.25, 15.25], y = [-15.25, 15.25])
            translate([x, y, -0.1]) cylinder(d = 3.2, h = pdb_th + 0.2, $fn = 24);
    }
}

color("DimGray")      import(body);                                              // modified body
// central tower
color("Goldenrod")    translate([stack_cx, stack_cy, z_pdb])     pdb();          // PDB (centered)
color("DarkSeaGreen") translate([stack_cx, stack_cy, z_carrier]) rotate([0,0,90]) esp32_carrier();
%color([0.3,0.3,0.3]) translate([stack_cx, stack_cy, z_carrier + carrier_t + pb_standoff])
                          translate([0,0,esp32_h/2]) cube([25, 55, esp32_h], center=true); // ESP32 envelope
// front bay — IMU soft-mount stack: plate (rigid) -> TPU pad (in recess) -> board (VHB)
imu_plate_top = z_imu + im_t;                       // 18.5
imu_pad_z     = imu_plate_top - imu_pad_seat;       // pad bottom seats 1mm into recess -> 17.5
imu_board_z   = imu_pad_z + imu_pad_h;              // board sits on pad top -> 20.5
color("SteelBlue")    translate([imu_cx, imu_cy, z_imu])     imu_mount();      // isolated IMU plate
color("Tomato")       translate([imu_cx, imu_cy, imu_pad_z]) imu_tpu_pad();    // TPU damping pad (flexible)
%color([0.1,0.4,0.1]) translate([imu_cx, imu_cy, imu_board_z + 0.8])
                          cube([sb_l, sb_w, 1.6], center = true);              // ICM-20948 board (VHB to pad)
color("IndianRed")    translate([tof_cx, tof_cy, -tof_standoff - 2.5]) tof_bracket(); // ToF on standoffs under the bay

// Battery bed mounts DIRECTLY TO THE FRAME on its four EXISTING perimeter
// standoff holes (already in the printed body — no new holes). Tall standoffs
// rise from the body top, beside the electronics, to the tray. The front pair
// clears the carrier; the rear pair passes the carrier's two notched corners.
bed_posts = [[-12, 26.1], [32.1, 26.1], [-9, 76.6], [29.1, 76.6]];
bed_ctr   = [10.05, 51.35];   // mean of the 4 holes = bed center
bed_standoff = 50;            // PURCHASED 50mm M3 standoffs (frame top -> tray)
bed_z     = body_top + bed_standoff;   // tray underside = 56mm
// sanity: bed_z (56) must stay above esp_top_z (~50.5); ~5.5mm clearance.
color("Silver") for (p = bed_posts)
    translate([p[0], p[1], body_top]) cylinder(d = 5, h = bed_z - body_top, $fn = 24);
color("Gainsboro")    translate([bed_ctr[0], bed_ctr[1], bed_z]) battery_tray();
%color([0.2,0.45,0.85]) translate([bed_ctr[0], bed_ctr[1], bed_z + 2.5 + 12.5])
                            cube([43, 132.58, 25], center = true); // pack

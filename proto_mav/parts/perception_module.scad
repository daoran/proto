include <config.scad>
include <lib/frame.scad>
include <lib/mount_holes.scad>
include <lib/spacer.scad>

standoff_w = 9.0;
standoff_h = 4.0;
support_w = 6.0;
support_h = 4.0;

fcu_w = 60.0;
fcu_d = 40.0;
fcu_h = 25.0;
fcu_mount_w = 56.0;
fcu_mount_d = 36.0;
fcu_standoff_h = standoff_h + 3;
fcu_support_h = standoff_h - 2;

nuc_mount_w = 95.0;
nuc_mount_d = 90.4;

stereo_baseline = 70.0;
pitch_frame_w = stereo_baseline + standoff_h * 4 + 30;

module lipo_battery(c=[0.4, 0.4, 0.4]) {
  color(c)
    cube([batt_w, batt_d, batt_h], center=true);
}

module encoder_board() {
  w = 22;
  d = 28;
  h = 1.5;

  chip_w = 5;
  chip_d = 4;
  chip_h = 0.5;

  hole_w = 2.5;
  mount_w = 18;
  mount_d = 11;

  offset_y = d / 2;

  translate([0, -offset_y / 2, 0])
  difference() {
    union() {
      // Body
      color([0, 1, 0])
        translate([0, 0, h / 2])
          cube([w, d, h], center=true);

      // Encoder chip
      color([0, 0, 0])
        translate([0, d / 2 - 7, h])
          cube([chip_w, chip_d, chip_h], center=true);
    }

    // Mount holes
    translate([mount_w / 2, (mount_d + offset_y) / 2, h / 2])
      cylinder(r=hole_w / 2, h=h + 0.1, center=true);
    translate([-mount_w / 2, (mount_d + offset_y) / 2, h / 2])
      cylinder(r=hole_w / 2, h=h + 0.1, center=true);
    translate([mount_w / 2, (-mount_d + offset_y) / 2, h / 2])
      cylinder(r=hole_w / 2, h=h + 0.1, center=true);
    translate([-mount_w / 2, (-mount_d + offset_y) / 2, h / 2])
      cylinder(r=hole_w / 2, h=h + 0.1, center=true);

    // Solder holes
    translate([-(7 * 2.54) / 2, -d / 2 + 1.5, 0])
      for (i = [0:7])
        translate([i * 2.54, 0, h / 2])
          cylinder(r=0.5, h=h + 0.1, center=true);
  }
}

module nut_tool() {
  tol = 0.2;

  // M3 size
  tool_h = 25.0;
  screw_size = 3.0;
  screw_hsize = screw_size / 2.0;
  nut_w = 6.5;

  // M2 size
  // tool_h = 20.0;
  // screw_size = 2.0;
  // screw_hsize = screw_size / 2.0;
  // nut_w = 4.7;

  wing_w = 35.0;
  wing_d = 8.0;
  wing_h = 5.0;

  translate([0.0, 0.0, tool_h / 2.0]) {
    difference() {
      // Tool body
      union() {
        cylinder(h=tool_h, r=screw_hsize + 2.5, center=true);
        translate([0.0, 0.0, -tool_h / 2.0 + wing_h / 2.0])
        cube([wing_w, wing_d, wing_h], center=true);
      }

      // Tool hole
      cylinder(h=tool_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
    }
  }
}

module m12_lens_mount() {
  w = 16;
  base_h = 4.5;
  mount_color = [0.3, 0.3, 0.3];
  flange_w = 18.0;

  difference() {
    union() {
      // Base
      translate([0.0, 0.0, base_h / 2.0])
        color(mount_color)
          cube([w, w, base_h], center=true);

      // Lens thread body
      translate([0.0, 0.0, base_h + 10.0 / 2.0])
        color(mount_color)
          cylinder(h=10, r=7.5, center=true);
    }

    // Lens thread hole
    translate([0.0, 0.0, base_h / 2.0 + 10.0 / 2.0])
      color(mount_color)
        cylinder(h=14.5 + 0.01, r=6, center=true);
  }

  difference() {
    // Lens mount body
    union() {
      translate([0.0, flange_w / 2.0, base_h / 2.0])
        color(mount_color)
          cylinder(h=base_h, r=4.2 / 2.0, center=true);

      translate([0.0, flange_w / 2.0 - 3.0 / 2.0, base_h / 2.0])
        color(mount_color)
          cube([4.2, 3.0, base_h], center=true);
    }

    // Lens mount hole
    translate([0.0, flange_w / 2.0, base_h / 2.0])
      color(mount_color)
        cylinder(h=base_h + 0.01, r=2.0 / 2.0, center=true);
  }

  difference() {
    // Lens mount body
    union() {
      translate([0.0, -flange_w / 2.0, base_h / 2.0])
        color(mount_color)
          cylinder(h=base_h, r=4.2 / 2.0, center=true);

      translate([0.0, -flange_w / 2.0 + 3.0 / 2.0, base_h / 2.0])
        color(mount_color)
          cube([4.2, 3.0, base_h], center=true);
    }

    // Lens mount hole
    translate([0.0, -flange_w / 2.0, base_h / 2.0])
      color(mount_color)
        cylinder(h=base_h + 0.01, r=2.0 / 2.0, center=true);
  }
}

module board_camera() {
  pcb_width = 30.0;
  pcb_thickness = 1.58;
  pcb_hole = 2.1;
  mount_w = 24.5;
  lens_hole = 2.0;

  difference() {
    // PCB body
    translate([0.0, 0.0, pcb_thickness / 2.0])
      color([0.0, 1.0, 0.0])
        cube([pcb_width, pcb_width, pcb_thickness], center=true);

    // Lens holes
    translate([0.0, 9.0, pcb_thickness / 2.0])
      cylinder(h=pcb_thickness + 0.01, r=lens_hole / 2.0, center=true);
    translate([0.0, -9.0, pcb_thickness / 2.0])
      cylinder(h=pcb_thickness + 0.01, r=lens_hole / 2.0, center=true);


    // Mount holes
    for (i = [1 : 4]) {
      rotate([0.0, 0.0, i * 90.0])
        translate([mount_w / 2.0, mount_w / 2.0, pcb_thickness / 2.0])
          cylinder(h=pcb_thickness + 0.01, r=pcb_hole / 2.0, center=true);
    }
  }

  // Lens mount
  translate([0.0, 0.0, pcb_thickness-0.01])
    m12_lens_mount();
}

module gimbal_imu() {
  tol = 0.01;
  w = 20.0;
  d = 18.0;
  h = 4.0;
  mount_w = 14.0;

  difference() {
    // Body
    color([0, 1, 0]) translate([0, 0, h / 2]) cube([w, d, h], center=true);

    // Mount holes
    translate([d / 2 - 7.5, mount_w / 2, h / 2])
      cylinder(r=M2_screw_w / 2, h=h + tol, center=true);
    translate([d / 2 - 7.5, -mount_w / 2, h / 2])
      cylinder(r=M2_screw_w / 2, h=h + tol, center=true);
  }
}

module gimbal_imu_frame(show_imu=1) {
  tol = 0.1;

  frame_w = 0.0;
  frame_d = 0.0;
  mount_w = 16.0;
  mount_standoff_w = 6.0;
  mount_standoff_h = 10.0;
  mount_support_w = 4.0;
  mount_support_h = 4.0;

  imu_w = 20.0;
  imu_d = 18.0;
  imu_h = 4.0;
  imu_mount_w = 14.0;
  imu_standoff_w = 6.0;
  imu_standoff_h = 8.0;

  // Show IMU
  if (show_imu) {
    translate([-1.5, 0, imu_standoff_h / 2])
      gimbal_imu();
  }

  difference() {
    union() {
      // IMU frame
      rotate([180, 0, 0])
        frame(mount_w, mount_w,
              M2_screw_w, M2_nut_w, M2_nut_h,
              mount_standoff_w, mount_standoff_h,
              mount_support_w, mount_support_h);

      // IMU mount standoffs
      translate([0, imu_mount_w / 2, 0])
        cylinder(r=imu_standoff_w / 2, h=imu_standoff_h, center=true);
      translate([0, -imu_mount_w / 2, 0])
        cylinder(r=imu_standoff_w / 2, h=imu_standoff_h, center=true);
    }

    // IMU mount holes
    translate([0, imu_mount_w / 2, 0])
      cylinder(r=M2_screw_w / 2, h=imu_standoff_h + tol, center=true);
    translate([0, -imu_mount_w / 2, 0])
      cylinder(r=M2_screw_w / 2, h=imu_standoff_h + tol, center=true);
    // -- Nut counter sink
    translate([0, imu_mount_w / 2, -imu_h + M2_nut_h / 2])
      cylinder(r=M2_nut_w / 2, h=M2_nut_h + tol, $fn=6, center=true);
    translate([0, -imu_mount_w / 2, -imu_h + M2_nut_h / 2])
      cylinder(r=M2_nut_w / 2, h=M2_nut_h + tol, $fn=6, center=true);
  }
}

module fcu_frame(show_fcu=0) {
  if (show_fcu) {
    translate([0.0, 0.0, fcu_standoff_h])
      color([0.0, 1.0, 0.0])
        difference() {
          translate([0, 0, fcu_h / 2])
            cube([fcu_w, fcu_d, fcu_h], center=true);

          translate([fcu_mount_w / 2, fcu_mount_d / 2, fcu_h / 2])
            cylinder(r=M2_screw_w / 2, h=fcu_h + 0.1, center=true);
          translate([-fcu_mount_w / 2, fcu_mount_d / 2, fcu_h / 2])
            cylinder(r=M2_screw_w / 2, h=fcu_h + 0.1, center=true);
          translate([fcu_mount_w / 2, -fcu_mount_d / 2, fcu_h / 2])
            cylinder(r=M2_screw_w / 2, h=fcu_h + 0.1, center=true);
          translate([-fcu_mount_w / 2, -fcu_mount_d / 2, fcu_h / 2])
            cylinder(r=M2_screw_w / 2, h=fcu_h + 0.1, center=true);
        }
  }

  difference() {
    union() {
      // Mount point
      frame(30.5, 30.5,
            M3_screw_w, M3_nut_w, M3_nut_h,
            standoff_w, standoff_h -2, standoff_h -2);

      // FCU frame
      frame(fcu_mount_w, fcu_mount_d,
            M2_screw_w, M2_nut_w, M2_nut_h,
            standoff_w - 2.5, fcu_standoff_h, fcu_support_h, 0, 1);

      // Battery frame
      rotate(90)
      frame(batt_frame_d, batt_frame_w, M3_screw_w, M3_nut_w, M3_nut_h,
            standoff_w, standoff_h, fcu_support_h, 0, 1);
    }

    // Holes
    for (i = [0:90:360])
      rotate([0, 0, i])
        translate([30.5 / 2, 30.5 / 2])
          cylinder(r=3.2 / 2.0, h=standoff_h + 0.1);
  }

  // // Fill in the gaps
  // translate([0.0, fcu_mount_d / 2 + 3, standoff_h / 2.0])
  //   cube([fcu_mount_w - M2_screw_w, standoff_w / 2.0, standoff_h], center=true);
  // translate([0.0, -fcu_mount_d / 2 - 3, standoff_h / 2.0])
  //   cube([fcu_mount_w - M2_screw_w, standoff_w / 2.0, standoff_h], center=true);
}

module battery_frame(mount_w, mount_d, show_battery=0) {
  // nb_supports = 4;
  // diff = batt_frame_d / (nb_supports + 1);

  // Lipo battery
  if (show_battery) {
    translate([0.0, 0.0, batt_h / 2.0 + standoff_h])
      rotate([0.0, 0.0, 90.0])
      lipo_battery();
  }

  // Frame
  difference() {
    union() {
      // Battery frame
      frame(batt_frame_w, batt_frame_d, M3_screw_w, M3_nut_w, M3_nut_h,
            standoff_w + 1, standoff_h, standoff_h, 0);

      // Supports
      translate([0, 9, standoff_h / 2])
        cube([batt_frame_w, 3.0, standoff_h], center=true);
      translate([0, -9, standoff_h / 2])
        cube([batt_frame_w, 3.0, standoff_h], center=true);
      translate([0, 18, standoff_h / 2])
        cube([batt_frame_w, 3.0, standoff_h], center=true);
      translate([0, -18, standoff_h / 2])
        cube([batt_frame_w, 3.0, standoff_h], center=true);
      // for (spacing = [diff:diff:batt_frame_d-diff]) {
      //   translate([0, -batt_frame_d / 2 + spacing, standoff_h / 2])
      //     #cube([batt_frame_w, 3.0, standoff_h], center=true);
      // }

      // Battery strap support
      translate([batt_frame_w / 2 - 6, 0, standoff_h / 2])
        cube([3.0, 16, standoff_h], center=true);
      translate([-batt_frame_w / 2 + 6, 0, standoff_h / 2])
        cube([3.0, 16, standoff_h], center=true);

      // Overhangs
      hang_w = 30;
      hang_t = 1.5;
      hang_s = 5;
      translate([0, batt_frame_d / 2 + hang_s, standoff_h / 2])
        cube([hang_w, hang_t, standoff_h], center=true);
      translate([hang_w / 2 - hang_t / 2, batt_frame_d / 2 + hang_s / 2, standoff_h / 2])
        cube([hang_t, hang_s, standoff_h], center=true);
      translate([-hang_w / 2 + hang_t / 2, batt_frame_d / 2 + hang_s / 2, standoff_h / 2])
        cube([hang_t, hang_s, standoff_h], center=true);

      translate([0, -batt_frame_d / 2 - hang_s, standoff_h / 2])
        cube([hang_w, hang_t, standoff_h], center=true);
      translate([hang_w / 2 - hang_t / 2, -batt_frame_d / 2 - hang_s / 2, standoff_h / 2])
        cube([hang_t, hang_s, standoff_h], center=true);
      translate([-hang_w / 2 + hang_t / 2, -batt_frame_d / 2 - hang_s / 2, standoff_h / 2])
        cube([hang_t, hang_s, standoff_h], center=true);
    }

    // translate([batt_frame_w / 2 - 3, 0, standoff_h / 2])
    //   #cube([3.0, 15, standoff_h + 0.1], center=true);
    // translate([-batt_frame_w / 2 + 3, 0, standoff_h / 2])
    //   cube([3.0, 15, standoff_h + 0.1], center=true);
  }
}

module encoder_frame(show_encoder=1, show_motor=0) {
  mount_w = 18;
  mount_d = 11;

  motor_standoff_w = 5;
  motor_standoff_h = 10; // Roll and Yaw motor
  // motor_standoff_h = 7; // Pitch motor

  encoder_standoff_w = 7;
  encoder_standoff_h = 4; // Roll and Yaw motor

  support_w = 4;
  support_h = 3.5;

  motor_mount_d = 20.0;

  if (show_motor) {
    rotate(45)
      translate([0, 0, motor_standoff_h])
        gimbal_motor();
  }

  difference() {
      union() {
      // Show encoder
      if (show_encoder) {
        translate([0, 0, encoder_standoff_h + 0.01]) encoder_board();
      }

      // Encoder frame
      frame(mount_w, mount_d,
            M2_screw_w, M2_nut_w, M2_nut_h,
            encoder_standoff_w, encoder_standoff_h,
            support_w, support_h);

      // Motor frame
      rotate(45)
        frame(motor_mount_d, motor_mount_d,
              M2_screw_w, M2_nut_w, M2_nut_h,
              motor_standoff_w, motor_standoff_h,
              support_w, support_h,
              disable=[2]);
    }

    // Mount holes
    hole_positions = [
      [mount_w / 2, mount_d / 2, encoder_standoff_h / 2],
      [mount_w / 2, -mount_d / 2, encoder_standoff_h / 2],
      [-mount_w / 2, mount_d / 2, encoder_standoff_h / 2],
      [-mount_w / 2, -mount_d / 2, encoder_standoff_h / 2]
    ];
    for (hole_pos = hole_positions) {
      translate(hole_pos) {
        cylinder(r=M2_screw_w / 2, h=encoder_standoff_h + 0.1, center=true);

        translate([0, 0, -encoder_standoff_h / 2 + M2_nut_h / 2])
          cylinder(r=M2_nut_w / 2, h=M2_nut_h + 0.1, $fn=6, center=true);
      }
    }
  }
}

module landing_frame(w, d) {
  leg_w = 5.0;
  leg_l = 40.0;
  leg_h = 5.0;

  difference() {
    union() {
      // Frame mount
      frame(w, d,
            M2_screw_w, M2_nut_w, M2_nut_h,
            standoff_w, standoff_h, standoff_w, standoff_h);

      // Feet
      N = sqrt(leg_w * leg_w + leg_w * leg_w);
      feet_h = 5.0;
      feet_positions = [
        [w / 2 + N, d / 2 + N, feet_h / 2, 45],
        [w / 2 + N, -d / 2 - N, feet_h / 2, -45],
        [-w / 2 - N, d / 2 + N, feet_h / 2, -45],
        [-w / 2 - N, -d / 2 - N, feet_h / 2, 45]
      ];
      for (feet_pos = feet_positions) {
        x = feet_pos[0];
        y = feet_pos[1];
        z = feet_pos[2];
        rotz = feet_pos[3];
        translate([x, y, z]) {
          rotate([0, 0, rotz])
          cube([leg_l, leg_w, feet_h], center=true);
        }
      }
    }

    // Counter sink holes
    positions = [
      [w / 2, d / 2, standoff_h / 4],
      [-w / 2, d / 2, standoff_h / 4],
      [w / 2, -d / 2, standoff_h / 4],
      [-w / 2, -d / 2, standoff_h / 4]
    ];
    for (pos = positions) {
      translate(pos) {
        cylinder(r=M2_screw_w / 2, h=10.0, center=true);
        cylinder(r=4/2, h=standoff_h / 2 + 0.01, center=true);
      }
    }

  }
}

module landing_feet() {
  feet_l = 30.0;
  feet_w = 10.0;
  feet_h = 8.0;
  support_h = 20.0 - 4;

  difference() {
    union() {
      // Support
      translate([(feet_l - feet_w) / 2, 0, support_h / 2])
        cylinder(r=feet_w / 2, h=support_h, center=true);

      // Feet
      translate([0, 0, feet_h / 2])
        cube([feet_l, feet_w, feet_h], center=true);
    }

    // Support hole
    translate([(feet_l - feet_w) / 2, 0, support_h / 2])
      cylinder(r=M3_screw_w / 2, h=support_h + 0.1, center=true);
    translate([(feet_l - feet_w) / 2, 0, (M3_caphead_h + 1) / 2])
      cylinder(r=M3_caphead_w / 2, h=(M3_caphead_h + 1) + 0.1, center=true);
  }
}

module odroid_frame(mount_w, mount_d, show_odroid=0) {
  // Show Odroid XU4
  if (show_odroid) {
    color([1, 0.0, 0.0])
      rotate([90.0, 0.0, 90.0])
        translate([-odroid_d / 2, 5.2, -odroid_w / 2])
          import("../proto_parts/Odroid_XU4/Odroid_XU4.STL");
  }

  h = 4.0;
  x = odroid_mount_w / 2.0;
  y = odroid_mount_d / 2.0;
  z = standoff_h + h / 2.0;
  odroid_mount_positions = [[x, y, z], [x, -y, z], [-x, y, z], [-x, -y, z]];
  mount_positions = [
    [mount_w / 2.0, mount_d / 2.0, standoff_h / 2.0],
    [mount_w / 2.0, -mount_d / 2.0, standoff_h / 2.0],
    [-mount_w / 2.0, mount_d / 2.0, standoff_h / 2.0],
    [-mount_w / 2.0, -mount_d / 2.0, standoff_h / 2.0]
  ];

  difference() {
    union() {
      // Mount frame
      frame(mount_w, mount_d,
            M3_screw_w, M3_nut_w, M3_nut_h,
            standoff_w, standoff_h, standoff_w, standoff_h, 0, 0);

      // Odroid frame
      frame(odroid_mount_w, odroid_mount_d,
            M3_screw_w, M3_nut_w, M3_nut_h,
            standoff_w, standoff_h, standoff_w, standoff_h, 0, 1);

      // Mount supports
      for (pos = odroid_mount_positions)
        translate(pos)
          cylinder(r=(M3_screw_w + 2.0) / 2, h=h, center=true);
    }

    // Mount holes
    for (pos = mount_positions) {
      translate(pos)
        cylinder(r=M3_screw_w / 2, h=20, center=true);
    }

    // Odroid mount holes
    for (pos = odroid_mount_positions) {
      translate(pos)
        cylinder(r=M3_screw_w / 2, h=20, center=true);
    }
  }
}

module stereo_camera_frame(show_cameras=1) {
  mount_w = 18.5;
  baseline = stereo_baseline;
  camera_mount_w = 24.5;

  // Mount frame
  frame(mount_w, mount_w,
        M2_screw_w, M2_nut_w, M2_nut_h,
        standoff_w - 3, standoff_h, standoff_w - 3, standoff_h);

  // // Mount frame
  // frame(0.0, 30.0,
  //       M2_screw_w, M2_nut_w, M2_nut_h,
  //       standoff_w - 3, standoff_h, standoff_w - 3, standoff_h,
  //       0, 1);

  // Board-cameras
  if (show_cameras) {
    translate([0.0, baseline / 2.0, standoff_h + 3.0]) board_camera();
    translate([0.0, -baseline / 2.0, standoff_h + 3.0]) board_camera();
  }

  // Board camera frames
  translate([0.0, baseline / 2.0, 0.0])
    frame(camera_mount_w, camera_mount_w,
          M2_screw_w, M2_nut_w, M2_nut_h,
          standoff_w - 3, standoff_h, standoff_w - 3, standoff_h,
          0, 1);
  translate([0.0, -baseline / 2.0, 0.0])
    frame(camera_mount_w, camera_mount_w,
          M2_screw_w, M2_nut_w, M2_nut_h,
          standoff_w - 3, standoff_h, standoff_w - 3, standoff_h,
          0, 1);

  // Board camera standoffs
  translate([0.0, baseline / 2.0, standoff_h + 3 / 2]) {
    mount_w = 24.5;
    x = mount_w / 2.0;
    y = mount_w / 2.0;
    z = 0.0;
    positions = [[x, y, z], [x, -y, z], [-x, y, z], [-x, -y, z]];
    for (pos = positions) {
      translate(pos) {
        difference() {
          cylinder(r=2.0, h=3.0, center=true);
          cylinder(r=M2_screw_w / 2.0, h=3.0 + 0.1, center=true);
        }
      }
    }
  }
  translate([0.0, -baseline / 2.0, standoff_h + 3 / 2]) {
    mount_w = 24.5;
    x = mount_w / 2.0;
    y = mount_w / 2.0;
    z = 0.0;
    positions = [[x, y, z], [x, -y, z], [-x, y, z], [-x, -y, z]];
    for (pos = positions) {
      translate(pos) {
        difference() {
          cylinder(r=2.0, h=3.0, center=true);
          cylinder(r=M2_screw_w / 2.0, h=3.0 + 0.1, center=true);
        }
      }
    }
  }

  // Supports
  translate([camera_mount_w / 2, 0.0, standoff_h / 2])
    cube([3.0, 40.5, standoff_h], center=true);
  translate([-camera_mount_w / 2, 0.0, standoff_h / 2])
    cube([3.0, 40.5, standoff_h], center=true);
}

module usb_hub_board() {
  board_w = 29;
  board_d = 19;
  board_h = 5;
  mount_w = 18;

  difference() {
    union() {
      // Body
      color([0, 1, 0])
        translate([0, 0, board_h / 2])
          cube([board_w, board_d, board_h], center=true);
    }

    // Mount holes
    translate([board_w / 2 - 2, 0, board_h / 2])
      cylinder(r=M2_screw_w / 2, h=board_h + 0.01, center=true);
    translate([-board_w / 2 + 2.5, mount_w / 2, board_h / 2])
      cylinder(r=M2_screw_w / 2, h=board_h + 0.01, center=true);
    translate([-board_w / 2 + 2.5, -mount_w / 2, board_h / 2])
      cylinder(r=M2_screw_w / 2, h=board_h + 0.01, center=true);
  }
}

module sbgc_board() {
  difference() {
    // Body
    color([0.0, 1.0, 0.0])
      translate([0, 0, sbgc_h / 2])
        cube([sbgc_w, sbgc_d, sbgc_h], center=true);

    // Holes
    for (i = [1:4]) {
      rotate(90 * i)
        translate([sbgc_mount_w / 2, sbgc_mount_d / 2, sbgc_h / 2])
          cylinder(r=M3_screw_w / 2, h=sbgc_h + 0.01, center=true);
    }
  }
}

module sbgc_frame(show_sbgc=1, show_usb_hub=1) {
  motor_mount_d = 20.0;
  mount_standoff_w = 8.0;
  mount_standoff_h = 4.0;
  mount_support_w = 4.0;
  mount_support_h = 4.0;

  sbgc_offset_x = 0.0;
  sbgc_standoff_w = 7.0;
  sbgc_standoff_h = 8.0;
  sbgc_support_w = 4.0;
  sbgc_support_h = 4.0;
  sbgc_support_x = abs(sbgc_offset_x) / 2;

  hub_offset_y = -40;
  hub_w = 29;
  hub_d = 19;
  hub_h = 5;
  hub_mount_w = 18;
  hub_support_w = -1 * hub_offset_y - sbgc_mount_w / 2;
  hub_standoff_h = 8;

  difference() {
    union() {
      // Mount Frame
      rotate(45)
        frame(motor_mount_d, motor_mount_d,
              M3_screw_w, M3_nut_w, M3_nut_h,
              mount_standoff_w, mount_standoff_h,
              mount_support_w, mount_support_h);

      // USB Hub board
      if (show_usb_hub) {
        translate([0, hub_offset_y, hub_standoff_h])
          usb_hub_board();
      }

      // USB Hub board frame
      // -- Standoffs
      translate([hub_w / 2 - 1, hub_offset_y, hub_standoff_h / 2])
        cylinder(r=M2_screw_w / 2 + 2, h=hub_standoff_h, center=true);
      translate([-hub_w / 2 + 2.5, hub_offset_y - hub_d / 2 + 0.5, hub_standoff_h / 2])
        cylinder(r=M2_screw_w / 2 + 2, h=hub_standoff_h, center=true);
      translate([-hub_w / 2 + 2.5, hub_offset_y + hub_d / 2 - 0.5, hub_standoff_h / 2])
        cylinder(r=M2_screw_w / 2 + 2, h=hub_standoff_h, center=true);
      // -- Supports
      translate([hub_w / 2 - 1, hub_offset_y + hub_support_w / 2, mount_support_h / 2])
        cube([mount_support_w, hub_support_w, mount_support_w], center=true);
      translate([-hub_w / 2 + 2.5, hub_offset_y, mount_support_h / 2])
        cube([mount_support_w, hub_mount_w, mount_support_w], center=true);
      translate([-hub_w / 2 + 2.5, hub_offset_y + hub_support_w / 2, mount_support_h / 2])
        cube([mount_support_w, hub_support_w, mount_support_w], center=true);
      translate([0, hub_offset_y, mount_support_h / 2])
        cube([24, mount_support_w, mount_support_w], center=true);

      // Simple BGC
      if (show_sbgc) {
        translate([sbgc_offset_x, 0, sbgc_standoff_h])
          sbgc_board();
      }

      // SBGC frame
      translate([sbgc_offset_x, 0, 0])
        rotate(90)
          frame(sbgc_mount_w, sbgc_mount_d,
                M2_screw_w, 0, 0,
                sbgc_standoff_w, sbgc_standoff_h,
                sbgc_support_w, sbgc_support_h,
                0, 1);

      // Supports
      translate([-sbgc_support_x / 2, sbgc_mount_w / 2, sbgc_support_h / 2])
        cube([sbgc_support_x, sbgc_support_w, sbgc_support_h], center=true);
      translate([-sbgc_support_x / 2, -sbgc_mount_w / 2, sbgc_support_h / 2])
        cube([sbgc_support_x, sbgc_support_w, sbgc_support_h], center=true);
      translate([sbgc_offset_x, 0, sbgc_support_h / 2])
        cube([sbgc_support_w, sbgc_mount_w + sbgc_support_w, sbgc_support_h], center=true);
      translate([sbgc_offset_x, 0, sbgc_support_h / 2])
        cube([sbgc_mount_w + sbgc_support_w, sbgc_support_w, sbgc_support_h], center=true);
      translate([0, 0, sbgc_support_h / 2])
        cube([sbgc_support_w, sbgc_mount_w + sbgc_support_w, sbgc_support_h], center=true);
    }

    // USB Hub mount holes
    // -- Screw mount holes
    translate([hub_w / 2 - 1, hub_offset_y, hub_standoff_h / 2])
      cylinder(r=M2_screw_w / 2, h=hub_standoff_h + 0.01, center=true);
    translate([-hub_w / 2 + 2.5, hub_offset_y - hub_d / 2 + 0.5, hub_standoff_h / 2])
      cylinder(r=M2_screw_w / 2, h=hub_standoff_h + 0.01, center=true);
    translate([-hub_w / 2 + 2.5, hub_offset_y + hub_d / 2 -0.5, hub_standoff_h / 2])
      cylinder(r=M2_screw_w / 2, h=hub_standoff_h + 0.01, center=true);
    // -- Nut counter sinks
    translate([hub_w / 2 - 1, hub_offset_y, M2_nut_h / 2])
      cylinder(r=M2_nut_w / 2, h=M2_nut_h + 0.01, $fn=6, center=true);
    translate([-hub_w / 2 + 2.5, hub_offset_y - hub_d / 2 + 0.5, M2_nut_h / 2])
      cylinder(r=M2_nut_w / 2, h=M2_nut_h + 0.01, $fn=6, center=true);
    translate([-hub_w / 2 + 2.5, hub_offset_y + hub_d / 2 - 0.5, M2_nut_h / 2])
      cylinder(r=M2_nut_w / 2, h=M2_nut_h + 0.01, $fn=6, center=true);

    // SBGC Mount holes
    for (i = [1:4]) {
      translate([sbgc_offset_x, 0.0, 0.0]) {
        rotate(90 * i) {
          translate([sbgc_mount_w / 2, sbgc_mount_d / 2, sbgc_standoff_h / 2])
            cylinder(r=M2_screw_w / 2, h=sbgc_standoff_h + 0.01, center=true);

          translate([sbgc_mount_w / 2, sbgc_mount_d / 2, M2_nut_h / 2])
            cylinder(r=M2_nut_w / 2, h=M2_nut_h + 0.01, $fn=6, center=true);
        }
      }
    }

    // Motor Mount holes
    for (i = [1:4]) {
      rotate(90 * i + 45.0) {
        translate([motor_mount_d / 2, motor_mount_d / 2, sbgc_support_h / 2])
          cylinder(r=M3_screw_w / 2, h=sbgc_support_h + 0.01, center=true);
      }
    }
  }
}


// Gimbal Motor
module gimbal_motor(has_encoders=0) {
  motor_r = 35.0 / 2.0;
  motor_h = (has_encoders) ? 25.0 : 15.0;
  base_mount_d = (has_encoders) ? 20.0 : 29.0;

  color([0.2, 0.2, 0.2])
  difference() {
    union() {
      // Main body
      cylinder(r=motor_r, h=motor_h);

      // Bottom shaft
      translate([0, 0, -3 / 2])
        cylinder(r=14 / 2, h=3, $fn=6, center=true);
    }

    // Top mount holes
    for (i = [45:90:360]) {
      rotate([0.0, 0.0, i])
        translate([19.0 / 2.0, 0.0, motor_h - 2.0 + 0.01])
          cylinder(r=1.0, h=2.0, center=false);
    }

    // Base mount holes
    for (i = [1 : 4]) {
      rotate([0.0, 0.0, i * 90.0 + 45.0])
        translate([base_mount_d / 2.0, 0.0, -0.01])
          cylinder(r=1.0, h=2.0, center=false);
    }

    // Wire hole
    if (has_encoders) {
      translate([-motor_r + 2.5, 0.0, 2.0 - 0.01])
        cube([5.0, 9.0, 4.0], center=true);
    }
  }
}

module gimbal_camera_frame(show_camera=1) {
  camera_mount_w = 24.5;


  frame_w = camera_mount_w + 18;
  frame_d = camera_mount_w + 5;
  frame_h = 6;
  frame_standoff_w = 8;

  rod_hole_w = 36;
  rod_standoff_x = 30.0;
  rod_standoff_w = 6.0;
  rod_standoff_l = 30.0;

  // Show camera
  if (show_camera) {
    translate([0, 0, -10])
      board_camera();
  }

  difference() {
    union() {
      // Camera frame
      translate([0, 0, frame_h / 2]) {
        // Body
        cube([frame_w, frame_d, frame_h], center=true);

        // Camera mount supports
        translate([camera_mount_w / 2, camera_mount_w / 2, 0])
          cylinder(r=frame_standoff_w / 2, h=frame_h, center=true);
        translate([-camera_mount_w / 2, camera_mount_w / 2, 0])
          cylinder(r=frame_standoff_w / 2, h=frame_h, center=true);
        translate([camera_mount_w / 2, -camera_mount_w / 2, 0])
          cylinder(r=frame_standoff_w / 2, h=frame_h, center=true);
        translate([-camera_mount_w / 2, -camera_mount_w / 2, 0])
          cylinder(r=frame_standoff_w / 2, h=frame_h, center=true);
      }
    }

    // Camera hole
    {
      w = camera_mount_w - 4;
      d = camera_mount_w - 4;
      h = frame_h + 0.01;
      translate([0, 0, frame_h / 2]) {
        // cube([w, d, h], center=true);
        cylinder(r=12, h=frame_h + 0.01, center=true);
      }
    }

    // Camera mount holes
    {
      // Screw hole
      mount_holes(camera_mount_w, camera_mount_w, M2_screw_w, frame_h);

      // Nut counter sink
      translate([0, 0, frame_h - M2_nut_h])
        mount_holes(camera_mount_w, camera_mount_w, M2_nut_w, M2_nut_h, fn=6);
    }

    // Rod standoffs
    rod_standoff_positions = [
      [rod_hole_w / 2, 0.0, rod_standoff_w / 2],
      [-rod_hole_w / 2, 0.0, rod_standoff_w / 2]
    ];
    for (pos = rod_standoff_positions) {
      translate(pos) {
        // Rod standoff hole
        rotate([90, 0, 0])
          cylinder(r=M3_screw_w / 2, h=rod_standoff_l + 0.01, center=true);

        // Rod standoff cutout
        cube([6.5 + 0.1, rod_hole_w / 3, frame_h + 0.01], center=true);
      }
    }
  }
}

module gimbal_camera_frame_spacer(base_h, top_h) {
  M3_spacer(spacer_w=M3_screw_w + 5,
            spacer_h=base_h,
            nut_w=M3_nut_w,
            nut_h=M3_nut_h,
            nts=1,
            tcs=0);

  difference() {
    translate([0, 0, base_h + top_h / 2])
      cylinder(r=(M3_screw_w + 2) / 2, h=top_h, center=true);

    translate([0, 0, base_h + top_h / 2])
      cylinder(r=(M3_screw_w + 0.5) / 2, h=top_h + 0.01, center=true);
  }
}

module gimbal_roll_frame(show_roll_motor=1,
                         show_pitch_motor=1,
                         show_pitch_frame=1,
                         show_encoder=1) {
  motor_h = 15.0;
  motor_mount_w = 13.2;
  motor_mount_d = 20.5;
  motor_offset_y = -4;
  motor_offset_z = 40;

  roll_mount_w = 30.0;
  roll_mount_d = 30.0;
  roll_mount_h = 6.0;

  pitch_mount_w = roll_mount_w;
  pitch_mount_d = roll_mount_h;
  pitch_mount_h = 50;
  pitch_mount_offset_y = (35 + pitch_mount_d) / 2;
  pitch_standoff_w = 6;
  pitch_standoff_h = 7.5;

  // Show roll motor
  if (show_roll_motor) {
    translate([0, 0, -motor_h]) gimbal_motor();
  }

  // Show pitch motor
  if (show_pitch_motor) {
    translate([0, motor_offset_y + motor_h / 2, motor_offset_z])
      rotate([90, 45, 0])
        gimbal_motor();
  }

  // Show encoder
  if (show_encoder) {
    translate([0, pitch_mount_offset_y - pitch_mount_d / 2 + motor_offset_y, motor_offset_z])
      rotate([90, 0, 0])
        encoder_frame();
  }

  // Show pitch frame
  if (show_pitch_frame) {
    translate([0, motor_offset_y - motor_h / 2 - 5, motor_offset_z])
      rotate([-90, -90, 0])
      gimbal_pitch_frame();
  }

  // Roll frame
  difference() {
    union() {
      // Roll motor mount
      translate([0, 0, roll_mount_h / 2])
        cylinder(r=motor_mount_d - 2, h=roll_mount_h, center=true);
        // cube([roll_mount_w, roll_mount_d, roll_mount_h], center=true);

      // Roll-Pitch mount connector
      translate([0, pitch_mount_offset_y / 2, roll_mount_h / 2])
        cube([roll_mount_w, pitch_mount_offset_y / 2, roll_mount_h], center=true);

      // Pitch motor mount
      translate([0, pitch_mount_offset_y + motor_offset_y, motor_offset_z])
        rotate([90, 0, 0])
          cylinder(r=motor_mount_d, h=pitch_mount_d, center=true);

      // Pitch mount plate support
      translate([0, pitch_mount_offset_y + motor_offset_y, motor_offset_z / 2])
        cube([roll_mount_w, pitch_mount_d, motor_offset_z], center=true);
    }

    // Roll mount holes
    mount_holes(motor_mount_w, motor_mount_w, M2_screw_w, roll_mount_h);

    // Pitch motor center hole
    translate([0, pitch_mount_offset_y + motor_offset_y, motor_offset_z])
      rotate([90, 0, 0])
        cylinder(r=10, h=pitch_mount_d + 0.01, center=true);

    // Pitch motor holes
    translate([0, pitch_mount_offset_y + motor_offset_y + roll_mount_h / 2, motor_offset_z])
      rotate([90, 45, 0])
        mount_holes(motor_mount_d, motor_mount_d, M2_screw_w, roll_mount_h);

    // Wire management holes
    translate([5, pitch_mount_offset_y + motor_offset_y,, 12])
      rotate([90, 0, 0])
        cylinder(r=5, h=pitch_mount_d + 0.01, center=true);
    translate([-5, pitch_mount_offset_y + motor_offset_y,, 12])
      rotate([90, 0, 0])
        cylinder(r=5, h=pitch_mount_d + 0.01, center=true);
    translate([0, pitch_mount_offset_y + motor_offset_y,, 12])
      cube([10, pitch_mount_d + 0.01, 10], center=true);
  }
}

module gimbal_pitch_frame(show_pitch_motor=0, show_encoder=0, show_cameras=1) {
  tol = 0.6;

  motor_h = 15.0;
  motor_mount_w = 13.2;
  motor_mount_d = 20.5;
  motor_offset_y = -8;
  motor_offset_z = 35;

  pitch_mount_h = 6;
  pitch_standoff_w = 6;
  pitch_standoff_h = 37.5;
  pitch_support_h = pitch_standoff_h;

  rod_offset_x = motor_mount_d + 5;
  rod_standoff_w = 6;
  rod_hole_w = 36;

  support_hole_d = 15;
  support_hole_h = 8;
  support_h = pitch_support_h / 2;

  ext_mount_w = 16;
  stereo_baseline = 75;

  // Show pitch motor
  if (show_pitch_motor) {
    translate([0, 0, -motor_h])
      gimbal_motor();
  }


  // Show camera frame
  if (show_cameras) {
    rotate([0, 90, 0]) {
      translate([-support_h + stereo_baseline / 2, 0, rod_offset_x - 6 / 2])
        rotate(90)
          gimbal_camera_frame(show_camera=1);

      translate([-support_h - stereo_baseline / 2, 0, rod_offset_x - 6 / 2])
        rotate(90)
          gimbal_camera_frame(show_camera=1);
    }
  }

  // Pitch frame
  difference() {
    union() {
      // Pitch motor mount
      translate([0, 0, pitch_mount_h / 2])
        cylinder(r=motor_mount_d, h=pitch_mount_h, center=true);

      // Rod standoff
      translate([rod_offset_x, rod_hole_w / 2, pitch_standoff_h / 2])
        cube([rod_standoff_w, rod_standoff_w, pitch_standoff_h], center=true);
      translate([rod_offset_x, -rod_hole_w / 2, pitch_standoff_h / 2])
        cube([rod_standoff_w, rod_standoff_w, pitch_standoff_h], center=true);

      // Support
      translate([rod_offset_x, 0, pitch_support_h / 2])
        cube([rod_standoff_w, motor_mount_d * 1.6, pitch_support_h], center=true);
      translate([rod_offset_x / 2, 0, pitch_mount_h / 2])
        cube([rod_offset_x, motor_mount_d * 1.6, pitch_mount_h], center=true);
    }

    // Rod standoff cutout
    translate([rod_offset_x, rod_hole_w / 2, pitch_standoff_h / 2])
      cube([rod_standoff_w + 0.01, rod_standoff_w + 0.01, pitch_standoff_h / 2], center=true);
    translate([rod_offset_x, -rod_hole_w / 2, pitch_standoff_h / 2])
      cube([rod_standoff_w + 0.01, rod_standoff_w + 0.01, pitch_standoff_h / 2], center=true);

    // Pitch motor holes
    mount_holes(motor_mount_w, motor_mount_w, M2_screw_w, pitch_mount_h);

    // Rod holes
    translate([rod_offset_x, rod_hole_w / 2, pitch_standoff_h / 2])
      cylinder(r=M2_screw_w / 2 + tol, h=pitch_standoff_h + 0.01, center=true);
    translate([rod_offset_x, -rod_hole_w / 2, pitch_standoff_h / 2])
      cylinder(r=M2_screw_w / 2 + tol, h=pitch_standoff_h + 0.01, center=true);

    // Support hole
    translate([rod_offset_x, 0, support_h])
      cube([rod_standoff_w + 0.01, support_hole_d, support_hole_h], center=true);
    translate([rod_offset_x, support_hole_d / 2, support_h])
      rotate([0, 90, 0])
        cylinder(r=support_hole_h / 2, h=rod_standoff_w + 0.01, center=true);
    translate([rod_offset_x, -support_hole_d / 2, support_h])
      rotate([0, 90, 0])
        cylinder(r=support_hole_h / 2, h=rod_standoff_w + 0.01, center=true);

    // External mount holes
    translate([rod_offset_x - rod_standoff_w / 2, 0, support_h]) {
      rotate([0, 90, 0]) {
        mount_holes(ext_mount_w,
                    ext_mount_w,
                    M2_screw_w,
                    rod_standoff_w);
        mount_holes(ext_mount_w,
                    ext_mount_w,
                    M2_nut_w,
                    M2_nut_h,
                    fn=6);
      }
    }
  }
}

module gimbal_yaw_frame(show_roll_frame=1, show_sbgc_frame=1) {
  has_encoders = 0;
  motor_h = 15.0;
  motor_mount_w = 13.2;
  motor_mount_d = (has_encoders) ? 14.2 : 20.5;

  standoff_w = 8.0;
  standoff_h = 8.0;

  support_w = 8.0;
  support_h = 8.0;

  encoder_h = 10;
  roll_mount_h = 50;
  roll_mount_d = 45;

  // Show roll frame
  if (show_roll_frame) {
    // rotate([0, 90, 0])
      // translate([-roll_mount_h, 0.0, -roll_mount_d + motor_h + encoder_h - 0.01])
        // gimbal_roll_frame();

    rotate([0, 90, 0])
      translate([-roll_mount_h, 0.0, -roll_mount_d + motor_h + encoder_h - 0.01])
        gimbal_roll_frame();

    rotate([0, 90, 0])
      translate([-roll_mount_h, 0.0, -roll_mount_d])
        rotate([0, 0, 90])
          encoder_frame(1);
  }

  // Show SBGC frame
  if (show_sbgc_frame) {
    rotate([0, -90, 0]) {
      translate([roll_mount_h, 0.0, roll_mount_d + standoff_h]) {
        sbgc_frame();
      }
    }
  }

  // Yaw frame
  difference() {
    union() {
      // Yaw motor mount
      frame(motor_mount_w, motor_mount_w,
            M2_screw_w, M2_nut_w, M2_nut_h,
            standoff_w, standoff_h,
            support_w, support_h);

      // Roll motor mount
      // rotate([0, 90, 0])
      //   translate([-roll_mount_h, 0.0, -roll_mount_d - standoff_h])
            // frame(motor_mount_d, motor_mount_d,
            //       M2_screw_w, M2_nut_w, M2_nut_h,
            //       standoff_w, standoff_h,
            //       support_w, support_h);

      // Horizontal supports
      hx = motor_mount_d / 2 + roll_mount_d + support_h;
      hy = motor_mount_w / 2 + standoff_w;
      translate([motor_mount_d / 2, 0, support_h / 2]) {
        translate([-hx / 2, hy, 0])
          cube([hx, support_w, support_h], center=true);
        translate([-hx / 2, -hy, 0])
          cube([hx, support_w, support_h], center=true);
        translate([-hx + support_w / 2, 0, 0])
          cube([support_w, motor_mount_d + support_w, support_h], center=true);
        translate([-hx + support_w / 2, 0, roll_mount_h - motor_mount_d / 2 - support_h])
          cube([support_w, hy * 2 + support_w, support_h], center=true);
        translate([-hx + support_w / 2, 0, roll_mount_h + motor_mount_d / 2])
          cube([support_w, hy * 2 + support_w, support_h], center=true);
      }
      translate([motor_mount_d / 2, 0, support_h / 2])
        cube([support_w, hy * 2 + support_w, support_h], center=true);
      translate([-motor_mount_d / 2, 0, support_h / 2])
        cube([support_w, hy * 2 + support_w, support_h], center=true);

      // Vertical supports
      vx = roll_mount_d + standoff_h / 2;
      vy = motor_mount_w / 2 + standoff_w;
      vz = roll_mount_h + motor_mount_d / 2 + support_w / 2;
      translate([-vx, vy, vz / 2])
        cube([support_h, support_w, vz], center=true);
      translate([-vx, -vy, vz / 2])
        cube([support_h, support_w, vz], center=true);

      // Diagonal supports
      dl = 50;
      dz = 16;
      dx_offset = 20;
      dx = roll_mount_d + standoff_h / 2 - dx_offset;
      dy = motor_mount_w / 2 + standoff_w;
      translate([-dx, dy, dz])
        rotate([0, -60, 0])
          cube([support_h, support_w, dl], center=true);
      translate([-dx, -dy, dz])
        rotate([0, -60, 0])
          cube([support_h, support_w, dl], center=true);
    }

    // Yaw motor mount holes
    for (i = [1:4]) {
      rotate(90 * i)
        translate([motor_mount_w / 2, motor_mount_w / 2, standoff_h / 2])
          cylinder(r=M2_screw_w / 2, h=support_h + 0.01, center=true);
    }
    translate([0, 0, support_h / 2])
      cylinder(r=10.0 / 2.0, h=support_h + 0.01, center=true);

    // Roll motor mount holes
    rotate([0, 90, 0]) {
      translate([-roll_mount_h, 0.0, -roll_mount_d - standoff_h]) {
        for (i = [1:4]) {
          rotate(90 * i + 45)
            translate([motor_mount_d / 2, motor_mount_d / 2, standoff_h / 2])
              cylinder(r=M2_screw_w / 2, h=standoff_h + 0.01, center=true);
        }

        translate([0, 0, standoff_h / 2])
          cylinder(r=20.0 / 2.0, h=standoff_h + 0.01, center=true);
      }
    }
  }
}

module gimbal_frame(mount_w, mount_d, show_yaw_frame=1) {
  has_encoders = 0;
  motor_h = (has_encoders) ? 25.0 : 15.0;
  motor_mount_d = (has_encoders) ? 14.2 : 20.5;

  roll_mount_w = 30.0;
  roll_mount_h = 50.0;
  encoder_standoff_h = 10;

  sbgc_standoff_w = 5.0;
  sbgc_standoff_base_w = 8.0;
  sbgc_standoff_h = 8.0;

  psu_standoff_w = 5.0;
  psu_standoff_base_w = 8.0;
  psu_standoff_h = 8.0;
  psu_support_w = 3.0;
  psu_support_h = 4.0;

  // Show yaw frame
  if (show_yaw_frame) {
    translate([0, 0, support_h + 0.01])
      encoder_frame();

    translate([0, 0, support_h + encoder_standoff_h +  0.01])
      rotate(45)
        gimbal_motor();

    translate([0, 0, support_h + encoder_standoff_h + motor_h])
      gimbal_yaw_frame();
  }

  difference() {
    union() {
      // Yaw motor mount
      translate([0, 0, support_h / 2])
        cylinder(r=20, h=support_h, center=true);

      // Mount frame
      frame(mount_w, mount_d,
            M3_screw_w, M3_nut_w, M3_nut_h,
            standoff_w, support_h,
            support_w, support_h);

      // // SBGC frame
      // sbgc_holes = [
      //   [sbgc_mount_w / 2,  35.0 + sbgc_mount_d / 2, 0],
      //   [-sbgc_mount_w / 2, 35.0 + -sbgc_mount_d / 2, 0]
      // ];
      // for (pos = sbgc_holes) {
      //   translate(pos) {
      //     translate([0, 0, sbgc_standoff_h / 2])
      //       cylinder(r=sbgc_standoff_w / 2, h=sbgc_standoff_h, center=true);
      //     translate([0, 0, support_h / 2])
      //       cylinder(r=sbgc_standoff_base_w / 2, h=support_h, center=true);
      //   }
      // }

      // // Pololu PSU frame
      // psu_holes = [
      //   [pololu_mount_w / 2,  -32.0 + pololu_mount_d / 2, 0],
      //   [-pololu_mount_w / 2, -32.0 + -pololu_mount_d / 2, 0]
      // ];
      // for (pos = psu_holes) {
      //   rotate(90)
      //   translate(pos) {
      //     translate([0, 0, psu_standoff_h / 2])
      //       cylinder(r=psu_standoff_w / 2, h=psu_standoff_h, center=true);
      //     translate([0, 0, support_h / 2])
      //       cylinder(r=psu_standoff_base_w / 2, h=support_h, center=true);
      //   }
      // }

      // Supports
      translate([motor_mount_d / 2 + 1.5, 0, support_h / 2])
        cube([support_w, mount_d, support_h], center=true);
      translate([-motor_mount_d / 2 - 1.2, 0, support_h / 2])
        cube([support_w, mount_d, support_h], center=true);
      translate([0, motor_mount_d / 2, support_h / 2])
        cube([mount_w, support_w, support_h], center=true);
      translate([0, -motor_mount_d / 2, support_h / 2])
        cube([mount_w, support_w, support_h], center=true);

      // // Pololu supports
      // rotate(90) {
      //   translate([pololu_mount_w / 2 - 5, -32 + pololu_mount_d / 2, support_h / 2])
      //     cube([10, psu_standoff_base_w, support_h], center=true);
      //   translate([-pololu_mount_w / 2 + 5, -32 - pololu_mount_d / 2, support_h / 2])
      //     cube([10, psu_standoff_base_w, support_h], center=true);
      // }
    }

    // Yaw motor center holes
    translate([0, 0, support_h / 2])
      cylinder(r=10, h=support_h + 0.01, center=true);

    // Yaw motor mount holes
    positions = [
      [motor_mount_d / 2, motor_mount_d / 2, support_h / 2],
      [-motor_mount_d / 2, motor_mount_d / 2, support_h / 2],
      [motor_mount_d / 2, -motor_mount_d / 2, support_h / 2],
      [-motor_mount_d / 2, -motor_mount_d / 2, support_h / 2]
    ];
    for (pos = positions) {
      rotate(45)
        translate(pos)
          cylinder(r=M2_screw_w / 2, h=support_h + 0.01, center=true);
    }

    // // SBGC mount holes
    // sbgc_holes = [
    //   [sbgc_mount_w / 2,  35.0 + sbgc_mount_d / 2, 0],
    //   [-sbgc_mount_w / 2, 35.0 + -sbgc_mount_d / 2, 0]
    // ];
    // for (pos = sbgc_holes) {
    //   translate(pos) {
    //     translate([0, 0, sbgc_standoff_h / 2])
    //       cylinder(r=M2_screw_w / 2, h=sbgc_standoff_h + 0.01, center=true);
    //     translate([0, 0, support_h / 2])
    //       cylinder(r=M2_nut_w / 2, h=support_h + 0.01, $fn=6, center=true);
    //   }
    // }

    // // PSU mount holes
    // psu_holes = [
    //   [pololu_mount_w / 2,  -32.0 + pololu_mount_d / 2, 0],
    //   [-pololu_mount_w / 2, -32.0 + -pololu_mount_d / 2, 0]
    // ];
    // for (pos = psu_holes) {
    //   rotate(90) {
    //     translate(pos) {
    //       translate([0, 0, psu_standoff_h / 2])
    //         cylinder(r=M2_screw_w / 2, h=psu_standoff_h + 0.01, center=true);
    //       translate([0, 0, support_h / 2])
    //         cylinder(r=M2_nut_w / 2, h=support_h + 0.01, $fn=6, center=true);
    //     }
    //   }
    // }
  }
}

module pcb_frame(mount_w, mount_d) {
  pcb_mount_w = 66;
  pcb_mount_d = 46;
  pcb_standoff_w = 5;
  pcb_standoff_h = support_h + 4;

  difference() {
    union() {
      // Mount frame
      frame(mount_w, mount_d,
            M3_screw_w, M3_nut_w, M3_nut_h,
            standoff_w, support_h,
            support_w, support_h);

      // PCB frame
      frame(pcb_mount_w, pcb_mount_d,
            M2_screw_w, M2_nut_w, M2_nut_h,
            pcb_standoff_w, pcb_standoff_h,
            support_w, support_h,
            nut_csb=1);

      // Supports
      translate([pcb_mount_w / 2, 0, support_h / 2])
        cube([support_w, mount_d, support_h], center=true);
      translate([-pcb_mount_w / 2, 0, support_h / 2])
        cube([support_w, mount_d, support_h], center=true);
    }

    // Holes
    mount_holes(pcb_mount_w, pcb_mount_d, M2_screw_w, support_h);
    mount_holes(pcb_mount_w, pcb_mount_d, M2_nut_w, M2_nut_h, fn=6);
  }
}

module nuc_frame(mount_w, mount_d, show_nuc=0) {
  nuc_standoff_w = 6;
  nuc_standoff_h = 16;
  nuc_support_w = 6;
  nuc_support_h = 6;

  // Show NUC
  if (show_nuc) {
    color([0.0, 0.0, 1.0])
      translate([0, 0, -28 + nuc_standoff_h])
        rotate([90.0, 0.0, 0.0])
          import("../proto_parts/Intel_NUC7i5DN/NUC7i5DN.STL");
  }

  // NUC frame
  frame(mount_w, mount_d,
        M3_screw_w, M3_nut_w, M3_nut_h,
        nuc_standoff_w, nuc_standoff_h,
        nuc_support_w, nuc_support_h);

  // Supports
  translate([0, 0, nuc_support_h / 2])
    cube([nuc_support_w, mount_d, nuc_support_h], center=true);
  // translate([mount_w / 2 * 0.5, 0, nuc_support_h / 2])
  //   cube([nuc_support_w, mount_d, nuc_support_h], center=true);
  // translate([-mount_w / 2 * 0.5, 0, nuc_support_h / 2])
  //   cube([nuc_support_w, mount_d, nuc_support_h], center=true);

  translate([0, 0, nuc_support_h / 2])
    cube([mount_w, nuc_support_w, nuc_support_h], center=true);
  // translate([0, mount_d / 2 * 0.5, nuc_support_h / 2])
  //   cube([mount_w, nuc_support_w, nuc_support_h], center=true);
  // translate([0, -mount_d / 2 * 0.5, nuc_support_h / 2])
  //   cube([mount_w, nuc_support_w, nuc_support_h], center=true);

  // Cable rails
  rail_offset = 10.0;
  difference() {
    translate([0, (mount_d / 2) + 4, nuc_support_h / 2])
      cube([mount_w - 5, rail_offset, nuc_support_h], center=true);
    translate([0, (mount_d / 2) + 4, nuc_support_h / 2])
      cube([mount_w - 10, 5, nuc_support_h + 0.01], center=true);
  }
  difference() {
    translate([0, -(mount_d / 2) - 4, support_h / 2])
      cube([mount_w - 5, rail_offset, support_h], center=true);
    translate([0, -(mount_d / 2) - 4, support_h / 2])
      cube([mount_w - 10, 5, support_h + 0.01], center=true);
  }
}

module payload_frame(mount_w, mount_d,
                     standoff_w, standoff_h,
                     support_w, support_h,
                     nuc_mount_w, nuc_mount_d) {
  difference() {
    union() {
      // Payload frame
      rotate(90)
        frame(mount_w, mount_d,
              M3_screw_w, M3_nut_w, M3_nut_h,
              support_w, support_h,
              support_w, support_h);

      // NUC frame
      frame(nuc_mount_w, nuc_mount_d,
            M3_screw_w, M3_nut_w, M3_nut_h,
            standoff_w, standoff_h,
            support_w, support_h);
    }

    // Nut countersink
    positions = [
      [mount_w / 2, mount_d / 2, support_h - M3_nut_h / 2],
      [-mount_w / 2, mount_d / 2, support_h - M3_nut_h / 2],
      [mount_w / 2, -mount_d / 2, support_h - M3_nut_h / 2],
      [-mount_w / 2, -mount_d / 2, support_h - M3_nut_h / 2]
    ];
    for (pos = positions) {
      rotate(90)
        translate(pos)
          cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
    }
  }
}

module top_stack(show_fcu=1, show_battery=1) {
  // FCU Frame
  fcu_frame(show_fcu);

  // Battery Frame
  translate([0.0, 0.0, fcu_standoff_h + fcu_h + 0.5])
    battery_frame(batt_frame_w, batt_frame_d, show_battery);
}

module perception_module() {
  rotate([0, 0, 0])
    payload_frame(mav_payload_mount_w, mav_payload_mount_d,
                  mav_frame_standoff_w, mav_payload_standoff_h,
                  mav_frame_support_w, mav_frame_support_h,
                  nuc_mount_w, nuc_mount_d);

  translate([0.0, 0.0, mav_payload_standoff_h])
      nuc_frame(nuc_mount_w, nuc_mount_d, show_nuc=1);

  // translate([0.0, 0.0, mav_payload_standoff_h + 32.0])
  //   rotate([0, 0, 180])
  //     sbgc_frame(nuc_mount_w, nuc_mount_d);

  translate([0, 0, mav_payload_standoff_h + 30.0])
      gimbal_frame(nuc_mount_w, nuc_mount_d);
}


// Assembly Development
// top_stack();
// perception_module();

// Component Development
// spacer(batt_h + 2, nut_counter_sink=1);

// spacer_h = 16; // NUC frame
// spacer_h = 25; // PCB frame
// spacer(spacer_w=9,
//        spacer_h=spacer_h,
//        hole_w=M3_screw_w,
//        nut_w=M3_nut_w,
//        nut_h=M3_nut_h,
//        nts=1);

// battery_frame(batt_frame_w, batt_frame_d);
// fcu_frame(show_fcu);
// encoder_frame(0);
// odroid_frame(nuc_mount_w, nuc_mount_d, 0);
// landing_frame(nuc_mount_w, nuc_mount_d);
// landing_feet();
// stereo_camera_frame();

// usb_hub_board();
// sbgc_board();
// sbgc_frame(show_sbgc=0, show_usb_hub=0);
// gimbal_motor();
// gimbal_imu();
// gimbal_imu_frame();
// gimbal_camera_frame(show_camera=0);
// gimbal_camera_frame_spacer(3.5, 3);
// gimbal_roll_frame(show_roll_motor=0,
//                   show_pitch_motor=1,
//                   show_pitch_frame=1,
//                   show_encoder=1);
// gimbal_pitch_frame(show_pitch_motor=0, show_encoder=0, show_cameras=0);
// gimbal_yaw_frame(show_roll_frame=1, show_sbgc_frame=1);
// gimbal_frame(nuc_mount_w, nuc_mount_d, 1);
// pcb_frame(nuc_mount_w, nuc_mount_d);
// nuc_frame(nuc_mount_w, nuc_mount_d, show_nuc=1);
// payload_frame(mav_payload_mount_w, mav_payload_mount_d,
//               mav_frame_support_w, mav_frame_support_h,
//               mav_frame_support_w, mav_frame_support_h,
//               nuc_mount_w, nuc_mount_d);

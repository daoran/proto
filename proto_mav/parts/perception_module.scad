include <config.scad>
include <lib/frame.scad>
include <lib/mount_holes.scad>
include <lib/motor_mount_holes.scad>
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

// Generic Board Camera
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

// Arducam MT9V034
module arducam_MT9V034() {
  pcb_width = 40.0;
  pcb_thickness = 1.58;
  pcb_hole = 2.1;
  mount_w = 34.0;
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

module intel_D430_depth_module() {
  body_w = 64.7;
  body_d = 14.0;
  body_h = 8.0;

  stereo_baseline = 50.0;
  lens_d = 10.0;

  conn_w = 15.0;
  conn_d = 6.0;
  conn_h = body_h - 2;
  conn_offset_x = body_d / 2 - conn_d / 2;
  conn_offset_y = -stereo_baseline / 2 + 35;
  conn_offset_z = conn_h / 2 + 2;

  lip_w = 3.0;
  lip_d = 6.0;
  lip_h = 1.0;

  lip_screw_d = 1.8;

  difference() {
    union() {
      // Body
      color([0.5, 0.5, 0.5])
      translate([0, 0, body_h / 2])
        cube([body_d, body_w, body_h], center=true);

      // Left IR camera
      color([0.2, 0.2, 0.2])
      translate([0, stereo_baseline / 2, body_h])
        cylinder(h=10.52 - body_h, r=lens_d / 2);

      // Right IR camera
      color([0.2, 0.2, 0.2])
      translate([0, -stereo_baseline / 2, body_h])
        cylinder(h=10.52 - body_h, r=lens_d / 2);

      // Left mount lip
      color([0.5, 0.5, 0.5])
      translate([0, body_w/ 2 + lip_w / 2, lip_h / 2])
        cube([lip_d, lip_w, lip_h], center=true);

      // Right mount lip
      color([0.5, 0.5, 0.5])
      translate([0, -body_w/ 2 - lip_w / 2, lip_h / 2])
        cube([lip_d, lip_w, lip_h], center=true);
    }

    // RGB camera
    translate([0, stereo_baseline / 2 - 29, body_h - 0.5])
      cylinder(h=10.52 - body_h, r=lens_d / 2);

    // Conector Hole
    translate([conn_offset_x, conn_offset_y, conn_offset_z])
      cube([conn_d + 0.1, conn_w + 0.1, conn_h + 0.1], center=true);

    // Left lip hole
    translate([0, body_w/ 2 + 1.8, lip_h / 2])
      cylinder(h=lip_h + 0.1, r=lip_screw_d / 2, center=true);

    // Right lip hole
    translate([0, -body_w/ 2 - lip_screw_d / 2, lip_h / 2])
      cylinder(h=lip_h + 0.1, r=lip_screw_d / 2, center=true);
    translate([0, -body_w / 2 - lip_w + 0.5, lip_h / 2])
      cube([lip_screw_d, lip_w, lip_h + 0.1], center=true);
  }
}

// OAK-D Lite Camera
module oak_d_lite() {
  body_width = 91.0;
  body_height = 28.0;
  body_depth = 17.5;
  mount_width = 75.0;
  mount_height = (body_height / 2.0) - 18.5;
  screw_width = 4.0;
  screw_depth = 6.5;
  cam_d = 8.0;

  difference() {
    // Body
    translate([0.0, 0.0, body_depth / 2])
      color([0.2, 0.2, 0.2])
        cube([body_height, body_width, body_depth], center=true);

    // Mount Holes
    translate([mount_height, mount_width / 2.0, screw_depth / 2.0 - 0.01])
      cylinder(h=screw_depth, r=screw_width / 2.0, center=true);
    translate([mount_height, -mount_width / 2.0, screw_depth / 2.0 - 0.01])
      cylinder(h=screw_depth, r=screw_width / 2.0, center=true);

    // RGB Camera
    translate([-5.0, 0.0, body_depth])
      cylinder(h=1, r=cam_d / 2.0, center=true);

    // SLAM Cameras
    translate([-5.0, body_width / 2.0 - 12, body_depth])
      cylinder(h=1, r=cam_d / 2.0, center=true);
    translate([-5.0, -(body_width / 2.0 - 12), body_depth])
      cylinder(h=1, r=cam_d / 2.0, center=true);
  }

}

// Gimbal Motor GM2804
module gimbal_motor_GM2804(has_encoders=0) {
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

// Gimbal Motor GM3506
module gimbal_motor_GM3506() {
  motor_r = 40.0 / 2.0;
  motor_h = 18.0;
  top_mount_d = 20.0;
  base_mount_d = 34.0;

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
        translate([top_mount_d / 2.0, 0.0, motor_h - 2.0 + 0.01])
          cylinder(r=1.0, h=2.0, center=false);
    }

    // Base mount holes
    for (i = [1 : 4]) {
      rotate([0.0, 0.0, i * 90.0 + 45.0])
        translate([base_mount_d / 2.0, 0.0, -0.01])
          cylinder(r=1.0, h=2.0, center=false);
    }

    // Wire hole
    // if (has_encoders) {
    //   translate([-motor_r + 2.5, 0.0, 2.0 - 0.01])
    //     cube([5.0, 9.0, 4.0], center=true);
    // }
  }
}

// Gimbal Motor GM4008
module gimbal_motor_GM4008() {
  motor_r = 46.0 / 2.0;
  motor_h = 21.0;
  top_mount_d = 30.0;
  base_mount_d = 30.0;

  color([0.2, 0.2, 0.2])
  difference() {
    union() {
      // Main body
      cylinder(r=motor_r, h=motor_h);
    }

    // Top mount holes
    for (i = [45:90:360]) {
      rotate([0.0, 0.0, i])
        translate([top_mount_d / 2.0, 0.0, motor_h - 2.0 + 0.01])
          cylinder(r=M3_SCREW_W / 2, h=2.0, center=false);
    }

    // Base mount holes
    for (i = [1 : 4]) {
      rotate([0.0, 0.0, i * 90.0 + 45.0])
        translate([base_mount_d / 2.0, 0.0, -0.01])
          cylinder(r=M3_SCREW_W / 2, h=2.0, center=false);
    }

    // Wire hole
    // if (has_encoders) {
    //   translate([-motor_r + 2.5, 0.0, 2.0 - 0.01])
    //     cube([5.0, 9.0, 4.0], center=true);
    // }
  }
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
      cylinder(r=M2_SCREW_W / 2, h=h + tol, center=true);
    translate([d / 2 - 7.5, -mount_w / 2, h / 2])
      cylinder(r=M2_SCREW_W / 2, h=h + tol, center=true);
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
            cylinder(r=M2_SCREW_W / 2, h=fcu_h + 0.1, center=true);
          translate([-fcu_mount_w / 2, fcu_mount_d / 2, fcu_h / 2])
            cylinder(r=M2_SCREW_W / 2, h=fcu_h + 0.1, center=true);
          translate([fcu_mount_w / 2, -fcu_mount_d / 2, fcu_h / 2])
            cylinder(r=M2_SCREW_W / 2, h=fcu_h + 0.1, center=true);
          translate([-fcu_mount_w / 2, -fcu_mount_d / 2, fcu_h / 2])
            cylinder(r=M2_SCREW_W / 2, h=fcu_h + 0.1, center=true);
        }
  }

  difference() {
    union() {
      // Mount point
      frame(30.5, 30.5,
            M3_SCREW_W, M3_NUT_W, M3_NUT_H,
            standoff_w, standoff_h -2, standoff_h -2);

      // FCU frame
      frame(fcu_mount_w, fcu_mount_d,
            M2_SCREW_W, M2_NUT_W, M2_NUT_H,
            standoff_w - 2.5, fcu_standoff_h, fcu_support_h, 0, 1);

      // Battery frame
      rotate(90)
      frame(batt_frame_d, batt_frame_w, M3_SCREW_W, M3_NUT_W, M3_NUT_H,
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
  //   cube([fcu_mount_w - M2_SCREW_W, standoff_w / 2.0, standoff_h], center=true);
  // translate([0.0, -fcu_mount_d / 2 - 3, standoff_h / 2.0])
  //   cube([fcu_mount_w - M2_SCREW_W, standoff_w / 2.0, standoff_h], center=true);
}

module encoder_frame_GM2804(show_encoder=1, show_motor=0, has_encoders=0) {
  mount_w = 18;
  mount_d = 11;
  motor_standoff_w = 5;
  motor_standoff_h = 10; // Roll and Yaw motor
  // motor_standoff_h = 7; // Pitch motor

  encoder_standoff_w = 7;
  encoder_standoff_h = 4; // Roll and Yaw motor
  support_w = 4;
  support_h = 3.5;
  motor_mount_d = sqrt(pow(29.0, 2) + pow(29.0, 2)) / 2;

  if (show_motor) {
    rotate(45)
      translate([0, 0, motor_standoff_h])
        gimbal_motor_GM2804();
  }

  difference() {
      union() {
      // Show encoder
      if (show_encoder) {
        translate([0, 0, encoder_standoff_h + 0.01]) encoder_board();
      }

      // Encoder frame
      frame(mount_w, mount_d,
            M2_SCREW_W, M2_NUT_W, M2_NUT_H,
            encoder_standoff_w, encoder_standoff_h,
            support_w, support_h);

      // Motor frame
      rotate(45)
        frame(motor_mount_d, motor_mount_d,
              M2_SCREW_W, M2_NUT_W, M2_NUT_H,
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
        cylinder(r=M2_SCREW_W / 2, h=encoder_standoff_h + 0.1, center=true);

        translate([0, 0, -encoder_standoff_h / 2 + M2_NUT_H / 2])
          cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, $fn=6, center=true);
      }
    }
  }
}

module encoder_frame_GM3506(show_encoder=1, show_motor=0, has_encoders=0) {
  mount_w = 18;
  mount_d = 11;
  motor_standoff_w = 5;
  motor_standoff_h = 11;

  encoder_standoff_w = 7;
  encoder_standoff_h = 4;
  support_w = 4;
  support_h = 3.5;
  motor_mount_d = sqrt(pow(34.0, 2) + pow(34.0, 2)) / 2;

  if (show_motor) {
    rotate(45)
      translate([0, 0, motor_standoff_h + 0.01])
        gimbal_motor_GM3506();
  }

  difference() {
      union() {
      // Show encoder
      if (show_encoder) {
        translate([0, 0, encoder_standoff_h + 0.01]) encoder_board();
      }

      // Encoder frame
      frame(mount_w, mount_d,
            M25_SCREW_W, M25_NUT_W, M25_NUT_H,
            encoder_standoff_w, encoder_standoff_h,
            support_w, support_h);

      // Motor frame
      rotate(45)
        frame(motor_mount_d, motor_mount_d,
              M25_SCREW_W, M25_NUT_W, M25_NUT_H,
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
        cylinder(r=M25_SCREW_W / 2, h=encoder_standoff_h + 0.1, center=true);

        translate([0, 0, -encoder_standoff_h / 2 + M25_NUT_H / 2])
          cylinder(r=M25_NUT_W / 2, h=M25_NUT_H + 0.1, $fn=6, center=true);
      }
    }
  }
}

module encoder_frame_GM4008(show_encoder=1, show_motor=0, has_encoders=0) {
  mount_w = 18;
  mount_d = 11;
  motor_standoff_w = 5;
  motor_standoff_h = 10; // Roll and Yaw motor

  encoder_standoff_w = 7;
  encoder_standoff_h = 4; // Roll and Yaw motor
  support_w = 4;
  support_h = 2;
  motor_mount_d = sqrt(pow(30.0, 2) + pow(30.0, 2)) / 2;

  if (show_motor) {
    rotate(45)
      translate([0, 0, motor_standoff_h + 0.01])
        gimbal_motor_GM4008();
  }

  difference() {
      union() {
      // Show encoder
      if (show_encoder) {
        translate([0, 0, encoder_standoff_h + 0.01]) encoder_board();
      }

      // Encoder frame
      frame(mount_w, mount_d,
            M2_SCREW_W, M2_NUT_W, M2_NUT_H,
            encoder_standoff_w, encoder_standoff_h,
            support_w, support_h);

      // Motor frame
      rotate(45)
        frame(motor_mount_d, motor_mount_d,
              M3_SCREW_W, M3_NUT_W, M3_NUT_H,
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
        cylinder(r=M2_SCREW_W / 2, h=encoder_standoff_h + 0.1, center=true);

        translate([0, 0, -encoder_standoff_h / 2 + M2_NUT_H / 2])
          cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, $fn=6, center=true);
      }
    }
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
            M3_SCREW_W, M3_NUT_W, M3_NUT_H,
            standoff_w, standoff_h, standoff_w, standoff_h, 0, 0);

      // Odroid frame
      frame(odroid_mount_w, odroid_mount_d,
            M3_SCREW_W, M3_NUT_W, M3_NUT_H,
            standoff_w, standoff_h, standoff_w, standoff_h, 0, 1);

      // Mount supports
      for (pos = odroid_mount_positions)
        translate(pos)
          cylinder(r=(M3_SCREW_W + 2.0) / 2, h=h, center=true);
    }

    // Mount holes
    for (pos = mount_positions) {
      translate(pos)
        cylinder(r=M3_SCREW_W / 2, h=20, center=true);
    }

    // Odroid mount holes
    for (pos = odroid_mount_positions) {
      translate(pos)
        cylinder(r=M3_SCREW_W / 2, h=20, center=true);
    }
  }
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
      cylinder(r=M2_SCREW_W / 2, h=board_h + 0.01, center=true);
    translate([-board_w / 2 + 2.5, mount_w / 2, board_h / 2])
      cylinder(r=M2_SCREW_W / 2, h=board_h + 0.01, center=true);
    translate([-board_w / 2 + 2.5, -mount_w / 2, board_h / 2])
      cylinder(r=M2_SCREW_W / 2, h=board_h + 0.01, center=true);
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
          cylinder(r=M3_SCREW_W / 2, h=sbgc_h + 0.01, center=true);
    }
  }
}

module sbgc_frame(show_sbgc=1, show_usb_hub=1) {
  motor_base_mount_d = sqrt(17 * 17 + 17 * 17);  // GM3506
  mount_standoff_w = 8.0;
  mount_standoff_h = 4.0;
  mount_support_w = 4.0;
  mount_support_h = 4.0;

  sbgc_offset_x = 0.0;
  sbgc_standoff_w = 7.0;
  sbgc_standoff_h = 10.0;
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
      // rotate(45)
      //   frame(motor_base_mount_d, motor_base_mount_d,
      //         M3_SCREW_W, M3_NUT_W, M3_NUT_H,
      //         mount_standoff_w, mount_standoff_h,
      //         mount_support_w, mount_support_h);

      // USB Hub board
      // if (show_usb_hub) {
      //   translate([0, hub_offset_y, hub_standoff_h])
      //     usb_hub_board();
      // }

      // // USB Hub board frame
      // // -- Standoffs
      // translate([hub_w / 2 - 1, hub_offset_y, hub_standoff_h / 2])
      //   cylinder(r=M2_SCREW_W / 2 + 2, h=hub_standoff_h, center=true);
      // translate([-hub_w / 2 + 2.5, hub_offset_y - hub_d / 2 + 0.5, hub_standoff_h / 2])
      //   cylinder(r=M2_SCREW_W / 2 + 2, h=hub_standoff_h, center=true);
      // translate([-hub_w / 2 + 2.5, hub_offset_y + hub_d / 2 - 0.5, hub_standoff_h / 2])
      //   cylinder(r=M2_SCREW_W / 2 + 2, h=hub_standoff_h, center=true);
      // // -- Supports
      // translate([hub_w / 2 - 1, hub_offset_y + hub_support_w / 2, mount_support_h / 2])
      //   cube([mount_support_w, hub_support_w, mount_support_w], center=true);
      // translate([-hub_w / 2 + 2.5, hub_offset_y, mount_support_h / 2])
      //   cube([mount_support_w, hub_mount_w, mount_support_w], center=true);
      // translate([-hub_w / 2 + 2.5, hub_offset_y + hub_support_w / 2, mount_support_h / 2])
      //   cube([mount_support_w, hub_support_w, mount_support_w], center=true);
      // translate([0, hub_offset_y, mount_support_h / 2])
      //   cube([24, mount_support_w, mount_support_w], center=true);

      // Simple BGC
      if (show_sbgc) {
        translate([sbgc_offset_x, 0, sbgc_standoff_h])
          sbgc_board();
      }

      // SBGC frame
      translate([sbgc_offset_x, 0, 0])
        rotate(0)
          frame(sbgc_mount_w, sbgc_mount_d,
                M2_SCREW_W, M2_NUT_W, M2_NUT_H,
                sbgc_standoff_w, sbgc_standoff_h,
                sbgc_support_w, sbgc_support_h,
                nut_csb=1);

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

    // // USB Hub mount holes
    // // -- Screw mount holes
    // translate([hub_w / 2 - 1, hub_offset_y, hub_standoff_h / 2])
    //   cylinder(r=M2_SCREW_W / 2, h=hub_standoff_h + 0.01, center=true);
    // translate([-hub_w / 2 + 2.5, hub_offset_y - hub_d / 2 + 0.5, hub_standoff_h / 2])
    //   cylinder(r=M2_SCREW_W / 2, h=hub_standoff_h + 0.01, center=true);
    // translate([-hub_w / 2 + 2.5, hub_offset_y + hub_d / 2 -0.5, hub_standoff_h / 2])
    //   cylinder(r=M2_SCREW_W / 2, h=hub_standoff_h + 0.01, center=true);
    // // -- Nut counter sinks
    // translate([hub_w / 2 - 1, hub_offset_y, M2_NUT_H / 2])
    //   cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.01, $fn=6, center=true);
    // translate([-hub_w / 2 + 2.5, hub_offset_y - hub_d / 2 + 0.5, M2_NUT_H / 2])
    //   cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.01, $fn=6, center=true);
    // translate([-hub_w / 2 + 2.5, hub_offset_y + hub_d / 2 - 0.5, M2_NUT_H / 2])
    //   cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.01, $fn=6, center=true);

    // SBGC Mount holes
    // for (i = [1:4]) {
    //   translate([sbgc_offset_x, 0.0, 0.0]) {
    //     rotate(90 * i) {
    //       translate([sbgc_mount_w / 2, sbgc_mount_d / 2, sbgc_standoff_h / 2])
    //         cylinder(r=M2_SCREW_W / 2, h=sbgc_standoff_h + 0.01, center=true);

    //       translate([sbgc_mount_w / 2, sbgc_mount_d / 2, M2_NUT_H / 2])
    //         cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.01, $fn=6, center=true);
    //     }
    //   }
    // }

    // Motor Mount holes
    for (i = [1:4]) {
      rotate(90 * i + 45.0) {
        translate([motor_base_mount_d / 2, motor_base_mount_d / 2, sbgc_support_h / 2])
          cylinder(r=M3_SCREW_W / 2, h=sbgc_support_h + 0.01, center=true);
      }
    }
  }
}

module oak_d_lite_frame(show_camera=0) {
  support_w = 90;
  support_d = 8;
  support_h = 4;
  standoff_w = 6;
  standoff_h = 6;

  oak_mount_w = 75.0;
  mount_w = 12.0;

  // Show camera
  if (show_camera) {
    translate([4, 0, standoff_h])
      oak_d_lite();
  }

  difference() {
    union() {
      // Frame
      translate([0, 0, support_h / 2])
        cube([support_d, oak_mount_w, support_h], center=true);
      translate([0, oak_mount_w / 2, support_h / 2])
        cylinder(r=support_d / 2, h=support_h, center=true);
      translate([0, -oak_mount_w / 2, support_h / 2])
        cylinder(r=support_d / 2, h=support_h, center=true);

      // Standoff
      translate([0, oak_mount_w / 2, standoff_h / 2])
        cylinder(r=standoff_w / 2, h=standoff_h, center=true);
      translate([0, -oak_mount_w / 2, standoff_h / 2])
        cylinder(r=standoff_w / 2, h=standoff_h, center=true);
    }

    // Standoff Hole
    translate([0, oak_mount_w / 2, standoff_h / 2])
      cylinder(r=2, h=standoff_h + 1, center=true);
    translate([0, -oak_mount_w / 2, standoff_h / 2])
      cylinder(r=2, h=standoff_h + 1, center=true);

    // Mount Hole
    translate([0, 0, support_h / 2])
      cube([M2_SCREW_W + 0.1, oak_mount_w * 0.7, 10], center=true);
  }
}

module gimbal_frame(mount_w, mount_d, show_yaw_frame=1) {
  motor_h = 15.0;
  motor_mount_d = sqrt(15 * 15 + 15 * 15);

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
      encoder_frame_GM2804();

    translate([0, 0, support_h + encoder_standoff_h +  0.01])
      rotate(45)
        gimbal_motor_GM2804();

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
            M3_SCREW_W, M3_NUT_W, M3_NUT_H,
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
          cylinder(r=M2_SCREW_W / 2, h=support_h + 0.01, center=true);
    }

    // // SBGC mount holes
    // sbgc_holes = [
    //   [sbgc_mount_w / 2,  35.0 + sbgc_mount_d / 2, 0],
    //   [-sbgc_mount_w / 2, 35.0 + -sbgc_mount_d / 2, 0]
    // ];
    // for (pos = sbgc_holes) {
    //   translate(pos) {
    //     translate([0, 0, sbgc_standoff_h / 2])
    //       cylinder(r=M2_SCREW_W / 2, h=sbgc_standoff_h + 0.01, center=true);
    //     translate([0, 0, support_h / 2])
    //       cylinder(r=M2_NUT_W / 2, h=support_h + 0.01, $fn=6, center=true);
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
    //         cylinder(r=M2_SCREW_W / 2, h=psu_standoff_h + 0.01, center=true);
    //       translate([0, 0, support_h / 2])
    //         cylinder(r=M2_NUT_W / 2, h=support_h + 0.01, $fn=6, center=true);
    //     }
    //   }
    // }
  }
}

module gimbal_pitch_frame(show_pitch_motor=0, show_camera=0) {
  tol = 0.6;

  // GM2804
  // motor_r = 20.5;
  // motor_h = 15.0;
  // motor_mount_w = 19;
  // motor_mount_h = 5;

  // GM3506
  motor_r = 40.0 / 2.0;
  motor_h = 18.0;
  motor_mount_w = 20.0;
  motor_mount_h = 5;

  // GM4008
  // motor_r = 46.0 / 2;
  // motor_h = 21.0;
  // motor_mount_w = 30.0;
  // motor_mount_h = 5;

  support_x = motor_r + 5;
  support_w = 8;
  support_d = 6;
  support_h = 35;

  hole_offset = 6;
  hole_w = 12;

  // Show pitch motor
  if (show_pitch_motor) {
    rotate([180.0, 0.0, 0.0])
      translate([0, 0, -motor_h - motor_mount_h - 0.01])
        // gimbal_motor_GM2804();
        gimbal_motor_GM3506();
        // gimbal_motor_GM4008();
  }

  // Show camera
  if (show_camera) {
    rotate([0, 90, 0])
      rotate([0, 0, -90])
        translate([0, -support_h / 2 - hole_offset, support_x + support_d / 2])
          oak_d_lite_frame(show_camera=1);
  }

  // Pitch frame
  difference() {
    union() {
      // Pitch motor mount
      translate([0, 0, motor_mount_h / 2])
        cylinder(r=motor_r, h=motor_mount_h, center=true);

      // Support
      translate([support_x / 2, 0, motor_mount_h / 2])
        cube([support_x, support_w, motor_mount_h], center=true);
      translate([support_x, 0, support_h / 2])
        cube([support_d, support_w, support_h], center=true);
    }

    // Pitch motor holes
    for (i = [45:90:360]) {
      rotate([0.0, 0.0, i])
        translate([motor_mount_w / 2.0, 0.0, -0.1])
          cylinder(r=M25_SCREW_W / 2, h=motor_mount_h + 0.2, center=false);
    }

    // Mount Holes
    translate([support_x, 0, support_h / 2 + hole_offset]) {
      translate([0, 0, hole_w / 2]) {
        rotate([0.0, 90.0, 0.0])
          cylinder(r=M25_SCREW_W / 2, h=support_d + 0.1, center=true);
      }
      translate([0, 0, -hole_w / 2]) {
        rotate([0.0, 90.0, 0.0])
          cylinder(r=M25_SCREW_W / 2, h=support_d + 0.1, center=true);
      }
    }

    // Hex Holes
    hex_x = support_x - support_d / 2 + M25_NUT_H / 2 - 0.01;
    hex_y = 0.0;
    hex_z = support_h / 2 + hole_offset;
    translate([hex_x, hex_y, hex_z]) {
      translate([0, 0, hole_w / 2])
        rotate([0.0, 90.0, 0.0])
          cylinder(r=M25_NUT_W / 2, h=M25_NUT_H, center=true, $fn=6);
      translate([0, 0, -hole_w / 2])
        rotate([0.0, 90.0, 0.0])
          cylinder(r=M25_NUT_W / 2, h=M25_NUT_H, center=true, $fn=6);
    }
  }
}

module gimbal_roll_frame(show_roll_motor=1,
                          show_pitch_motor=1,
                          show_pitch_frame=1,
                          show_encoder=1) {

  // GM2804
  // motor_r = 20.5;
  // motor_h = 15.0;
  // motor_mount_w = 13.2;
  // motor_mount_d = 20.5;
  // motor_base_mount_w = 29.0;
  // motor_offset_y = 0;
  // motor_offset_z = 40;

  // GM3506
  motor_r = 40 / 2.0;
  motor_h = 18.0;
  motor_mount_w = 20.0;
  motor_mount_d = 20.0;
  motor_base_mount_w = 34.0;
  motor_offset_y = -2;
  motor_offset_z = 40;

  // GM4008
  // motor_r = 46.0 / 2;
  // motor_h = 21.0;
  // motor_mount_w = 30.0;
  // motor_mount_d = 30.0;
  // motor_base_mount_w = 30.0;
  // motor_offset_y = 2;
  // motor_offset_z = 35;

  // Roll mount
  roll_mount_w = 30.0;
  roll_mount_d = 30.0;
  roll_mount_h = 6.0;

  // Pitch mount
  pitch_mount_w = roll_mount_w;
  pitch_mount_d = roll_mount_h + 1;
  pitch_mount_offset_y = (35 + pitch_mount_d) / 2;

  // Show roll motor
  if (show_roll_motor) {
    // translate([0, 0, -motor_h]) gimbal_motor_GM2804();
    translate([0, 0, -motor_h - 0.1]) gimbal_motor_GM3506();
    // translate([0, 0, -motor_h]) gimbal_motor_GM4008();
  }

  // Show encoder
  if (show_pitch_motor) {
    // GM2804
    // translate([0, pitch_mount_offset_y + motor_offset_y - roll_mount_h / 2, motor_offset_z])
    //   rotate([90, 0, 0])
    //     encoder_frame_GM2804(show_motor=1);

    // GM3506
    translate([0, pitch_mount_offset_y - pitch_mount_d / 2 + motor_offset_y, motor_offset_z])
      rotate([90, 0, 0])
        encoder_frame_GM3506(show_motor=1);

    // GM4008
    // translate([0, pitch_mount_offset_y + motor_offset_y - roll_mount_h / 2, motor_offset_z])
    //   rotate([90, 0, 0])
    //     encoder_frame_GM4008(show_motor=1);
  }

  // Show pitch frame
  if (show_pitch_frame) {
    // y = motor_offset_y - 16;
    y = motor_offset_y - 15.5;
    // y = motor_offset_y - 12.5;
    z = motor_offset_z;
    translate([0, y, z])
      rotate([-90, -90, 0])
        gimbal_pitch_frame(show_camera=1);
  }

  // Roll frame
  difference() {
    union() {
      // Roll motor mount
      translate([0, 0, roll_mount_h / 2])
        cylinder(r=motor_r, h=roll_mount_h, center=true);

      // Roll-Pitch mount connector
      // translate([0, pitch_mount_offset_y / 2, roll_mount_h / 2])
      //   cube([roll_mount_w, pitch_mount_offset_y / 2, roll_mount_h], center=true);

      // Pitch motor mount
      translate([0, pitch_mount_offset_y + motor_offset_y, motor_offset_z])
        rotate([90, 0, 0])
          cylinder(r=motor_r, h=pitch_mount_d, center=true);

      // Pitch mount plate support
      translate([0, pitch_mount_offset_y + motor_offset_y, motor_offset_z / 2])
        cube([roll_mount_w, pitch_mount_d, motor_offset_z], center=true);
    }

    // Roll mount holes
    for (i = [45:90:360]) {
      rotate([0.0, 0.0, i])
        translate([motor_mount_w / 2.0, 0.0, -0.1])
          cylinder(r=M25_SCREW_W / 2, h=roll_mount_h + 0.2, center=false);
    }

    // Pitch motor center hole
    translate([0, pitch_mount_offset_y + motor_offset_y, motor_offset_z])
      rotate([90, 0, 0])
        cylinder(r=10, h=pitch_mount_d + 0.01, center=true);

    // Pitch motor holes
    pitch_holes_x = 0;
    pitch_holes_y = pitch_mount_offset_y + motor_offset_y + roll_mount_h / 2;
    pitch_holes_z = motor_offset_z;
    translate([0, pitch_holes_y, pitch_holes_z]) {
      rotate([90, 0, 0]) {
        for (i = [45:90:360]) {
          rotate([0.0, 0.0, i]) {
            translate([motor_base_mount_w / 2.0, 0.0, -0.1]) {
              cylinder(r=M25_SCREW_W / 2, h=pitch_mount_d + 0.2, center=false);
            }
          }
        }
      }
    }

    // Wire management holes
    translate([5, pitch_mount_offset_y + motor_offset_y, 12])
      rotate([90, 0, 0])
        cylinder(r=5, h=pitch_mount_d + 0.01, center=true);
    translate([-5, pitch_mount_offset_y + motor_offset_y, 12])
      rotate([90, 0, 0])
        cylinder(r=5, h=pitch_mount_d + 0.01, center=true);
    translate([0, pitch_mount_offset_y + motor_offset_y, 12])
      cube([10, pitch_mount_d + 0.01, 10], center=true);
  }
}

module gimbal_yaw_frame(show_roll_frame=1, show_yaw_motor=1, show_sbgc_frame=1) {
  // GM3506
  roll_motor_r = 40.0 / 2.0;
  roll_motor_h = 18.0;
  roll_top_mount_d = 20.0;
  roll_base_mount_d = 34.0;
  roll_encoder_h = 10;

  // GM4008
  yaw_motor_r = 46.0 / 2.0;
  yaw_motor_h = 21.0;
  yaw_top_mount_d = 30.0;
  yaw_base_mount_d = 30.0;
  yaw_encoder_h = 8;

  // Support
  frame_thickness = 8;
  support_w = 30.0;
  support_h = 8.0;

  // Roll mount offset
  roll_mount_x = -50;
  roll_mount_z = 40;

  // Show roll frame
  if (show_roll_frame) {
    // Roll motor
    encoder_offset_x = roll_mount_x + frame_thickness / 2;
    encoder_offset_z = roll_mount_z;
    translate([encoder_offset_x, 0.0, encoder_offset_z])
      rotate([0, 90, 0])
        rotate([0, 0, 0])
          encoder_frame_GM3506(show_encoder=1, show_motor=1);
          // encoder_frame_GM4008(show_encoder=1, show_motor=1);

    // Roll frame
    offset_x = encoder_offset_x + roll_motor_h + roll_encoder_h;
    offset_z = roll_mount_z;
    translate([offset_x, 0.0, offset_z])
      rotate([0, 90, 0])
        rotate([0, 0, 180])
          gimbal_roll_frame(show_roll_motor=0);
  }

  // Show yaw motor
  if (show_yaw_motor) {
    translate([0, 0, -yaw_motor_h - yaw_encoder_h])
      rotate([0, 0, -90])
        // encoder_frame_GM3506(show_encoder=1, show_motor=1);
        encoder_frame_GM4008(show_encoder=1, show_motor=1);
  }

  // Show SBGC frame
  if (show_sbgc_frame) {
    translate([roll_mount_x - frame_thickness / 2, 0, roll_mount_z]) {
      rotate([0, -90, 0]) {
        sbgc_frame(show_sbgc=1);
      }
    }
  }

  // Yaw frame
  difference() {
    union() {
      // Yaw motor mount
      translate([0, 0, frame_thickness / 2])
        cylinder(r=yaw_motor_r, h=frame_thickness, center=true);

      // Roll motor mount
      translate([roll_mount_x, 0.0, roll_mount_z])
        rotate([0, 90, 0])
          cylinder(r=roll_motor_r, h=frame_thickness, center=true);

      // SBGC frame
      translate([roll_mount_x, 0.0, roll_mount_z])
        rotate([0, -90, 0])
          sbgc_frame(show_sbgc=0, show_usb_hub=0);

      // Encoder frame
      // translate([roll_mount_x + frame_thickness / 2, 0.0, roll_mount_z])
      //   rotate([0, 90, 0])
      //     encoder_frame_GM3506(show_motor=1);

      // Horizontal support
      translate([roll_mount_x / 2, 0, frame_thickness / 2])
        cube([-roll_mount_x, support_w, frame_thickness], center=true);

      // Vertical support
      translate([roll_mount_x, 0, roll_mount_z / 2])
        cube([frame_thickness, support_w, roll_mount_z], center=true);

      // Diagonal supports
      translate([roll_mount_x / 2 - 15, support_w / 2 - frame_thickness / 2, roll_mount_z / 2 - 8])
        rotate([0, 45, 0])
          cube([25, frame_thickness, frame_thickness], center=true);
      translate([roll_mount_x / 2 - 15, -support_w / 2 + frame_thickness / 2, roll_mount_z / 2 - 8])
        rotate([0, 45, 0])
          cube([25, frame_thickness, frame_thickness], center=true);
    }

    // Yaw motor mount holes
    motor_mount_holes([0, 0, frame_thickness], yaw_top_mount_d, M3_SCREW_W, frame_thickness);

    // Yaw motor mount cutout
    translate([0, 0, frame_thickness / 2])
      cylinder(r=roll_motor_r * 0.5, h=frame_thickness + 0.1, center=true);

    // Roll motor mount holes
    rotate([0, 90, 0])
        motor_mount_holes([-roll_mount_z, 0, roll_mount_x + frame_thickness / 2],
                          roll_base_mount_d,
                          M25_SCREW_W,
                          frame_thickness,
                          offset_rot_z=45);

    // Roll motor mount cutout
    translate([roll_mount_x, 0.0, roll_mount_z])
      rotate([0, 90, 0])
        cylinder(r=roll_motor_r * 0.5, h=frame_thickness + 0.1, center=true);

    // Support cutout
    support_cutout_d = 35;
    support_cutout_h = 18;
    support_cutout_w = support_w * 0.5;
    support_cutout_offset_x = roll_mount_x + support_cutout_d / 2 - frame_thickness / 2;
    translate([support_cutout_offset_x, 0, support_cutout_h / 2])
      cube([support_cutout_d + 0.1, support_cutout_w + 0.1, support_cutout_h + 0.1], center=true);
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
            M3_SCREW_W, M3_NUT_W, M3_NUT_H,
            standoff_w, support_h,
            support_w, support_h);

      // PCB frame
      frame(pcb_mount_w, pcb_mount_d,
            M2_SCREW_W, M2_NUT_W, M2_NUT_H,
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
    mount_holes(pcb_mount_w, pcb_mount_d, M2_SCREW_W, support_h);
    mount_holes(pcb_mount_w, pcb_mount_d, M2_NUT_W, M2_NUT_H, fn=6);
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
          import("../../proto_parts/Intel_NUC7i5DN/NUC7i5DN.STL");
  }

  // NUC frame
  frame(mount_w, mount_d,
        M3_SCREW_W, M3_NUT_W, M3_NUT_H,
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

/////////////////////////////////////////////////////////////////////////////
// ASSEMBLY                                                                //
/////////////////////////////////////////////////////////////////////////////

// module gimbal_assembly() {
//   // Gimbal frame
//   gimbal_frame(nuc_mount_w, nuc_mount_d, 0);

//   // GM4008
//   yaw_motor_r = 46.0 / 2;
//   yaw_motor_h = 21.0;
//   yaw_motor_mount_w = 30.0;
//   yaw_motor_mount_h = 5;
//   yaw_encoder_h = 8;

//   // Gimbal
//   offset_z = yaw_motor_h + yaw_encoder_h + standoff_h;
//   translate([0, 0, offset_z])
//     gimbal_yaw_frame(show_roll_frame=1, show_yaw_motor=1, show_sbgc_frame=1);
// }

// translate([0, 0, 32])
//   gimbal_assembly();
// nuc_frame(nuc_mount_w, nuc_mount_d, show_nuc=1);

/////////////////////////////////////////////////////////////////////////////
// COMPONENT DEVELOPMENT                                                   //
/////////////////////////////////////////////////////////////////////////////

// battery_frame(batt_frame_w, batt_frame_d);
// fcu_frame(1);
// encoder_frame_GM2804(show_encoder=0, show_motor=0);
// encoder_frame_GM3506(show_encoder=0, show_motor=0);
// encoder_frame_GM4008(show_encoder=0, show_motor=0);
// odroid_frame(nuc_mount_w, nuc_mount_d, 0);

// -- COMPONENTS
// board_camera();
// arducam_MT9V034();
intel_D430_depth_module();
// oak_d_lite();
// usb_hub_board();
// sbgc_board();
// gimbal_motor_GM2804();
// gimbal_motor_GM3506();
// gimbal_motor_GM4008();

// -- GIMBAL FRAMES
// gimbal_imu();
// gimbal_frame(nuc_mount_w, nuc_mount_d, 0);
// gimbal_pitch_frame(show_pitch_motor=0, show_camera=0);
// gimbal_roll_frame(show_roll_motor=0, show_pitch_motor=0, show_pitch_frame=0);
// gimbal_yaw_frame(show_roll_frame=0, show_yaw_motor=0, show_sbgc_frame=0);

// -- FRAMES
// oak_d_lite_frame(show_camera=1);
// sbgc_frame(show_sbgc=0, show_usb_hub=0);
// pcb_frame(nuc_mount_w, nuc_mount_d);
// nuc_frame(nuc_mount_w, nuc_mount_d, show_nuc=1);
// payload_frame(mav_payload_mount_w, mav_payload_mount_d,
//               mav_frame_support_w, mav_frame_support_h,
//               mav_frame_support_w, mav_frame_support_h,
//               nuc_mount_w, nuc_mount_d);

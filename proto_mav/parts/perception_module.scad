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

module lipo_battery(c=[0.4, 0.4, 0.4]) {
  color(c)
    cube([batt_w, batt_d, batt_h], center=true);
}

module encoder_AS5048() {
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

module encoder_AS5600() {
  pcb_w = 24.0;
  pcb_d = 22.5;
  pcb_h = 2.0;

  chip_w = 5;
  chip_d = 4;
  chip_h = 2;

  mount_hole_d = 4;
  mount_w = 16;
  mount_d = 16;

  difference() {
    union() {
      // PCB
      color([0.0, 1.0, 0.0])
      translate([0, 0, pcb_h / 2]) {
        cube([pcb_d, pcb_w, pcb_h], center=true);
      }

      // Chip
      color([0.0, 0.0, 0.0])
      translate([0, 0, pcb_h + chip_h / 2]) {
        cube([chip_d, chip_w, chip_h], center=true);
      }
    }

    // Mount Holes
    translate([mount_d / 2, mount_w / 2, pcb_h / 2])
      cylinder(r=mount_hole_d / 2, h=pcb_h + 0.1, center=true);
    translate([-mount_d / 2, mount_w / 2, pcb_h / 2])
      cylinder(r=mount_hole_d / 2, h=pcb_h + 0.1, center=true);
    translate([mount_d / 2, -mount_w / 2, pcb_h / 2])
      cylinder(r=mount_hole_d / 2, h=pcb_h + 0.1, center=true);
    translate([-mount_d / 2, -mount_w / 2, pcb_h / 2])
      cylinder(r=mount_hole_d / 2, h=pcb_h + 0.1, center=true);
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

// Intel D430 Depth Module
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

// Intel D4 Vision Processor
module intel_D4_vision_processor() {
  pcb_w = 72.0;
  pcb_d = 16.0;
  pcb_h = 1;

  usbc_w = 9.0;
  usbc_d = 8.0;
  usbc_h = 3.5;

  screw_w = M2_SCREW_W;

  difference() {
    union() {
      // PCB
      color([0, 1, 0])
      translate([0, 0, pcb_h / 2])
        cube([pcb_d, pcb_w, pcb_h], center=true);

      // USB-C port
      color([0.6, 0.6, 0.6])
      translate([0, -pcb_w / 2 + usbc_w / 2, pcb_h + usbc_h / 2])
        rotate([0, 0, 90])
          cube([usbc_d, usbc_w, usbc_h], center=true);
    }

    // Hole 1
    hole1_offset_x = -pcb_d / 2 + 0.6;
    hole1_offset_y = pcb_w / 2 - 67.5;
    translate([hole1_offset_x, hole1_offset_y, pcb_h / 2])
      cylinder(r=screw_w / 2, h=pcb_h + 0.1, center=true);
    translate([hole1_offset_x - 1.5, hole1_offset_y, pcb_h / 2])
      cube([screw_w, screw_w, pcb_h + 0.1], center=true);

    // Hole 2
    hole2_offset_x = -pcb_d / 2 + 1.5;
    hole2_offset_y = pcb_w / 2 - 4.5;
    translate([hole2_offset_x, hole2_offset_y, pcb_h / 2])
      cylinder(r=screw_w / 2, h=pcb_h + 0.1, center=true);
    translate([hole2_offset_x - screw_w / 2, hole2_offset_y, pcb_h / 2])
      cube([screw_w, screw_w, pcb_h + 0.1], center=true);

    // Hole 3
    hole3_offset_x = pcb_d / 2 - 1;
    hole3_offset_y = pcb_w / 2 - 62;
    translate([hole3_offset_x, hole3_offset_y, pcb_h / 2])
      cylinder(r=screw_w / 2, h=pcb_h + 0.1, center=true);
    translate([hole3_offset_x + 1.5, hole3_offset_y, pcb_h / 2])
      cube([screw_w, screw_w, pcb_h + 0.1], center=true);

    // Hole 4
    hole4_offset_x = pcb_d / 2 - 1.7;
    hole4_offset_y = pcb_w / 2 - 10;
    translate([hole4_offset_x, hole4_offset_y, pcb_h / 2])
      cylinder(r=screw_w / 2, h=pcb_h + 0.1, center=true);
    translate([hole4_offset_x + 1.5, hole4_offset_y, pcb_h / 2])
      cube([screw_w, screw_w, pcb_h + 0.1], center=true);
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

module encoder_magnet_holder_GM3506() {
  // magnet_r = 6.2 / 2;
  // magnet_h = 3;

  magnet_r = 4 / 2 + 0.1;
  magnet_h = 2;

  tol = 0.3;
  holder_bottom_r = 8 / 2 + tol;
  holder_bottom_h = 8;
  holder_top_r = 9.0 / 2 + tol;
  holder_top_h = 2 + 4;

  // color([0.5, 0.5, 0.5])
  // translate([0, 0, holder_bottom_h + holder_top_h])
  //   cylinder(r=magnet_r, h=magnet_h, center=true);

  difference() {
    union() {
      // Magnet holder bottom
      translate([0, 0, holder_bottom_h / 2])
        cylinder(r=holder_bottom_r, h=holder_bottom_h, center=true);

      // Magnet holder top
      translate([0, 0, holder_bottom_h + holder_top_h / 2])
        cylinder(r=holder_top_r, h=holder_top_h, center=true);
    }

    // Magnet hole
    translate([0, 0, holder_bottom_h + holder_top_h - magnet_h / 2])
      cylinder(r=magnet_r, h=magnet_h + 0.1, center=true);
  }
}

module encoder_magnet_holder_GM4008() {
  // magnet_r = 6.2 / 2;
  // magnet_h = 3;

  magnet_r = 4 / 2 + 0.1;
  magnet_h = 2;

  tol = 0.1;
  holder_bottom_r = 8.5 / 2 + tol;
  holder_bottom_h = 8;
  holder_top_r = 9.5 / 2 + tol;
  holder_top_h = 4;

  // color([0.5, 0.5, 0.5])
  // translate([0, 0, holder_bottom_h + holder_top_h])
  //   cylinder(r=magnet_r, h=magnet_h, center=true);

  difference() {
    union() {
      // Magnet holder bottom
      translate([0, 0, holder_bottom_h / 2])
        cylinder(r=holder_bottom_r, h=holder_bottom_h, center=true);

      // Magnet holder top
      translate([0, 0, holder_bottom_h + holder_top_h / 2])
        cylinder(r=holder_top_r, h=holder_top_h, center=true);
    }

    // Magnet hole
    translate([0, 0, holder_bottom_h + holder_top_h - magnet_h / 2])
      cylinder(r=magnet_r, h=magnet_h + 0.1, center=true);
  }
}

module S500_arm() {
  color([0.3, 0.3, 0.3])
  rotate([90, 0, 0])
    translate([3.5, 0, 0])
      import("../../proto_parts/S500/S500_Arm.STL");
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
        translate([0, 0, encoder_standoff_h + 0.01]) encoder_AS5048();
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

module encoder_frame_GM3506(show_encoder=1, show_motor=0, has_encoders=0, roll_limit=0, pitch_limit=0) {

  // AS5048
  encoder_mount_w = 18;
  encoder_mount_d = 11;
  motor_standoff_w = 5;
  motor_standoff_h = 11;
  encoder_standoff_w = 7;
  encoder_standoff_h = 4;

  // AS5600
  // encoder_mount_w = 16;
  // encoder_mount_d = 16;
  // motor_standoff_w = 5;
  // motor_standoff_h = 13;
  // encoder_standoff_w = 7;
  // encoder_standoff_h = 6;

  support_w = 4;
  support_h = 3.5;
  motor_mount_d = sqrt(pow(34.0, 2) + pow(34.0, 2)) / 2;

  // Pitch limiter
  pitch_limit_w = 5;
  pitch_limit_d = 7;
  // pitch_limit_offset = 135; // Pitch
  pitch_limit_offset = 135 - 90; // Pitch

  // Roll limiter
  roll_limit_w = 6;
  roll_limit_d = 10;
  roll_limit_h = 28;
  roll_limit_offset = 135; // Pitch

  // Show motor
  if (show_motor) {
    rotate(45)
      translate([0, 0, motor_standoff_h + 0.01])
        gimbal_motor_GM3506();
  }

  // Show encoder
  if (show_encoder) {
    // translate([0, 0, encoder_standoff_h + 0.01]) encoder_AS5048();
    translate([0, 0, encoder_standoff_h + 0.01]) encoder_AS5600();
  }

  difference() {
      union() {

      // Encoder frame
      frame(encoder_mount_w, encoder_mount_d,
            M2_SCREW_W, M2_NUT_W, M2_NUT_H,
            encoder_standoff_w, encoder_standoff_h,
            support_w, support_h);

      // Motor frame
      rotate(45)
        frame(motor_mount_d, motor_mount_d,
              M25_SCREW_W, M25_NUT_W, M25_NUT_H,
              motor_standoff_w, motor_standoff_h,
              support_w, support_h,
              disable=[2]);

      // Roll limiter
      if (roll_limit) {
        translate([-motor_mount_d / 2, motor_mount_d / 2, roll_limit_h / 2])
          rotate(-55)
            translate([roll_limit_d / 2 - 4, -roll_limit_w / 2, 0])
              cube([roll_limit_d, roll_limit_w, roll_limit_h], center=true);

        translate([motor_mount_d / 2, motor_mount_d / 2, roll_limit_h / 2])
          rotate(55)
            translate([roll_limit_d / 2 - 6, -roll_limit_w / 2, 0])
              cube([roll_limit_d, roll_limit_w, roll_limit_h], center=true);


        // rotate(35)
        //   translate([18, 0, roll_limit_h / 2])
        //     cube([roll_limit_d, roll_limit_w, roll_limit_h], center=true);
        // rotate(-35)
        //   translate([-18, 0, roll_limit_h / 2])
        //     cube([roll_limit_d, roll_limit_w, roll_limit_h], center=true);
      }

      // Pitch limiter
      if (pitch_limit) {
        rotate(pitch_limit_offset) {
          translate([motor_mount_d / 2, -motor_mount_d / 2, support_h / 2]) {
            rotate(45)
            translate([-pitch_limit_w / 2 - 4, -pitch_limit_d / 2, 0])
              cube([pitch_limit_w, pitch_limit_d, support_h], center=true);

            rotate(45)
            translate([-pitch_limit_w / 2 - 4, 2.5, 0])
              cube([pitch_limit_w, pitch_limit_d, support_h], center=true);
          }

          translate([motor_mount_d / 2, motor_mount_d / 2, support_h / 2]) {
            rotate(-45)
            translate([-pitch_limit_w / 2 - 4, -pitch_limit_d / 2, 0])
              cube([pitch_limit_w, pitch_limit_d, support_h], center=true);

            rotate(-45)
            translate([-pitch_limit_w / 2 - 4, 2.5, 0])
              cube([pitch_limit_w, pitch_limit_d, support_h], center=true);
          }
        }
      }
    }

    // Mount holes
    hole_positions = [
      [encoder_mount_w / 2, encoder_mount_d / 2, encoder_standoff_h / 2],
      [encoder_mount_w / 2, -encoder_mount_d / 2, encoder_standoff_h / 2],
      [-encoder_mount_w / 2, encoder_mount_d / 2, encoder_standoff_h / 2],
      [-encoder_mount_w / 2, -encoder_mount_d / 2, encoder_standoff_h / 2]
    ];
    for (hole_pos = hole_positions) {
      translate(hole_pos) {
        cylinder(r=M25_SCREW_W / 2, h=encoder_standoff_h + 0.1, center=true);

        translate([0, 0, -encoder_standoff_h / 2 + M25_NUT_H / 2])
          cylinder(r=M25_NUT_W / 2, h=M25_NUT_H + 0.1, $fn=6, center=true);
      }
    }

    // Pitch limiter holes
    if (pitch_limit) {
      rotate(pitch_limit_offset) {
        translate([motor_mount_d / 2, motor_mount_d / 2, support_h / 2])
          cylinder(r=M3_SCREW_W / 2, h=support_h + 0.1, center=true);
        translate([motor_mount_d / 2, -motor_mount_d / 2, support_h / 2])
          cylinder(r=M3_SCREW_W / 2, h=support_h + 0.1, center=true);
      }
    }

    if (roll_limit) {
      // GM3506
      motor_r = 40.0 / 2.0 + 0.5;
      motor_h = 30.0;
      translate([0, 0, motor_h / 2 + motor_standoff_h + 0.01])
        cylinder(r=motor_r, h=motor_h, center=true);

      // Extra cuts for encoder board
      cutt_h = 10;
      translate([encoder_mount_w / 2, encoder_mount_d / 2, cutt_h / 2 + support_h + 0.5])
        cylinder(r=M25_SCREW_W +2.5, h = cutt_h, center=true);
      translate([-encoder_mount_w / 2, encoder_mount_d / 2, cutt_h / 2 + support_h + 0.5])
        cylinder(r=M25_SCREW_W +2.5, h = cutt_h, center=true);
    }

    // Chop
    // translate([0, -19, 0])
    // cube([50, 20, 10], center=true);
  }
}

module encoder_frame_GM4008(show_encoder=1, show_motor=0, has_encoders=0) {
  // AS5048
  encoder_mount_w = 18;
  encoder_mount_d = 11;
  motor_standoff_w = 5;
  motor_standoff_h = 10;
  encoder_standoff_w = 7;
  encoder_standoff_h = 4;

  // AS5600
  // encoder_mount_w = 16;
  // encoder_mount_d = 16;
  // motor_standoff_w = 5;
  // motor_standoff_h = 13;
  // encoder_standoff_w = 7;
  // encoder_standoff_h = 6;

  support_w = 4;
  support_h = 2;
  motor_mount_d = sqrt(pow(30.0, 2) + pow(30.0, 2)) / 2;

  // Yaw limiter
  limiter_w = 6;
  limiter_d = 15;
  limiter_h = 40;
  limiter_offset = 45 + 180;

  // Show motor
  if (show_motor) {
    rotate(45)
      translate([0, 0, motor_standoff_h + 0.01])
        gimbal_motor_GM4008();
  }

  // Show encoder
  if (show_encoder) {
    // translate([0, 0, encoder_standoff_h + 0.01]) encoder_AS5048();
    translate([0, 0, encoder_standoff_h + 0.01]) encoder_AS5600();
  }

  difference() {
      union() {
      // Encoder frame
      frame(encoder_mount_w, encoder_mount_d,
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

      // Pitch limiter
      rotate(limiter_offset) {
          rotate(35)
          translate([limiter_w, -limiter_d / 2, limiter_h / 2])
            translate([0, -12.5, 0])
            cube([limiter_w, limiter_d, limiter_h], center=true);

          rotate(270 - 35)
          translate([-limiter_w, -limiter_d / 2, limiter_h / 2])
            translate([0, -12.5, 0])
            cube([limiter_w, limiter_d, limiter_h], center=true);
      }
    }

    // Mount holes
    hole_positions = [
      [encoder_mount_w / 2, encoder_mount_d / 2, encoder_standoff_h / 2],
      [encoder_mount_w / 2, -encoder_mount_d / 2, encoder_standoff_h / 2],
      [-encoder_mount_w / 2, encoder_mount_d / 2, encoder_standoff_h / 2],
      [-encoder_mount_w / 2, -encoder_mount_d / 2, encoder_standoff_h / 2]
    ];
    for (hole_pos = hole_positions) {
      translate(hole_pos) {
        cylinder(r=M2_SCREW_W / 2, h=encoder_standoff_h + 0.1, center=true);

        translate([0, 0, -encoder_standoff_h / 2 + M2_NUT_H / 2])
          cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, $fn=6, center=true);
      }
    }

    // Yaw limiter holes
    rotate(limiter_offset) {
      translate([motor_mount_d / 2, -motor_mount_d / 2, limiter_h / 2])
        cylinder(r=M3_SCREW_W / 2, h=limiter_h + 0.1, center=true);
      translate([-motor_mount_d / 2, motor_mount_d / 2, limiter_h / 2])
        cylinder(r=M3_SCREW_W / 2, h=limiter_h + 0.1, center=true);
    }

    // Gimbal motor
    rotate(45) {
      motor_r = 46.0 / 2.0 + 0.5;
      motor_h = 21.0 + 10.0;
      translate([0, 0, motor_standoff_h + 0.01]) {
        cylinder(r=motor_r, h=motor_h);
      }
    }

    // // Chop
    // translate([0, -19, 0])
    // cube([50, 20, 10], center=true);
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

module usb_hub() {
  board_w = 87;
  board_d = 22;
  board_h = 2;

  usb_port_w = 13;
  usb_port_d = 14;
  usb_port_h = 8;


  difference() {
    union() {
      // Body
      color([0, 1, 0])
        translate([0, 0, board_h / 2])
          cube([board_d, board_w, board_h], center=true);

      // USB port 1
      union() {
        offset_x = usb_port_d / 2;
        offset_y = -board_w / 2 + usb_port_w / 2 + 2;
        offset_z = usb_port_h / 2 + board_h;
        color([0.5, 0.5, 0.5])
        translate([offset_x, offset_y, offset_z])
          cube([usb_port_d, usb_port_w, usb_port_h], center=true);
      }

      // USB port 2
      union() {
        offset_x = usb_port_d / 2;
        offset_y = -board_w / 2 + usb_port_w / 2 + 2 + 18.5;
        offset_z = usb_port_h / 2 + board_h;
        color([0.5, 0.5, 0.5])
        translate([offset_x, offset_y, offset_z])
          cube([usb_port_d, usb_port_w, usb_port_h], center=true);
      }

      // USB port 3
      union() {
        offset_x = usb_port_d / 2;
        offset_y = -board_w / 2 + usb_port_w / 2 + 2 + 18.5 * 2;
        offset_z = usb_port_h / 2 + board_h;
        color([0.5, 0.5, 0.5])
        translate([offset_x, offset_y, offset_z])
          cube([usb_port_d, usb_port_w, usb_port_h], center=true);
      }

      // USB port 4
      union() {
        offset_x = usb_port_d / 2;
        offset_y = -board_w / 2 + usb_port_w / 2 + 2 + 18.5 * 3;
        offset_z = usb_port_h / 2 + board_h;
        color([0.5, 0.5, 0.5])
        translate([offset_x, offset_y, offset_z])
          cube([usb_port_d, usb_port_w, usb_port_h], center=true);
      }
    }

    // Mount holes
    translate([0, -board_w / 2 + 17, board_h / 2])
      cylinder(r= 4 / 2, h=board_h + 0.01, center=true);
    translate([1, -board_w / 2 + 72.5, board_h / 2])
      cylinder(r= 4 / 2, h=board_h + 0.01, center=true);
  }
}

module gimbal_yaw_limiter() {
  support_w = 4.0;
  support_h = 2;
  motor_standoff_w = 5.0;
  motor_standoff_h = 2.0;
  motor_mount_d = sqrt(pow(30.0, 2) + pow(30.0, 2)) / 2;
  motor_h = 21.0;

  limiter_w = 6;
  limiter_h = 30;
  limiter_d = 46 / 2 + (2) + 0.5;
  limiter_offset_w = 45.0 + limiter_w + 1;

  difference() {
    union() {
      // Mount frame
      frame(motor_mount_d, motor_mount_d,
            M3_SCREW_W, M3_NUT_W, M3_NUT_H,
            motor_standoff_w, motor_standoff_h,
            support_w, support_h);

      // Horizontal support
      translate([limiter_d / 2, 0, support_h / 2])
        cube([limiter_d, limiter_w, support_h], center=true);
      translate([0, 0, support_h / 2])
        cube([motor_mount_d, limiter_w, support_h], center=true);

      // Limiter
      translate([46 / 2 + (2) / 2 + 0.5, 0, -limiter_h / 2])
        cube([2, limiter_w, limiter_h], center=true);
    }
  }

}

module pololu_psu() {
  pcb_w = 48;
  pcb_d = 15;
  pcb_h = 2;

  term_w = 10;
  term_d = 8;
  term_h = 10;

  difference() {
    union() {
      // PCB
      color([0, 1, 0])
        translate([0, 0, pcb_h / 2])
          cube([pcb_d, pcb_w, pcb_h], center=true);

      // Screw terminal
      color([0, 0, 1])
        rotate(90)
          translate([pcb_w / 2 - term_d / 2 + 1, -3, term_h / 2 + pcb_h])
            cube([term_d, term_w, term_h], center=true);

      // Screw terminal
      color([0, 0, 1])
        rotate(-90)
          translate([pcb_w / 2 - term_d / 2 + 1, -3, term_h / 2 + pcb_h])
            cube([term_d, term_w, term_h], center=true);
    }

    // Mount Holes
    psu_holes = [
      [POLOLU_MOUNT_D / 2,  -POLOLU_MOUNT_W / 2, pcb_h / 2],
      [-POLOLU_MOUNT_D / 2, POLOLU_MOUNT_W / 2, pcb_h / 2]
    ];

    for (pos = psu_holes) {
      translate(pos)
        cylinder(r=M2_SCREW_W / 2, h=pcb_h + 0.1, center=true);
    }
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
      //     usb_hub();
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

module intel_realsense_frame(show_camera=0) {
  frame_thickness = 6;
  frame_w = 90;
  frame_d = 20;

  mount_w = 30;
  mount_d = 6;
  mount_hole_w = 12;

  module_w = 64.7 + 6;
  processor_w = 72.0;
  imu_w = 20.0;
  imu_standoff_h = 4.0;

  // Show camera
  if (show_camera) {
    // D430 Depth Module
    translate([0, -frame_w / 2 + module_w / 2, frame_thickness])
      intel_D430_depth_module();

    // D4 Vision Processor
    translate([0, -frame_w / 2 + processor_w / 2 + 10, 0])
      rotate([0, -180, 0])
        intel_D4_vision_processor();

    // Gimbal IMU
    translate([0, frame_w / 2 - imu_w / 2 + 1, frame_thickness + imu_standoff_h])
      rotate([0, 0, 180])
        gimbal_imu();
  }


  difference() {
    union() {
      // Frame body
      translate([0, 0, frame_thickness / 2])
        cube([frame_d, frame_w, frame_thickness], center=true);

      // Mount Lip
      translate([-frame_d / 2 - mount_d / 2, 0, frame_thickness / 2])
        cube([mount_d, mount_w, frame_thickness], center=true);

      // Gimbal IMU standoff
      translate([0, frame_w / 2 - imu_w / 2 + 1, frame_thickness / 2])
      rotate([0, 0, 180]) {
        tol = 0.1;
        w = 20.0;
        d = 18.0;
        h = imu_standoff_h;
        mount_w = 14.0;

        // Mount holes
        translate([d / 2 - 7.5, mount_w / 2, frame_thickness / 2 + h / 2])
          cylinder(r=M3_SCREW_W / 2 + 0.5, h=h + tol, center=true);
        translate([d / 2 - 7.5, -mount_w / 2, frame_thickness / 2 + h / 2])
          cylinder(r=M3_SCREW_W / 2 + 0.5, h=h + tol, center=true);
      }
    }

    // Mount lip holes
    translate([-frame_d / 2 - mount_d / 2, mount_hole_w / 2, frame_thickness / 2])
      cylinder(r=M2_SCREW_W / 2, h=frame_thickness + 0.1, center=true);
    translate([-frame_d / 2 - mount_d / 2, -mount_hole_w / 2, frame_thickness / 2])
      cylinder(r=M2_SCREW_W / 2, h=frame_thickness + 0.1, center=true);

    // D430 depth module mount holes
    translate([0, -frame_w / 2 + module_w / 2 + 1, frame_thickness - 1]) {
      module_w = 64.7;
      lip_w = 3.0;
      lip_d = 6.0;
      lip_h = 1.0;
      lip_screw_d = 1.8 - 0.1;
      lip_screw_h = 4.0;

      // Left lip hole
      translate([0, module_w/ 2 + 1.8, lip_h - lip_screw_h / 2])
        cylinder(h=lip_screw_h + 0.1, r=lip_screw_d / 2, center=true);

      // Right lip hole
      translate([0, -module_w/ 2 - lip_screw_d / 2 - 1, lip_h - lip_screw_h / 2])
        cylinder(h=lip_screw_h + 0.1, r=lip_screw_d / 2, center=true);
    }

    // D4 vision processor mount holes
    translate([0.5, -frame_w / 2 + processor_w / 2 + 10, 0])
    rotate([0, -180, 0])
    {
      pcb_w = 72.0;
      pcb_d = 16.0;
      pcb_h = 1;

      usbc_w = 9.0;
      usbc_d = 8.0;
      usbc_h = 3.5;

      screw_w = M2_SCREW_W;

      // // Hole 1
      // hole1_offset_x = -pcb_d / 2 + 0.6;
      // hole1_offset_y = pcb_w / 2 - 67.5;
      // translate([hole1_offset_x, hole1_offset_y, -frame_thickness / 2])
      //   cylinder(r=screw_w / 2, h=frame_thickness + 0.1, center=true);
      // translate([hole1_offset_x, hole1_offset_y, -frame_thickness + M2_NUT_H / 2])
      //   cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, center=true, $fn=6);

      // Hole 2
      hole2_offset_x = -pcb_d / 2 + 1.5;
      hole2_offset_y = pcb_w / 2 - 3.5;
      translate([hole2_offset_x, hole2_offset_y, -frame_thickness / 2])
        cylinder(r=screw_w / 2, h=frame_thickness + 0.1, center=true);
      translate([hole2_offset_x, hole2_offset_y, -frame_thickness + M2_NUT_H / 2])
        cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, center=true, $fn=6);

      // Hole 3
      hole3_offset_x = pcb_d / 2 - 1;
      hole3_offset_y = pcb_w / 2 - 62;
      translate([hole3_offset_x, hole3_offset_y, -frame_thickness / 2])
        cylinder(r=screw_w / 2, h=frame_thickness + 0.1, center=true);
      translate([hole3_offset_x, hole3_offset_y, -frame_thickness + M2_NUT_H / 2])
        cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, center=true, $fn=6);

      // // Hole 4
      // hole4_offset_x = pcb_d / 2 - 1.7;
      // hole4_offset_y = pcb_w / 2 - 10;
      // translate([hole4_offset_x, hole4_offset_y, -frame_thickness / 2])
      //   cylinder(r=screw_w / 2, h=frame_thickness + 0.1, center=true);
      // translate([hole4_offset_x, hole4_offset_y, -frame_thickness + M2_NUT_H / 2])
      //   cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, center=true, $fn=6);
    }

    // Gimbal IMU mount holes
    translate([0, frame_w / 2 - imu_w / 2 + 1, frame_thickness])
    rotate([0, 0, 180]) {
      tol = 0.2;
      w = 20.0;
      d = 18.0;
      h = imu_standoff_h;
      mount_w = 14.0;

      // Mount holes
      translate([d / 2 - 7.5, mount_w / 2, h / 2])
        cylinder(r=M2_SCREW_W / 2 - 0.2, h=h + tol, center=true);
      translate([d / 2 - 7.5, -mount_w / 2, h / 2])
        cylinder(r=M2_SCREW_W / 2 - 0.2, h=h + tol, center=true);
    }
  }
}

module intel_realsense_frame2(show_camera=0) {
  frame_thickness = 6;
  frame_w = 95;
  frame_d = 20;

  mount_w = 30;
  mount_d = 6;
  mount_hole_w = 12;

  module_w = 64.7 + 6;
  processor_w = 72.0;
  rs_offset = 11;

  // Show camera
  if (show_camera) {
    // D430 Depth Module
    translate([0, -frame_w / 2 + module_w / 2 + rs_offset, frame_thickness])
      intel_D430_depth_module();

    // D4 Vision Processor
    translate([0, -frame_w / 2 + processor_w / 2 + 10 + rs_offset, 0])
      rotate([0, -180, 0])
        intel_D4_vision_processor();
  }


  difference() {
    union() {
      // Frame body
      translate([0, 0, frame_thickness / 2])
        cube([frame_d, frame_w, frame_thickness], center=true);

      // Mount cylinders
      rotate([0, 90, 0])
        translate([-frame_thickness / 2, frame_w / 2, 0])
          cylinder(r=M3_SCREW_W, h=frame_d, center=true);
      rotate([0, 90, 0])
        translate([-frame_thickness / 2, -frame_w / 2, 0])
          cylinder(r=M3_SCREW_W, h=frame_d, center=true);
    }

    // D430 depth module mount holes
    translate([0, -frame_w / 2 + module_w / 2 + rs_offset, frame_thickness - 1]) {
      module_w = 64.7;
      lip_w = 3.0;
      lip_d = 6.0;
      lip_h = 1.0;
      lip_screw_d = 1.8 - 0.1;
      lip_screw_h = 4.0;

      // Left lip hole
      translate([0, module_w/ 2 + 1.8, lip_h - lip_screw_h / 2])
        cylinder(h=lip_screw_h + 0.1, r=lip_screw_d / 2, center=true);

      // Right lip hole
      translate([0, -module_w/ 2 - lip_screw_d / 2 - 1, lip_h - lip_screw_h / 2])
        cylinder(h=lip_screw_h + 0.1, r=lip_screw_d / 2, center=true);
    }

    // D4 vision processor mount holes
    translate([0.5, -frame_w / 2 + processor_w / 2 + 10 + rs_offset, 0])
    rotate([0, -180, 0])
    {
      pcb_w = 72.0;
      pcb_d = 16.0;
      pcb_h = 1;

      usbc_w = 9.0;
      usbc_d = 8.0;
      usbc_h = 3.5;

      screw_w = M2_SCREW_W;

      // // Hole 1
      // hole1_offset_x = -pcb_d / 2 + 0.6;
      // hole1_offset_y = pcb_w / 2 - 67.5;
      // translate([hole1_offset_x, hole1_offset_y, -frame_thickness / 2])
      //   cylinder(r=screw_w / 2, h=frame_thickness + 0.1, center=true);
      // translate([hole1_offset_x, hole1_offset_y, -frame_thickness + M2_NUT_H / 2])
      //   cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, center=true, $fn=6);

      // Hole 2
      hole2_offset_x = -pcb_d / 2 + 1.5;
      hole2_offset_y = pcb_w / 2 - 4.5;
      translate([hole2_offset_x, hole2_offset_y, -frame_thickness / 2])
        cylinder(r=screw_w / 2, h=frame_thickness + 0.1, center=true);
      translate([hole2_offset_x, hole2_offset_y, -frame_thickness + M2_NUT_H / 2])
        cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, center=true, $fn=6);

      // Hole 3
      hole3_offset_x = pcb_d / 2;
      hole3_offset_y = pcb_w / 2 - 62;
      translate([hole3_offset_x, hole3_offset_y, -frame_thickness / 2])
        cylinder(r=screw_w / 2, h=frame_thickness + 0.1, center=true);
      translate([hole3_offset_x, hole3_offset_y, -frame_thickness + M2_NUT_H / 2])
        cylinder(r=M2_NUT_W / 2, h=M2_NUT_H + 0.1, center=true, $fn=6);
    }

    // Mount cylinders
    {
      rotate([0, 90, 0])
        translate([-frame_thickness / 2, frame_w / 2, 0])
          cylinder(r=M3_SCREW_W / 2, h=frame_d + 0.1, center=true);
      rotate([0, 90, 0])
        translate([-frame_thickness / 2, -frame_w / 2, 0])
          cylinder(r=M3_SCREW_W / 2, h=frame_d + 0.1, center=true);
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

  support_x = motor_r + 3;
  support_w = 8;
  support_d = 6;
  support_h = 41;
  support_hole_h = 35;

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
      rotate([0, 0, 90])
        translate([13, support_h / 2 + hole_offset - 3, support_x + support_d / 2])
          intel_realsense_frame(show_camera=1);
          // oak_d_lite_frame(show_camera=1);
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
    translate([support_x, 0, support_hole_h / 2 + hole_offset]) {
      translate([0, 0, hole_w / 2]) {
        rotate([0.0, 90.0, 0.0])
          cylinder(r=M2_SCREW_W / 2, h=support_d + 0.1, center=true);
      }
      translate([0, 0, -hole_w / 2]) {
        rotate([0.0, 90.0, 0.0])
          cylinder(r=M2_SCREW_W / 2, h=support_d + 0.1, center=true);
      }
    }

    // Hex Holes
    hex_x = support_x - support_d / 2 + M25_NUT_H / 2 - 0.01;
    hex_y = 0.0;
    hex_z = support_hole_h / 2 + hole_offset;
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

  // Limiter
  limiter_w = 6;
  limiter_d = 10;
  limiter_h = 6;
  limiter_reach_h = 10;
  limiter_offset_x = -4;

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
      rotate([90, 45, 0])
        encoder_frame_GM3506(show_motor=1, pitch_limit=1);

    // GM4008
    // translate([0, pitch_mount_offset_y + motor_offset_y - roll_mount_h / 2, motor_offset_z])
    //   rotate([90, 0, 0])
    //     encoder_frame_GM4008(show_motor=1);
  }

  // Show pitch frame
  if (show_pitch_frame) {
    // y = motor_offset_y - 16;
    y = motor_offset_y - 16.5;
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

      // Roll Limiter
      rotate(180) {
        translate([motor_r + limiter_d / 2 + limiter_offset_x, 0, limiter_h / 2])
          cube([limiter_d + 0.5, limiter_w, limiter_h], center=true);
        translate([motor_r + limiter_w / 2 + 0.25, 0, -limiter_reach_h / 2])
          cube([limiter_w, limiter_w, limiter_reach_h], center=true);
      }
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
    pitch_holes_y = pitch_mount_offset_y + motor_offset_y + roll_mount_h / 2 - pitch_mount_d /2;
    pitch_holes_z = motor_offset_z;
    translate([0, pitch_holes_y, pitch_holes_z]) {
      rotate([90, 0, 0]) {
        for (i = [45:90:360]) {
          rotate([0.0, 0.0, i]) {
            translate([motor_base_mount_w / 2.0, 0, -0.1]) {
              cylinder(r=M25_SCREW_W / 2, h=10, center=true);
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
  yaw_encoder_h = 10;

  // Support
  frame_thickness = 8;
  support_w = 30.0;
  support_h = 8.0;

  // Roll mount offset
  roll_mount_x = -45;
  roll_mount_z = 50;

  // Limiter
  limiter_w = 6;
  limiter_h = frame_thickness;

  // Show roll frame
  if (show_roll_frame) {
    // Roll motor
    encoder_offset_x = roll_mount_x + (frame_thickness - 2) / 2;
    encoder_offset_z = roll_mount_z;
    translate([encoder_offset_x, 0.0, encoder_offset_z])
      rotate([0, 90, 0])
        rotate([0, 0, -90])
          encoder_frame_GM3506(show_encoder=1, show_motor=1, roll_limit=1);
          // encoder_frame_GM4008(show_encoder=1, show_motor=1);

    // Roll frame
    offset_x = encoder_offset_x + roll_motor_h + roll_encoder_h + 1;
    offset_z = roll_mount_z;
    roll_angle = 0.0;
    translate([offset_x, 0.0, offset_z])
      rotate([0, 90, 0])
        rotate([0, 0, roll_angle])
          gimbal_roll_frame(show_roll_motor=0);
  }

  // Show yaw motor
  if (show_yaw_motor) {
    translate([0, 0, -yaw_motor_h - yaw_encoder_h])
      rotate([0, 0, -90])
        // encoder_frame_GM3506(show_encoder=1, show_motor=1);
        encoder_frame_GM4008(show_encoder=1, show_motor=1);

    translate([0, 0, frame_thickness])
      rotate(80)
      #gimbal_yaw_limiter();

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
          cylinder(r=roll_motor_r, h=frame_thickness - 2, center=true);

      // SBGC frame
      translate([roll_mount_x + 1, 0.0, roll_mount_z])
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
        cube([frame_thickness - 2, support_w, roll_mount_z], center=true);

      // Diagonal supports
      translate([roll_mount_x / 2 - 10, support_w / 2 - frame_thickness / 2, roll_mount_z / 2 - 12])
        rotate([0, 35, 0])
          cube([30, frame_thickness, frame_thickness], center=true);
      translate([roll_mount_x / 2 - 10, -support_w / 2 + frame_thickness / 2, roll_mount_z / 2 - 12])
        rotate([0, 35, 0])
          cube([30, frame_thickness, frame_thickness], center=true);

      // Yaw Stopper
      // translate([yaw_motor_r, 0.0, frame_thickness / 2])
      //   cube([limiter_w + 3, limiter_w, frame_thickness], center=true);
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

  // // Cable rails
  // rail_offset = 10.0;
  // difference() {
  //   translate([0, (mount_d / 2) + 4, nuc_support_h / 2])
  //     cube([mount_w - 5, rail_offset, nuc_support_h], center=true);
  //   translate([0, (mount_d / 2) + 4, nuc_support_h / 2])
  //     cube([mount_w - 10, 5, nuc_support_h + 0.01], center=true);
  // }
  // difference() {
  //   translate([0, -(mount_d / 2) - 4, support_h / 2])
  //     cube([mount_w - 5, rail_offset, support_h], center=true);
  //   translate([0, -(mount_d / 2) - 4, support_h / 2])
  //     cube([mount_w - 10, 5, support_h + 0.01], center=true);
  // }
}

module base_frame(mount_w, mount_d, show_nuc=0) {
  nuc_standoff_w = 8;
  nuc_standoff_h = 6;
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
        M3_SCREW_W, M3_NUT_W, M3_NUT_H + 0.5,
        nuc_standoff_w, nuc_standoff_h,
        nuc_support_w, nuc_support_h,
        nut_csb=1);

  // Supports
  translate([0, 0, nuc_support_h / 2])
    cube([nuc_support_w, mount_d, nuc_support_h], center=true);
  translate([mount_w / 2 * 0.5, 0, nuc_support_h / 2])
    cube([nuc_support_w, mount_d, nuc_support_h], center=true);
  translate([-mount_w / 2 * 0.5, 0, nuc_support_h / 2])
    cube([nuc_support_w, mount_d, nuc_support_h], center=true);

  translate([0, 0, nuc_support_h / 2])
    cube([mount_w, nuc_support_w, nuc_support_h], center=true);
  translate([0, mount_d / 2 * 0.5, nuc_support_h / 2])
    cube([mount_w, nuc_support_w, nuc_support_h], center=true);
  translate([0, -mount_d / 2 * 0.5, nuc_support_h / 2])
    cube([mount_w, nuc_support_w, nuc_support_h], center=true);
}

module utility_frame(mount_w, mount_d, show_components=1) {
  psu_standoff_h = 2;
  hub_standoff_h = 5;
  hub_w = 87;
  hub_d = 22;
  hub_h = 2;

  if (show_components) {
    translate([-30, 0, support_h + hub_standoff_h])
      usb_hub();

    translate([42.5, 0, support_h + psu_standoff_h])
      pololu_psu();
  }


  difference() {
    union() {
      // Utility frame
      frame(mount_w, mount_d,
            M3_SCREW_W, M3_NUT_W, M3_NUT_H,
            standoff_w, support_h,
            support_w - 2, support_h);

      // USB HUB Supports
      rotate(90)
      translate([-31, 0, support_h / 2]) {
        // USB Hub Horizontal support
        translate([0.5, 0, 0])
        cube([support_w, mount_w, support_h], center=true);

        // USB HUB vertical support
        translate([0, -hub_w / 2 + 17, support_h / 2 + hub_standoff_h / 2])
          cylinder(r= M3_SCREW_W / 2 + 1, h=hub_standoff_h, center=true);
        translate([1, -hub_w / 2 + 72.5, support_h / 2 + hub_standoff_h / 2])
          cylinder(r= M3_SCREW_W / 2 + 1, h=hub_standoff_h, center=true);
      }

      // PSU horizontal support
      rotate(90)
      translate([35, 0, support_h / 2])
        cube([support_w - 2, mount_w, support_h], center=true);

      // PSU vertical support
      rotate(90)
      translate([40, 0, support_h / 2 + psu_standoff_h / 2]) {
        psu_holes = [
          [POLOLU_MOUNT_D / 2,  -POLOLU_MOUNT_W / 2, support_h / 2],
          [-POLOLU_MOUNT_D / 2, POLOLU_MOUNT_W / 2, support_h / 2]
        ];

        for (pos = psu_holes) {
          translate(pos)
            cylinder(r=M2_SCREW_W / 2 + 0.5, h=psu_standoff_h, center=true);
        }
      }

      // Supports
      translate([0, 0, support_h / 2])
        cube([support_w * 0.5, mount_d, support_h], center=true);
      translate([mount_w / 2 * 0.5, 0, support_h / 2])
        cube([support_w * 0.5, mount_d, support_h], center=true);
      translate([-mount_w / 2 * 0.5, 0, support_h / 2])
        cube([support_w * 0.5, mount_d, support_h], center=true);

      translate([0, 0, support_h / 2])
        cube([mount_w, support_w * 0.5, support_h], center=true);
      translate([0, mount_w / 2 * 0.5, support_h / 2])
        cube([mount_w, support_w * 0.5, support_h], center=true);
      translate([0, -mount_w / 2 * 0.5, support_h / 2])
        cube([mount_w, support_w * 0.5, support_h], center=true);
    }

    // USB hub support holes
    rotate(90)
    translate([-31, 0, support_h / 2]) {
      // USB HUB vertical support
      translate([0, -hub_w / 2 + 17, support_h / 2 + hub_standoff_h / 2])
        cylinder(r= M3_SCREW_W / 2 - 0.1, h=20, center=true);
      translate([1, -hub_w / 2 + 72.5, support_h / 2 + hub_standoff_h / 2])
        cylinder(r= M3_SCREW_W / 2 - 0.1, h=20, center=true);
    }

    // PSU vertical support
    rotate(90)
    translate([40, 0, support_h / 2 + psu_standoff_h / 2]) {
      psu_holes = [
        [POLOLU_MOUNT_D / 2,  -POLOLU_MOUNT_W / 2, support_h / 2],
        [-POLOLU_MOUNT_D / 2, POLOLU_MOUNT_W / 2, support_h / 2]
      ];

      for (pos = psu_holes) {
        translate(pos)
          cylinder(r=M2_SCREW_W / 2 - 0.5, h=100, center=true);
      }
    }
  }
}

module mav_top_plate(show_arms=0, show_nuc=1) {
  thickness = 4;
  wheelbase = 500;
  plate_w = 120;
  mount_w = 30;
  mount_d = 30;

  // MAV Arms
  if (show_arms) {
    for (i = [1:4]) {
      rotate(45)
        rotate(90 * i)
          translate([-60, 0, 0])
            S500_arm();
    }
  }

  // Intel NUC
  if (show_nuc) {
    rotate([90.0, 0.0, 0.0])
      import("../../proto_parts/Intel_NUC7i5DN/NUC7i5DN.STL");
  }

  // Main plate
  difference() {
    union() {
      // Plate
      // -- Outer support
      for (i = [1:4]) {
        rotate(90 * i)
          translate([52, 0, thickness / 2])
            cube([10, 69.3, thickness], center=true);
      }
      // -- Inner support
      for (i = [1:4]) {
        rotate(90 * i)
          translate([25, 0, thickness / 2])
            cube([10, 50, thickness], center=true);
      }
      // -- Spinal support
      for (i = [0:1]) {
        rotate(90 * i)
        translate([0, 0, thickness / 2])
          cube([10, 114, thickness], center=true);
      }

      for (i = [1:4]) {
        rotate(45)
        rotate(90 * i)
        translate([-60, 0, 0]) {
        // Mount top-left
        translate([0, -mount_w / 2, thickness / 2])
          cylinder(r=M25_SCREW_W / 2 * 2, h=thickness, center=true);

        // Mount top-right
        translate([0, mount_w / 2, thickness / 2])
          cylinder(r=M25_SCREW_W / 2 * 2, h=thickness, center=true);

        // Mount bottom
        translate([mount_d, 0, thickness / 2])
          cylinder(r=M25_SCREW_W / 2 * 4, h=thickness, center=true);

        // Mount supports
        translate([0, 0, thickness / 2])
          cube([thickness + 5, mount_w + 2, thickness], center=true);
        translate([15, -8, thickness / 2])
          rotate(-63)
          cube([thickness + 4, 32, thickness], center=true);
        translate([15, 8, thickness / 2])
          rotate(63)
          cube([thickness + 4, 32, thickness], center=true);
        }
      }
    }

    // Arm mount holes
    for (i = [1:4]) {
      rotate(45)
      rotate(90 * i)
      translate([-60, 0, 0]) {
        // Hole top-left
        translate([0, -mount_w / 2, thickness / 2])
          cylinder(r=M25_SCREW_W / 2, h=thickness + 0.1, center=true);

        // Hole top-right
        translate([0, mount_w / 2, thickness / 2])
          cylinder(r=M25_SCREW_W / 2, h=thickness + 0.1, center=true);

        // Hole bottom
        translate([mount_d, 0, thickness / 2])
          cylinder(r=M25_SCREW_W / 2, h=thickness + 0.1, center=true);
      }
    }

    // Module mount holes
    for (i = [0:3]) {
      rotate(90 * i)
        translate([25, 0, thickness / 2])
        cylinder(r=M2_SCREW_W / 2, h=thickness + 0.1, center=true);
    }
  }
}

module mav_bottom_plate(show_arms=0) {
  thickness = 4;
  wheelbase = 500;
  plate_w = 120;
  mount_w = 20;
  mount_d = 30;

  // MAV Arms
  if (show_arms) {
    for (i = [1:4]) {
      rotate(45)
        rotate(90 * i)
          translate([-60, 0, 19 + thickness])
            S500_arm();
    }
  }

  // Main plate
  difference() {
    union() {
      // Plate
      // -- Outer support
      for (i = [1:4]) {
        rotate(90 * i)
          translate([52, 0, thickness / 2])
            cube([10, 69.3, thickness], center=true);
      }
      // -- Inner support
      for (i = [1:4]) {
        rotate(90 * i)
          translate([25, 0, thickness / 2])
            cube([10, 50, thickness], center=true);
      }
      // -- Spinal support
      for (i = [1:4]) {
        rotate(90 * i)
        translate([36, 0, thickness / 2])
          cube([5, 36, thickness], center=true);

        rotate(90 * i)
        translate([45, 0, thickness / 2])
          cube([6, 36, thickness], center=true);
      }
      translate([22, 0, thickness / 2])
        cube([10, 114, thickness], center=true);
      translate([-22, 0, thickness / 2])
        cube([10, 114, thickness], center=true);
      translate([0, 22, thickness / 2])
        cube([114, 10, thickness], center=true);
      translate([0, -22, thickness / 2])
        cube([114, 10, thickness], center=true);

      for (i = [1:4]) {
        rotate(45 + i * 90)
        translate([49, 0, thickness / 2])
        cylinder(r=12, h=thickness, center=true);
      }

      // for (i = [1:4]) {
      //   rotate(45 + i * 90)
      //   translate([60, 0, thickness / 2]) {
      //     translate([-14, 6, 2]) {
      //       cylinder(r=M25_SCREW_W * 0.8, h=thickness + 0.1, center=true);
      //     }
      //     translate([-14, -6, 2]) {
      //       cylinder(r=M25_SCREW_W * 0.8, h=thickness + 0.1, center=true);
      //     }
      //   }
      // }

      // Fill middle
      translate([0, 0, thickness / 2])
        cube([35, 35, thickness], center=true);

      for (i = [1:4]) {
        rotate(45)
        rotate(90 * i)
        translate([-60, 0, 0]) {
        // Mount top-left
        translate([0, -mount_w / 2, thickness / 2])
          cylinder(r=M25_SCREW_W / 2 * 2, h=thickness, center=true);

        // Mount top-right
        translate([0, mount_w / 2, thickness / 2])
          cylinder(r=M25_SCREW_W / 2 * 2, h=thickness, center=true);

        // Mount bottom
        translate([mount_d, 0, thickness / 2])
          cylinder(r=M25_SCREW_W / 2 * 4, h=thickness, center=true);

        // Mount supports
        translate([0, 0, thickness / 2])
          cube([thickness + 5.5, mount_w + 12, thickness], center=true);
        translate([15, -8, thickness / 2])
          rotate(-63)
          cube([thickness + 4, 32, thickness], center=true);
        translate([15, 8, thickness / 2])
          rotate(63)
          cube([thickness + 4, 32, thickness], center=true);
        }
      }
    }

    // Arm mount holes
    for (i = [1:4]) {
      rotate(45)
      rotate(90 * i)
      translate([-60, 0, 0]) {
        // Hole top-left
        translate([0, -mount_w / 2, thickness / 2])
          cylinder(r=M25_SCREW_W / 2, h=thickness + 0.1, center=true);

        // Hole top-right
        translate([0, mount_w / 2, thickness / 2])
          cylinder(r=M25_SCREW_W / 2, h=thickness + 0.1, center=true);

        // // Hole bottom
        // translate([mount_d, 0, thickness / 2])
        //   cylinder(r=M25_SCREW_W / 2, h=thickness + 0.1, center=true);
      }
    }

    // FPV stack mount holes
    translate([30.5 / 2, 30.5 / 2, thickness / 2])
      cylinder(r=M3_SCREW_W / 2, h=thickness + 0.1, center=true);
    translate([30.5 / 2, -30.5 / 2, thickness / 2])
      cylinder(r=M3_SCREW_W / 2, h=thickness + 0.1, center=true);
    translate([-30.5 / 2, 30.5 / 2, thickness / 2])
      cylinder(r=M3_SCREW_W / 2, h=thickness + 0.1, center=true);
    translate([-30.5 / 2, -30.5 / 2, thickness / 2])
      cylinder(r=M3_SCREW_W / 2, h=thickness + 0.1, center=true);

    // Module mount holes
    for (i = [0:3]) {
      rotate(90 * i)
        translate([23.5, 0, thickness / 2])
        cylinder(r=M3_SCREW_W / 2, h=thickness + 0.1, center=true);
    }

    // Cable rails
    for (i = [0:3]) {
      rotate(90 * i)
        translate([53, 0, thickness / 2])
        cube([2, 48, thickness + 0.1], center=true);
    }

    // Landing leg hole
    for (i = [1:4]) {
      rotate(45 + i * 90)
      translate([60, 0, thickness / 2]) {
        translate([-14, 6, 0]) {
          cylinder(r=M25_SCREW_W / 2, h=thickness + 0.1, center=true);
        }
        translate([-14, -6, 0]) {
          cylinder(r=M25_SCREW_W / 2, h=thickness + 0.1, center=true);
        }

        // translate([-14, 6, thickness - 1]) {
        //   cylinder(r=M25_SCREW_W / 2, h=2 + 0.2, center=true, $fn=6);
        // }
        // translate([-14, -6, thickness - 1]) {
        //   cylinder(r=M25_SCREW_W / 2, h=2 + 0.2, center=true, $fn=6);
        // }
      }
    }
  }
}


/////////////////////////////////////////////////////////////////////////////
// ASSEMBLY                                                                //
/////////////////////////////////////////////////////////////////////////////

module gimbal_assembly() {
  // Gimbal frame
  gimbal_frame(nuc_mount_w, nuc_mount_d, 0);

  // GM4008
  yaw_motor_r = 46.0 / 2;
  yaw_motor_h = 21.0;
  yaw_motor_mount_w = 30.0;
  yaw_motor_mount_h = 5;
  yaw_encoder_h = 10;

  // Gimbal
  offset_z = yaw_motor_h + yaw_encoder_h + standoff_h;
  translate([0, 0, offset_z])
    gimbal_yaw_frame(show_roll_frame=1, show_yaw_motor=1, show_sbgc_frame=1);
}

module perception_module_assembly() {
  // Gimbal
  translate([0, 0, 56]) {
    gimbal_assembly();
  }

  // NUC
  translate([0, 0, 20]) {
    // NUC frame
    nuc_frame(nuc_mount_w, nuc_mount_d, show_nuc=1);

    // Spacers
    nuc_standoff_h = 16;
    translate([nuc_mount_w / 2, nuc_mount_d / 2, nuc_standoff_h + 2])
      spacer(M3_SCREW_W + 3, 18.0, M3_SCREW_W);
    translate([-nuc_mount_w / 2, nuc_mount_d / 2, nuc_standoff_h + 2])
      spacer(M3_SCREW_W + 3, 18.0, M3_SCREW_W);
    translate([nuc_mount_w / 2, -nuc_mount_d / 2, nuc_standoff_h + 2])
      spacer(M3_SCREW_W + 3, 18.0, M3_SCREW_W);
    translate([-nuc_mount_w / 2, -nuc_mount_d / 2, nuc_standoff_h + 2])
      spacer(M3_SCREW_W + 3, 18.0, M3_SCREW_W);
  }
  utility_frame(nuc_mount_w, nuc_mount_d, show_components=1);
}

// gimbal_assembly();
// perception_module_assembly();

/////////////////////////////////////////////////////////////////////////////
// COMPONENT DEVELOPMENT                                                   //
/////////////////////////////////////////////////////////////////////////////

// battery_frame(batt_frame_w, batt_frame_d);
// fcu_frame(1);
// encoder_frame_GM2804(show_encoder=0, show_motor=0);
// encoder_frame_GM3506(show_encoder=0, show_motor=0, roll_limit=0, pitch_limit=1);
// encoder_frame_GM4008(show_encoder=0, show_motor=0);
// odroid_frame(nuc_mount_w, nuc_mount_d, 0);
// gimbal_yaw_limiter();

// -- COMPONENTS
// lipo_battery();
// encoder_AS5600();
// encoder_AS5048();
// board_camera();
// arducam_MT9V034();
// intel_D430_depth_module();
// intel_D4_vision_processor();
// oak_d_lite();
// usb_hub();
// pololu_psu();
// sbgc_board();
// gimbal_motor_GM2804();
// gimbal_motor_GM3506();
// gimbal_motor_GM4008();
// gimbal_imu();
// encoder_magnet_holder_GM3506();
// encoder_magnet_holder_GM4008();
// S500_arm();

// -- GIMBAL FRAMES
// gimbal_frame(nuc_mount_w, nuc_mount_d, 0);
// gimbal_pitch_frame(show_pitch_motor=0, show_camera=0);
// #rotate(90 + 35)
// gimbal_roll_frame(show_roll_motor=0, show_pitch_motor=0, show_pitch_frame=0);
// gimbal_yaw_frame(show_roll_frame=0, show_yaw_motor=0, show_sbgc_frame=0);

// -- FRAMES
// intel_realsense_frame(show_camera=1);
// intel_realsense_frame2(show_camera=1);
// oak_d_lite_frame(show_camera=1);
// sbgc_frame(show_sbgc=0, show_usb_hub=0);
// pcb_frame(nuc_mount_w, nuc_mount_d);
// nuc_frame(nuc_mount_w, nuc_mount_d, show_nuc=0);
// base_frame(nuc_mount_w, nuc_mount_d, show_nuc=0);
// utility_frame(nuc_mount_w, nuc_mount_d, show_components=0);
// mav_top_plate(show_arms=1, show_nuc=0);
mav_bottom_plate(show_arms=0);

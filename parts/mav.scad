include <config.scad>
include <frame.scad>
include <perception_module.scad>

module mav_motor() {
  color([1.0, 0.0, 0.0]) {
    // Shaft
    translate([0, 0, 25.7 + 14.0 / 2]) cylinder(r=6 / 2, h=14.0, center=true);

    // Body
    difference() {
      translate([0, 0, 25.7 / 2]) cylinder(r=27.9 / 2, h=25.7, center=true);

        translate([16 / 2, 0.0, 2.0 / 2.0])
            cylinder(r=M3_screw_w / 2, h=2.0 + 0.1, center=true);
        translate([-16 / 2, 0.0, 2.0 / 2.0])
            cylinder(r=M3_screw_w / 2, h=2.0 + 0.1, center=true);
        translate([0.0, 19.0 / 2.0, 2.0 / 2.0])
            cylinder(r=M3_screw_w / 2, h=2.0 + 0.1, center=true);
        translate([0.0, -19.0 / 2.0, 2.0 / 2.0])
            cylinder(r=M3_screw_w / 2, h=2.0 + 0.1, center=true);
    }

    // Bottom shaft
    h = 42.2 - (25.7 + 14.0);
    r = 2.0;
    translate([0.0, 0.0, -h / 2])
      cylinder(r=r, h=h + 0.1, center=true);
  }

  translate([0.0, 0.0, 25.7 + 14 - 12]) {
    color([0.0, 0.0, 0.0])
    scale(25.4)
      translate([0.0, 0.0, 0.0])
        rotate([90.0, 0.0, 90.0])
          import("../proto_parts/Propeller_1045/Propeller_1045.STL");
  }
}

module motor_mount(w, d, h, show_motor=1) {
  standoff_w = 8.0;
  standoff_h = 5.0;
  support_w = 3.5;
  support_h = 3.0;

  // Show motor
  if (show_motor) {
    translate([0, 0, mav_motor_hole_h + 0.01]) mav_motor();
  }

  // Motor mount
  difference() {
    // Mount body
    union() {
      // Motor mounts
      translate([0, mav_motor_hole_w / 2, mav_motor_hole_h / 2])
        cylinder(r=4, h=mav_motor_hole_h, center=true);
      translate([0, -mav_motor_hole_w / 2, mav_motor_hole_h / 2])
        cylinder(r=4, h=mav_motor_hole_h, center=true);
      translate([mav_motor_hole_d / 2, 0, mav_motor_hole_h / 2])
        cylinder(r=4, h=mav_motor_hole_h, center=true);
      translate([-mav_motor_hole_d / 2, 0, mav_motor_hole_h / 2])
        cylinder(r=4, h=mav_motor_hole_h, center=true);

      // Supports
      translate([0, 0, support_h / 2])
        cube([mav_motor_hole_d, 8, support_h], center=true);
      translate([0, 0, support_h / 2])
        cube([8, mav_motor_hole_w, support_h], center=true);

      // Arm
      cube([mav_motor_hole_d + 10, arm_w, mav_motor_hole_h], center=true);
      translate([0, 0, -arm_w / 4])
        cube([mav_motor_hole_d + 10, arm_w, arm_w / 2], center=true);
    }

    // Mount holes
    translate([0, mav_motor_hole_w / 2, mav_motor_hole_h / 2])
      cylinder(r=M3_screw_w / 2, h=mav_motor_hole_h + 0.01, center=true);
    translate([0, -mav_motor_hole_w / 2, mav_motor_hole_h / 2])
      cylinder(r=M3_screw_w / 2, h=mav_motor_hole_h + 0.01, center=true);
    translate([mav_motor_hole_d / 2, 0, mav_motor_hole_h / 2])
      cylinder(r=M3_screw_w / 2, h=mav_motor_hole_h + 1, center=true);
    translate([-mav_motor_hole_d / 2, 0, mav_motor_hole_h / 2])
      cylinder(r=M3_screw_w / 2, h=mav_motor_hole_h + 1, center=true);

    // Mount hole conter sink
    translate([0, mav_motor_hole_w / 2, M3_caphead_h / 2])
      cylinder(r=M3_caphead_w / 2, h=M3_caphead_h + 0.1, center=true);
    translate([0, -mav_motor_hole_w / 2, M3_caphead_h / 2])
      cylinder(r=M3_caphead_w / 2, h=M3_caphead_h + 0.1, center=true);

    // Arm divit
    translate([0, 0, -arm_w / 2])
      rotate([90, 0, 90])
        cylinder(r=arm_w / 2, h=mav_motor_hole_d + 10 + 0.01, center=true);
    translate([0, 0, -5])
      cube([mav_motor_hole_w + 10, mav_motor_hole_d + 10, 2.0], center=true);
  }
}

module motor_mount_spacer(w, d, h, show_motor=1) {
  standoff_w = 8.0;
  standoff_h = 5.0;
  support_w = 3.5;
  support_h = 3.0;

  motor_hole_w = 19.0;
  motor_hole_d = 16.0;
  motor_hole_h = 6.0;

  difference() {
    // Body
    union() {
      rotate(90)
        cube([arm_w, motor_hole_d + 10, motor_hole_h], center=true);
    }

    // Arm divit
    translate([0, 0, arm_w / 2])
      rotate([90, 0, 90])
        cylinder(r=arm_w / 2, h=motor_hole_d + 10 + 0.01, center=true);

    // Mount holes
    translate([motor_hole_d / 2, 0, -motor_hole_h / 4])
      cylinder(r=M3_screw_w / 2, h=motor_hole_h / 2 + 1, center=true);
    translate([-motor_hole_d / 2, 0, -motor_hole_h / 4])
      cylinder(r=M3_screw_w / 2, h=motor_hole_h / 2 + 1, center=true);

    // Caphead counter sink
    translate([motor_hole_d / 2, 0, -M3_caphead_h])
      cylinder(r=M3_caphead_w / 2, h=M3_caphead_h, center=true);
    translate([-motor_hole_d / 2, 0, -M3_caphead_h])
      cylinder(r=M3_caphead_w / 2, h=M3_caphead_h, center=true);
  }
}

module mav_arm(arm_w, arm_l, show_motor_mount=1, show_motor=1) {
  // Motor mounts
  if (show_motor_mount) {
    y_offset = (arm_l - mav_motor_mount_d) / 2;
    translate([0, y_offset, arm_w])
      rotate(90)
        motor_mount(mav_motor_mount_w, mav_motor_mount_d, mav_motor_mount_h, show_motor=show_motor);
    translate([0, y_offset, 0])
      rotate(90)
        motor_mount_spacer(mav_motor_mount_w, mav_motor_mount_d, mav_motor_mount_h);
  }

  // Arm
  color([0.2, 0.2, 0.2])
  translate([0.0, 0.0, arm_w / 2.0]) {
    rotate([90.0, 0.0, 0.0]) {
      difference() {
        cylinder(r=arm_w / 2, h=arm_l, center=true);
        cylinder(r=arm_inner_w / 2, h=arm_l + 0.01, center=true);

        // cube([arm_w, arm_w, arm_l], center=true);
        // cube([arm_inner_w, arm_inner_w, arm_l + 0.01], center=true);
      }
    }
  }
}

module mav_arm_peg(peg_inner_w, peg_outer_w) {
  tol = 0.1;

  color([0, 0, 1]) {
    difference() {
      union() {
        // Center
        translate([0, 0, arm_w / 2])
          rotate([90, 0, 0])
            cylinder(r=arm_w / 2, h=peg_inner_w, center=true);
        translate([0, 0, arm_w / 2])
          rotate([90, 0, 90])
            cylinder(r=arm_w / 2, h=peg_inner_w, center=true);

        // Pegs
        translate([0, 0, arm_w / 2])
          rotate([90, 0, 0])
            cylinder(r=(arm_inner_w + tol) / 2, h=peg_outer_w, center=true);
        translate([0, 0, arm_w / 2])
          rotate([90, 0, 90])
            cylinder(r=(arm_inner_w + tol) / 2, h=peg_outer_w, center=true);
      }

      // Screw hole
      for (i = [1:4]) {
        rotate(90.0 * i) {
          translate([(peg_inner_w + mav_motor_mount_d) / 2 + mav_motor_hole_d / 2, 0.0, arm_w / 2])
            cylinder(r=M3_screw_w / 2, h=arm_w + 0.01, center=true);

          translate([(peg_inner_w + mav_motor_mount_d) / 2 - mav_motor_hole_d / 2, 0.0, arm_w / 2])
            cylinder(r=M3_screw_w / 2, h=arm_w + 0.01, center=true);
        }
      }
    }
  }
}

module mav_arm_supports(peg_inner_w, counter_sink_type=0) {
  support_l = arm_l * 0.48;
  support_h = 4.0;
  holes_outer = 50.0;

  difference() {
    union() {
      translate([0, 0, support_h / 2])
        cube([arm_w, support_l, support_h], center=true);
      translate([0, 0, support_h / 2])
        cube([support_l, arm_w, support_h], center=true);
      translate([0, 0, -arm_w / 4])
        cube([arm_w, support_l, arm_w / 2], center=true);
      translate([0, 0, -arm_w / 4])
        cube([support_l, arm_w, arm_w / 2], center=true);
    }

    for (i = [1:4]) {
      inner_x = (peg_inner_w + mav_motor_mount_d) / 2 - mav_motor_hole_d / 2;
      outer_x = (peg_inner_w + mav_motor_mount_d) / 2 + mav_motor_hole_d / 2;
      end_x = (peg_inner_w + mav_motor_mount_d) / 2 + mav_motor_hole_d * 1.5;

      rotate(90 * i) {
        // Inner motor hole
        translate([inner_x, 0.0, support_h / 2])
          cylinder(r=M3_screw_w / 2, h=support_h + 0.01, center=true);

        // Outer motor hole
        translate([outer_x, 0.0, support_h / 2])
          cylinder(r=M3_screw_w / 2, h=support_h + 0.01, center=true);

        // End hole
        translate([end_x, 0.0, support_h / 2])
          cylinder(r=M3_screw_w / 2, h=support_h + 0.01, center=true);

        if (counter_sink_type == 1) {
          // Hex counter sink
          translate([inner_x, 0.0, support_h - M3_nut_h / 2])
            cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
          // translate([outer_x, 0.0, support_h - M3_nut_h / 2])
          //   cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
          translate([end_x, 0.0, support_h - M3_nut_h / 2])
            cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);

        } else if (counter_sink_type == 0) {
          // Caphead counter sink
          translate([inner_x, 0.0, support_h - M3_caphead_h / 2])
            cylinder(r=M3_caphead_w / 2, h=M3_caphead_h + 0.01, center=true);
          // translate([outer_x, 0.0, support_h - M3_caphead_h / 2])
          //   cylinder(r=M3_caphead_w / 2, h=M3_caphead_h + 0.01, center=true);
          translate([end_x, 0.0, support_h - M3_nut_h / 2])
            cylinder(r=M3_caphead_w / 2, h=M3_caphead_h + 0.01, center=true);
        }
      }
    }

    rotate([90, 0, 0])
      translate([0, -arm_w / 2, 0])
        cylinder(r=(arm_w + 0.5) / 2, h=support_l + 0.1, center=true);
    rotate([90, 0, 90])
      translate([0, -arm_w / 2, 0])
        cylinder(r=(arm_w + 0.5) / 2, h=support_l + 0.1, center=true);
    translate([0, 0, -support_h / 2 - 3])
      cube([support_l + 1, support_l + 1, support_h], center=true);
  }
}

module mav_frame(frame_standoff_w, frame_standoff_h,
                 frame_support_w, frame_support_h,
                 peg_inner_w) {
  outer_w = (peg_inner_w + mav_motor_mount_d) / 2 + mav_motor_hole_d / 2;
  mount_w = sqrt(pow(outer_w, 2) + pow(outer_w, 2));
  frame_w = 90.0;
  frame_d = 40.0;

  difference() {
    union() {
      // Mount frame
      frame(mount_w, mount_w,
            M3_screw_w, M3_nut_w, M3_nut_h,
            frame_standoff_w, frame_support_h,
            frame_support_w, frame_support_h);

      // Mav frame
      frame(frame_d, frame_w,
            M3_screw_w, M3_nut_w, M3_nut_h,
            frame_standoff_w, frame_standoff_h,
            frame_support_w, frame_support_h);
    }

    // Mount screw hole
    translate([mount_w / 2, mount_w / 2, frame_support_h / 2])
      cylinder(r=M3_screw_w / 2, h=frame_support_h + 0.01, center=true);
    translate([-mount_w / 2, mount_w / 2, frame_support_h / 2])
      cylinder(r=M3_screw_w / 2, h=frame_support_h + 0.01, center=true);
    translate([mount_w / 2, -mount_w / 2, frame_support_h / 2])
      cylinder(r=M3_screw_w / 2, h=frame_support_h + 0.01, center=true);
    translate([-mount_w / 2, -mount_w / 2, frame_support_h / 2])
      cylinder(r=M3_screw_w / 2, h=frame_support_h + 0.01, center=true);

    // Screw counter sink
    translate([mount_w / 2, mount_w / 2, frame_support_h - M3_caphead_h / 2])
      cylinder(r=M3_caphead_w / 2, h=M3_caphead_h + 0.01, center=true);
    translate([-mount_w / 2, mount_w / 2, frame_support_h - M3_caphead_h / 2])
      cylinder(r=M3_caphead_w / 2, h=M3_caphead_h + 0.01, center=true);
    translate([mount_w / 2, -mount_w / 2, frame_support_h - M3_caphead_h / 2])
      cylinder(r=M3_caphead_w / 2, h=M3_caphead_h + 0.01, center=true);
    translate([-mount_w / 2, -mount_w / 2, frame_support_h - M3_caphead_h / 2])
      cylinder(r=M3_caphead_w / 2, h=M3_caphead_h + 0.01, center=true);

    // Screw counter sink
    translate([frame_d / 2, frame_w / 2, M3_nut_h / 2])
      cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
    translate([-frame_d / 2, frame_w / 2, M3_nut_h / 2])
      cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
    translate([frame_d / 2, -frame_w / 2, M3_nut_h / 2])
      cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
    translate([-frame_d / 2, -frame_w / 2, M3_nut_h / 2])
      cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
  }
}

module mav_payload_frame(payload_mount_w, payload_mount_d,
                         frame_standoff_w, frame_standoff_h,
                         frame_support_w, frame_support_h,
                         peg_inner_w, nut_counter_sink=1) {
  outer_w = (peg_inner_w + mav_motor_mount_d) / 2 + mav_motor_hole_d / 2;
  mount_w = sqrt(pow(outer_w, 2) + pow(outer_w, 2));

  difference() {
    union() {
      // Mount frame
      frame(mount_w, mount_w,
            M3_screw_w, M3_nut_w, M3_nut_h,
            frame_standoff_w, frame_support_h,
            frame_support_w, frame_support_h);

      // Mav frame
      frame(payload_mount_d, payload_mount_w,
            M3_screw_w, M3_nut_w, M3_nut_h,
            frame_standoff_w, frame_standoff_h,
            frame_support_w, frame_support_h);
    }

    // Screw holes
    translate([mount_w / 2, mount_w / 2, frame_support_h / 2])
      cylinder(r=M3_screw_w / 2, h=frame_support_h + 0.01, center=true);
    translate([-mount_w / 2, mount_w / 2, frame_support_h / 2])
      cylinder(r=M3_screw_w / 2, h=frame_support_h + 0.01, center=true);
    translate([mount_w / 2, -mount_w / 2, frame_support_h / 2])
      cylinder(r=M3_screw_w / 2, h=frame_support_h + 0.01, center=true);
    translate([-mount_w / 2, -mount_w / 2, frame_support_h / 2])
      cylinder(r=M3_screw_w / 2, h=frame_support_h + 0.01, center=true);

    if (nut_counter_sink) {
      // Nut counter sink
      translate([mount_w / 2, mount_w / 2, frame_support_h - M3_nut_h / 2])
        cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
      translate([-mount_w / 2, mount_w / 2, frame_support_h - M3_nut_h / 2])
        cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
      translate([mount_w / 2, -mount_w / 2, frame_support_h - M3_nut_h / 2])
        cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
      translate([-mount_w / 2, -mount_w / 2, frame_support_h - M3_nut_h / 2])
        cylinder(r=M3_nut_w / 2, h=M3_nut_h + 0.01, $fn=6, center=true);
    }
  }
}

module mav_assembly() {
  // Arm center peg
  rotate(45)
    mav_arm_peg(mav_peg_inner_w, mav_peg_outer_w);

  // Arms
  for (i = [0:4]) {
    rotate(45.0 + 90.0 * i)
      translate([0.0, arm_l / 2.0 + 15.0, 0.0])
        mav_arm(arm_w, arm_l, show_motor_mount=1, show_motor=0);
  }

  // Arm supports
  rotate(45) {
    // Top support
    translate([0, 0, arm_w + 0.01])
      mav_arm_supports(mav_peg_inner_w, 0);

    // Bottom support
    rotate([180, 0, 0])
      translate([0, 0, 0.01])
        mav_arm_supports(mav_peg_inner_w, 1);
  }

  // Frame
  // -- Top Frame
  translate([0.0, 0.0, 4 + arm_w + 0.1])
    rotate(90)
      mav_frame(mav_frame_standoff_w, mav_frame_standoff_h,
                mav_frame_support_w, mav_frame_support_h,
                mav_peg_inner_w);
  // -- Bottom Frame
  translate([0.0, 0.0, -4 + 0.1])
    rotate([180, 0, 0])
      mav_payload_frame(mav_payload_mount_w, mav_payload_mount_d,
                        mav_frame_standoff_w, mav_frame_support_h,
                        mav_frame_support_w, mav_frame_support_h,
                        mav_peg_inner_w);

  // Perception Module
  // rotate([180, 0, 0])
  //   translate([0, 0, 8])
  //     perception_module();
}

module tool_mav_arm_holes(dev_mode=0) {
  tol = 0.2;
  tool_w = arm_w * 2;
  tool_d = 80;
  tool_h = arm_w * 2;
  guide_w = 10.0;
  guide_h = 8.0;

  // Show arm
  if (dev_mode) {
    // translate([0, 0, mav_motor_mount_h])
    //   rotate(90)
    //     motor_mount(mav_motor_mount_w,
    //                 mav_motor_mount_d,
    //                 mav_motor_mount_h,
    //                 show_motor=0);

    translate([0, -28, mav_motor_mount_h])
      mav_arm_supports(mav_peg_inner_w);
  }

  difference() {
    if (dev_mode == 0) {
      union() {
        // Body
        translate([0, 15, 0])
        cube([tool_w, tool_d, tool_h], center=true);

        // Drill guide
        translate([0, mav_motor_hole_d / 2, (tool_h + guide_h) / 2])
          cylinder(r=guide_w / 2, h=guide_h, center=true);
        translate([0, -mav_motor_hole_d / 2, (tool_h + guide_h) / 2])
          cylinder(r=guide_w / 2, h=guide_h, center=true);
        translate([0, 48 / 2, , (tool_h + guide_h) / 2])
          cylinder(r=guide_w / 2, h=guide_h + 0.01, center=true);
      }
    }

    // Arm body hole
    translate([0, tool_d / 4 - tol, 0])
      rotate([90, 0, 0])
        cylinder(r=arm_w / 2 + tol, h=tool_d, center=true);

    // Arm screw holes
    translate([0, mav_motor_hole_d / 2, 0])
      cylinder(r=M3_screw_w / 2 + tol, h=tool_h + 0.01, center=true);
    translate([0, -mav_motor_hole_d / 2, 0])
      cylinder(r=M3_screw_w / 2 + tol, h=tool_h + 0.01, center=true);
    translate([0, 48 / 2, 0])
      cylinder(r=M3_screw_w / 2 + tol, h=tool_h + 0.01, center=true);

    // Drill guide hole
    translate([0, mav_motor_hole_d / 2, (tool_h + guide_h) / 2])
      cylinder(r=M3_screw_w / 2 + tol, h=guide_h + 0.1, center=true);
    translate([0, -mav_motor_hole_d / 2, (tool_h + guide_h) / 2])
      cylinder(r=M3_screw_w / 2 + tol, h=guide_h + 0.1, center=true);
    translate([0, 48 / 2, (tool_h + guide_h) / 2])
      cylinder(r=M3_screw_w / 2 + tol, h=guide_h + 0.1, center=true);
  }
}

// Main
// print();
mav_assembly();

// Develop
// translate([mav_peg_inner_w / 2 + mav_motor_mount_d / 2, 0, 15])
// motor_mount(mav_motor_mount_w, mav_motor_mount_d, mav_motor_mount_h, show_motor=1);
// motor_mount_spacer(mav_motor_mount_w, mav_motor_mount_d, mav_motor_mount_h, show_motor=0);
// mav_arm(arm_w, arm_l);
// mav_arm_peg(mav_peg_inner_w, mav_peg_outer_w);
// mav_arm_supports(mav_peg_inner_w);
// mav_frame(mav_frame_standoff_w, mav_frame_standoff_h,
//           mav_frame_support_w, mav_frame_support_h,
//           mav_peg_inner_w);
// mav_payload_frame(mav_payload_mount_w, mav_payload_mount_d,
//                   mav_frame_standoff_w, mav_frame_support_h,
//                   mav_frame_support_w, mav_frame_support_h,
//                   mav_peg_inner_w);

// Tools
// tool_mav_arm_holes();

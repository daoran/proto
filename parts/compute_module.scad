$fn = 50;
M2_screw_w = 2.7;
M2_nut_w = 4.9;
M2_nut_h = 2.0;

M3_screw_w = 3.2;
M3_nut_w = 6.5;
M3_nut_h = 2.5;

standoff_w = 9.0;
standoff_h = 4.0;

fcu_w = 60.0;
fcu_d = 40.0;
fcu_h = 25.0;
fcu_mount_w = 56.0;
fcu_mount_d = 36.0;
fcu_standoff_h = standoff_h + 3;
fcu_support_h = standoff_h - 2;

mav_mount_w = 50.0;
mav_mount_d = 37.5;

odroid_w = 59.0;
odroid_d = 83.0;
odroid_mount_w = 52.0;
odroid_mount_d = 76.0;

pololu_w = 48.3;
pololu_d = 15.2;
pololu_mount_w = 43.2;
pololu_mount_d = 10.2;

sbgc_w = 40.0;
sbgc_d = 25.0;
sbgc_mount_w = 23.1;
sbgc_mount_d = 19.0;

batt_w = 76.0;
batt_d = 35.0;
batt_h = 26.0;
batt_frame_w = batt_d + 8.99;
batt_frame_d = batt_w - 20;

roll_rod_mount_w = 35.0;
roll_rod_mount_d = 10.0;
roll_rod_mount_h = 10.0;

module propeller_nut_wrench() {
  // M5 Nut Wrench
  wrench_h = 20.0;
  lever_w = 10.0;
  nut_w = 10.0;
  nut_h = 8;

  difference() {
    cylinder(h=wrench_h, r=(nut_w * 1.5) / 2.0, center=true);

    translate([0.0, 0.0, wrench_h / 2.0 - nut_h / 2.0])
      cylinder(h=nut_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
  }

  translate([0.0, 0.0, - wrench_h / 2.0 + lever_w / 2.0])
    cube([nut_w * 0.8, nut_w * 6.0, lever_w], center=true);
}

module lipo_battery(c=[0.4, 0.4, 0.4]) {
  color(c)
    cube([batt_w, batt_d, batt_h], center=true);
}

module stack_spacer(h, nut_counter_sink=1) {
  screw_size = M3_screw_w;
  screw_hsize = screw_size / 2.0;
  nut_w = M3_nut_w;
  nut_h = M3_nut_h;
  tol = 0.2;

  translate([0.0, 0.0, h / 2.0]) {
    difference() {
      // Spacer body
      cylinder(h=h, r=screw_size + 1.0, center=true);

      // Thread hole
      cylinder(h=h + 0.01, r=screw_hsize + tol, center=true);
      // Nut counter sinks
      if (nut_counter_sink) {
        translate([0.0, 0.0, h / 2.0 - nut_h / 2.0]) {
          cylinder(h=nut_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
        }
        translate([0.0, 0.0, -h / 2.0 + nut_h / 2.0]) {
          cylinder(h=nut_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
        }
      }
    }
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

module frame(w, d, screw_w, nut_w, nut_h,
             standoff_w, standoff_h, support_h,
             nut_cst=0, nut_csb=0, disable=[]) {
  positions = [
    [w / 2, d / 2, standoff_h / 2],
    [w / 2, -d / 2, standoff_h / 2],
    [-w / 2, -d / 2, standoff_h / 2],
    [-w / 2, d / 2, standoff_h / 2]
  ];

  difference() {
    union() {
      // Standoffs
      supports = [0, 1, 2, 3];
      for (pos_idx = supports) {
        translate(positions[pos_idx]) {
          cylinder(r=standoff_w / 2.0, h=standoff_h, center=true);
        }
      }

      // Supports
      translate([0.0, d / 2, support_h / 2.0])
        cube([w, 3.0, support_h], center=true);
      translate([0.0, -d / 2, support_h / 2.0])
        cube([w, 3.0, support_h], center=true);
      translate([w / 2.0, 0.0, support_h / 2.0])
        cube([3.0, d, support_h], center=true);
      translate([-w / 2.0, 0.0, support_h / 2.0])
        cube([3.0, d, support_h], center=true);
    }

    // Mount Holes
    for (pos = positions) {
      translate(pos) {
        cylinder(r=screw_w / 2.0, h=standoff_h + 0.1, center=true);
      }
    }

    // Nut hole
    if (nut_cst || nut_csb) {
      for (pos = positions) {
        x = pos[0];
        y = pos[1];
        z = (nut_cst) ? standoff_h - nut_h / 2.0 : nut_h / 2.0;
        translate([x, y, z]) {
          cylinder(r=nut_w / 2.0, h=nut_h + 0.1, $fn=6, center=true);
        }
      }
    }

    // Disable standoffs
    for (pos_idx = disable) {
      x = positions[pos_idx][0];
      y = positions[pos_idx][1];
      z = positions[pos_idx][2] + (standoff_h - support_h) - 0.5;
      translate([x, y, z]) {
        cylinder(r=standoff_w / 2.0 + 0.01, h=support_h, center=true);
      }
    }
  }

  // Disable counter sinks
  for (pos_idx = disable) {
    x = positions[pos_idx][0];
    y = positions[pos_idx][1];
    z = (nut_cst) ? standoff_h - nut_h / 2.0 : nut_h / 2.0;
    translate([x, y, z]) {
      cylinder(r=nut_w / 2.0, h=nut_h, $fn=6, center=true);
    }
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

module landing_frame(w, d) {
  leg_w = 5.0;
  leg_l = 40.0;
  leg_h = 5.0;

  difference() {
    union() {
      // Frame mount
      frame(w, d,
            M2_screw_w, M2_nut_w, M2_nut_h,
            standoff_w, standoff_h, standoff_h);

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
            standoff_w, standoff_h, standoff_h, 0, 0);

      // Odroid frame
      frame(odroid_mount_w, odroid_mount_d,
            M3_screw_w, M3_nut_w, M3_nut_h,
            standoff_w, standoff_h, standoff_h, 0, 1);

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

module sbgc_frame(mount_w, mount_d, show_sbgc=1, show_pololu=1) {
  // Mount frame
  frame(mount_w, mount_d,
        M3_screw_w, M3_nut_w, M3_nut_h,
        standoff_w, standoff_h, standoff_h);

  // Pololu PSU
  if (show_pololu) {
    color([0.0, 1.0, 0.0])
      translate([-pololu_w / 2, -pololu_d / 2 + mount_d / 2 - 12, standoff_h + 3])
        import("../proto_parts/Pololu_U3V50X/Pololu-U3V50X.STL");
  }

  // Simple BGC
  if (show_sbgc) {
    color([1.0, 0.5, 0.0])
        rotate(90)
      translate([-sbgc_w / 2 - 10.0, -sbgc_d / 2, standoff_h + 4.5])
        import("../proto_parts/SimpleBGC_Tiny/Tiny_revC_PCB.stl");
  }

  // Pololu PSU frame
  translate([0.0, mount_d / 2 - 12, 0.0])
    frame(pololu_mount_w, pololu_mount_d,
          M2_screw_w, M2_nut_w, M2_nut_h,
          standoff_w - 3, standoff_h + 3, standoff_h,
          0, 1,
          [1, 3]);
  translate([mount_w / 2.0 - 2, mount_d / 2 - 12, standoff_h / 2.0])
    cube([3.0, 3.0, standoff_h], center=true);
  translate([-mount_w / 2.0 + 2, mount_d / 2 - 12, standoff_h / 2.0])
    cube([3.0, 3.0, standoff_h], center=true);

  // SBGC frame
  translate([0.0, -10.0, 0.0])
  rotate(90)
  frame(sbgc_mount_w, sbgc_mount_d,
        M2_screw_w, M2_nut_w, M2_nut_h,
        standoff_w - 3, standoff_h + 3, standoff_h,
        0, 1,
        [1, 3]);

  support_positions_x = [
    [19, 5.0 - 8, standoff_h / 2.0],
    [19, -5.0 - 12, standoff_h / 2.0],
    [-19, 5.0 - 8, standoff_h / 2.0],
    [-19, -5.0 - 12, standoff_h / 2.0]
  ];
  for (pos = support_positions_x) {
    translate(pos) cube([16.0, 3.0, standoff_h], center=true);
  }

  support_positions_y0 = [
    [5.0, mount_d / 2 - 18.0, standoff_h / 2.0],
    [-5.0, mount_d / 2 - 18.0, standoff_h / 2.0]
  ];
  for (pos = support_positions_y0) {
    translate(pos) cube([3.0, 38.0, standoff_h], center=true);
  }

  support_positions_y1 = [
    [5.0, -mount_d / 2 + 8.0, standoff_h / 2.0],
    [-5.0, -mount_d / 2 + 8.0, standoff_h / 2.0]
  ];
  for (pos = support_positions_y1) {
    translate(pos) cube([3.0, 18.0, standoff_h], center=true);
  }
}

module stereo_camera_frame(show_cameras=1) {
  mount_w = 18.5;
  baseline = 70.0;
  camera_w = 30.0;
  camera_mount_w = 24.5;

  // Mount frame
  frame(mount_w, mount_w,
        M2_screw_w, M2_nut_w, M2_nut_h,
        standoff_w - 3, standoff_h, standoff_h);

  // Mount frame
  frame(0.0, 30.0,
        M2_screw_w, M2_nut_w, M2_nut_h,
        standoff_w - 3, standoff_h, standoff_h,
        0, 1);

  // Board-cameras
  if (show_cameras) {
    translate([0.0, baseline / 2.0, standoff_h + 3.0]) board_camera();
    translate([0.0, -baseline / 2.0, standoff_h + 3.0]) board_camera();
  }

  // Board camera frames
  translate([0.0, baseline / 2.0, 0.0])
    frame(camera_mount_w, camera_mount_w,
          M2_screw_w, M2_nut_w, M2_nut_h,
          standoff_w - 3, standoff_h, standoff_h,
          0, 1);
  translate([0.0, -baseline / 2.0, 0.0])
    frame(camera_mount_w, camera_mount_w,
          M2_screw_w, M2_nut_w, M2_nut_h,
          standoff_w - 3, standoff_h, standoff_h,
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

// Gimbal Motor
module gimbal_motor(has_encoders=0) {
  motor_r = 35.0 / 2.0;
  motor_h = (has_encoders) ? 25.0 : 15.0;
  base_mount_d = (has_encoders) ? 20.0 : 29.0;

  difference() {
    // Main body
    color([0.2, 0.2, 0.2])
      cylinder(r=motor_r, h=motor_h);

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

module gimbal_imu() {
  w = 20.0;
  d = 17.5;
  h = 4.0;

  difference() {
    color([0, 1, 0]) translate([0, 0, h / 2]) cube([w, d, h], center=true);
  }
}

module mpu6050() {
  w = 21.0;
  d = 15.5;
  h = 4.0;
  mount_w = 15.5;
  mount_r = 3.0;

  rotate(90.0)
    difference() {
      color([0, 1, 0]) translate([0, 0, h / 2]) cube([w, d, h], center=true);

      translate([mount_w / 2, -d / 2 + 2.5, h / 2])
        cylinder(r=3.5 / 2, h=h + 0.1, center=true);
      translate([-mount_w / 2, -d / 2 + 2.5, h / 2])
        cylinder(r=3.5 / 2, h=h + 0.1, center=true);
    }
}

module gimbal_frame(mount_w, mount_d, show_motor=1, show_pitch_frame=1) {
  has_encoders = 0;
  motor_h = (has_encoders) ? 25.0 : 15.0;

  // Show motor
  if (show_motor) {
    translate([mount_w / 2 + standoff_h / 2, 0.0, 0.0])
      rotate([0.0, 90.0, 0.0])
        gimbal_motor(has_encoders);
  }

  // Show pitch frame
  if (show_pitch_frame) {
    rotate([-90, 0, 0])
      translate([70, 0, -50])
        roll_frame(has_encoders);
  }

  // Mount frame
  frame(mount_w, mount_d,
        M2_screw_w, M2_nut_w, M2_nut_h,
        standoff_w, standoff_h, standoff_h);

  // Motor frame mount
  frame(20, 35,
        M2_screw_w, M2_nut_w, M2_nut_h,
        standoff_w, standoff_h, standoff_h);
}

module pitch_frame(has_encoders=0, show_camera=1, show_motor=1, show_imu=1) {
  camera_mount_point = 58.0;
  motor_h = (has_encoders) ? 25.0 : 15.0;

  // Camera
  if (show_camera) {
    translate([-12.0, 0, camera_mount_point - standoff_h / 2])
      rotate([90, 0, 90])
        stereo_camera_frame();
  }

  // Motor
  if (show_motor) {
    translate([0.0, 0.0, -motor_h + 0.01])
        gimbal_motor(has_encoders);
  }

  // IMU
  if (show_imu) {
    translate([0, 0, camera_mount_point - standoff_h / 2])
      rotate([-90, 0, -90])
        translate([20, 0, 6])
          mpu6050();
  }

  // IMU mount frame
  imu_mount_w = 15.5;
  rotate([0, -90, 0])
    translate([camera_mount_point - standoff_h / 2, -25.0, (standoff_h + 2) / 2])
        rotate([180.0, 0, 0])
        frame(15.5, 0,
              M2_screw_w, M2_nut_w, M2_nut_h,
              standoff_w - 3, standoff_h + 5, standoff_h + 2,
              0, 1);
  translate([0.0, -22.0, camera_mount_point - standoff_h / 2])
    cube([6.0, 6.0, 9.0], center=true);


  // Motor mount frame
  motor_mount_w = 13.2;
  frame(motor_mount_w, motor_mount_w,
        M2_screw_w, M2_nut_w, M2_nut_h,
        standoff_w, standoff_h, standoff_h,
        0, 0);
  translate([motor_mount_w / 2 + 1.2, 0, standoff_h / 2])
    cube([6.0, 10.0, standoff_h], center=true);
  translate([-motor_mount_w / 2 - 1.2, 0, standoff_h / 2])
    cube([6.0, 10.0, standoff_h], center=true);
  translate([0, motor_mount_w / 2 + 1.2, standoff_h / 2])
    cube([10.0, 6.0, standoff_h], center=true);
  translate([0, -motor_mount_w / 2 - 1.2, standoff_h / 2])
    cube([10.0, 6.0, standoff_h], center=true);

  // Camera mount frame
  camera_mount_w = 18.5;
  translate([0, 0, camera_mount_point - standoff_h / 2])
    rotate([0, 90, 0])
      translate([0, 0, -standoff_h / 2.0 - 1])
        frame(camera_mount_w, camera_mount_w,
              M2_screw_w, M2_nut_w, M2_nut_h,
              standoff_w, standoff_h + 2, standoff_h + 2,
              1, 0);

  // Supports
  support_h = camera_mount_point * 2;
  pivot_h = 5.0;
  pivot_disc_h = 2.0;
  pivot_disc_r = 7.0 / 2.0;
  difference() {
    x = 0;
    y = 19.0;
    h = support_h - standoff_h;
    union() {
      // Main support body
      translate([-x, -y, h / 2])
        cube([standoff_h + 2, standoff_h + 1, h], center=true);
      translate([-x, y, h / 2])
        cube([standoff_h + 2, standoff_h + 1, h], center=true);
      translate([-x, 0, standoff_h / 2])
        cube([standoff_h + 2, 36 + 2, standoff_h], center=true);
      translate([-x, 0, standoff_h / 2 + h - standoff_h])
        cube([standoff_h + 2, 36 + 2, standoff_h], center=true);

      // Frame supports
      translate([-x, 0, support_h / 2 - standoff_h / 2])
        cube([standoff_h + 2, 36 + 2, standoff_h], center=true);
      translate([-x, 0, support_h / 3 + 1])
        cube([standoff_h + 2, 36 + 2, standoff_h], center=true);
      translate([-x, 0, support_h / 1.5 - 5])
        cube([standoff_h + 2, 36 + 2, standoff_h], center=true);
    }


    // Trim the base end
    translate([0, 0, standoff_h / 2])
      cube([6.0 + 0.1, 10.0 + 0.3, standoff_h + 0.1], center=true);
  }

  // Pivot
  translate([0, 0, support_h - standoff_h + pivot_h / 2])
    cylinder(r=5/2, h=pivot_h, center=true);

  // Pivot disc
  translate([0, 0, support_h - standoff_h - pivot_disc_h / 2])
    cylinder(r=pivot_disc_r, h=pivot_disc_h, center=true);
}

module roll_pivot_frame(has_encoders=0) {
  pivot_frame_h = 6.0;
  base_mount_d = (has_encoders) ? 14.2 : 20.5;
  mount_w = roll_rod_mount_w;
  mount_d = roll_rod_mount_d;
  mount_h = roll_rod_mount_h;

  difference() {
    // Body
    union() {
      // Main Body
      translate([0, 0, mount_h / 2])
        cylinder(r=20.0 / 2, h=mount_h, center=true);

      // Mount
      translate([-mount_w, mount_d, mount_h / 2])
        cylinder(r=standoff_w / 2, h=mount_h, center=true);
      translate([-mount_w, -mount_d, mount_h / 2])
        cylinder(r=standoff_w / 2, h=mount_h, center=true);

      // Join body and mount together
      translate([-mount_w / 2, 0, mount_h / 2])
        cube([mount_w, 20.0 + standoff_w, mount_h], center=true);
      translate([0, mount_d, mount_h / 2])
        cylinder(r=standoff_w / 2, h=mount_h, center=true);
      translate([0, -mount_d, mount_h / 2])
        cylinder(r=standoff_w / 2, h=mount_h, center=true);
    }

    // Mount holes
    translate([-mount_w, mount_d, mount_h / 2])
      cylinder(r=M3_screw_w / 2, h=mount_h + 0.1, center=true);
    translate([-mount_w, -mount_d, mount_h / 2])
      cylinder(r=M3_screw_w / 2, h=mount_h + 0.1, center=true);

    // Bearing counter-sink
    translate([0, 0, mount_h / 2 + 1])
      cylinder(r=14.2 / 2, h=mount_h, center=true);
    cylinder(r=5.2 / 2, h=mount_h, center=true);
  }
}

module roll_motor_frame(has_encoders=0) {
  camera_mount_point = 59.0;

  motor_r = 35.0 / 2.0;
  motor_h = (has_encoders) ? 25.0 : 15.0;
  motor_mount_w = 13.2;
  motor_mount_d = (has_encoders) ? 14.2 : 20.5;
  motor_mount_h = 8.0;

  mount_w = roll_rod_mount_w;
  mount_d = roll_rod_mount_d;
  mount_h = roll_rod_mount_h;

  // Roll frame
  difference() {
    union() {
      // Motor frame
      translate([motor_mount_d / 2, motor_mount_d / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([motor_mount_d / 2, -motor_mount_d / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([-motor_mount_d / 2, motor_mount_d / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([-motor_mount_d / 2, -motor_mount_d / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([0, motor_mount_d / 2, standoff_h / 2])
        cube([motor_mount_d, standoff_w, standoff_h], center=true);
      translate([0, -motor_mount_d / 2, standoff_h / 2])
        cube([motor_mount_d, standoff_w, standoff_h], center=true);
      translate([motor_mount_d / 2, 0.0, standoff_h / 2])
        cube([standoff_w, motor_mount_d, standoff_h], center=true);
      translate([-motor_mount_d / 2, 0.0, standoff_h / 2])
        cube([standoff_w, motor_mount_d, standoff_h], center=true);


      // Roll bar mount
      translate([-mount_w, mount_d, mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=mount_h, center=true);
      translate([-mount_w, -mount_d, mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=mount_h, center=true);

      translate([motor_mount_d / 2, mount_d, standoff_h / 2])
        cylinder(r=standoff_w / 2.0, h=standoff_h, center=true);
      translate([motor_mount_d / 2, -mount_d, standoff_h / 2])
        cylinder(r=standoff_w / 2.0, h=standoff_h, center=true);

      x = (motor_mount_d / 2) + (mount_w - (motor_mount_d / 2)) / 2 - motor_mount_d / 2;
      w = (mount_w + (motor_mount_d / 2));
      translate([-x, mount_d, standoff_h / 2])
        cube([w, standoff_w, standoff_h], center=true);
      translate([-x, -mount_d, standoff_h / 2])
        cube([w, standoff_w, standoff_h], center=true);
      translate([-mount_w, 0.0, standoff_h / 2])
        cube([standoff_w, mount_d * 2, standoff_h], center=true);
      translate([motor_mount_d / 2, 0.0, standoff_h / 2])
        cube([standoff_w, mount_d * 2, standoff_h], center=true);
    }


    // Roll bar holes
    translate([-mount_w, mount_d, mount_h / 2])
      cylinder(r=M3_screw_w / 2.0, h=mount_h + 0.1, center=true);
    translate([-mount_w, -mount_d, mount_h / 2])
      cylinder(r=M3_screw_w / 2.0, h=mount_h + 0.1, center=true);
    translate([-mount_w, mount_d, M3_nut_h / 2])
      cylinder(r=M3_nut_w / 2.0, h=M3_nut_h + 0.1, $fn=6, center=true);
    translate([-mount_w, -mount_d, M3_nut_h / 2])
      cylinder(r=M3_nut_w / 2.0, h=M3_nut_h + 0.1, $fn=6, center=true);

    // Motor mount holes
    translate([-motor_mount_d / 2, motor_mount_d / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
    translate([-motor_mount_d / 2, -motor_mount_d / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
    translate([motor_mount_d / 2, -motor_mount_d / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
    translate([motor_mount_d / 2, motor_mount_d / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
  }
}

module roll_bar_frame(has_encoders=0) {
  motor_mount_w = 13.2;
  motor_mount_d = (has_encoders) ? 14.2 : 20.5;
  motor_mount_h = 8.0;

  roll_bar_w = 50;
  min_y = -1 * roll_bar_w / 2.0;
  max_y = roll_bar_w / 2.0;

  support_h = 6;

  difference() {
    union() {
      // Motor frame
      translate([motor_mount_w / 2, motor_mount_w / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([motor_mount_w / 2, -motor_mount_w / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([-motor_mount_w / 2, motor_mount_w / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([-motor_mount_w / 2, -motor_mount_w / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([0, motor_mount_w / 2, standoff_h / 2])
        cube([motor_mount_w, standoff_w, standoff_h], center=true);
      translate([0, -motor_mount_w / 2, standoff_h / 2])
        cube([motor_mount_w, standoff_w, standoff_h], center=true);
      translate([motor_mount_w / 2, 0.0, standoff_h / 2])
        cube([standoff_w, motor_mount_w, standoff_h], center=true);
      translate([-motor_mount_w / 2, 0.0, standoff_h / 2])
        cube([standoff_w, motor_mount_w, standoff_h], center=true);

      // Rod mount body
      translate([motor_mount_d / 2, 0, (standoff_w - 1) / 2])
        rotate([90, 0, 0])
          cube([standoff_w, standoff_w -1, roll_bar_w], center=true);
      translate([-motor_mount_d / 2, 0, (standoff_w - 1) / 2])
        rotate([90, 0, 0])
          cube([standoff_w, standoff_w -1, roll_bar_w], center=true);
    }

    // Motor mount holes
    translate([-motor_mount_w / 2, motor_mount_w / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
    translate([-motor_mount_w / 2, -motor_mount_w / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
    translate([motor_mount_w / 2, -motor_mount_w / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
    translate([motor_mount_w / 2, motor_mount_w / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);

    // Rod mount holes
    translate([motor_mount_d / 2, 0, (standoff_w - 1) / 2])
      rotate([90, 0, 0])
        cylinder(r=M3_screw_w / 2, h=roll_bar_w + 30 + 0.1, center=true);
    translate([-motor_mount_d / 2, 0, (standoff_w - 1) / 2])
      rotate([90, 0, 0])
        cylinder(r=M3_screw_w / 2, h=roll_bar_w + 30 + 0.1, center=true);
  }
}

module roll_mount_frame(has_encoders=0) {
  motor_r = 35.0 / 2.0;
  motor_h = (has_encoders) ? 25.0 : 15.0;
  motor_mount_w = 13.2;
  motor_mount_d = (has_encoders) ? 14.2 : 20.5;
  motor_mount_h = 8.0;

  // Roll frame
  difference() {
    union() {
      // Motor frame
      translate([motor_mount_d / 2, motor_mount_d / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([motor_mount_d / 2, -motor_mount_d / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([-motor_mount_d / 2, motor_mount_d / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([-motor_mount_d / 2, -motor_mount_d / 2, motor_mount_h / 2])
        cylinder(r=standoff_w / 2.0, h=motor_mount_h, center=true);
      translate([0, motor_mount_d / 2, standoff_h / 2])
        cube([motor_mount_d, standoff_w, standoff_h], center=true);
      translate([0, -motor_mount_d / 2, standoff_h / 2])
        cube([motor_mount_d, standoff_w, standoff_h], center=true);
      translate([motor_mount_d / 2, 0.0, standoff_h / 2])
        cube([standoff_w, motor_mount_d, standoff_h], center=true);
      translate([-motor_mount_d / 2, 0.0, standoff_h / 2])
        cube([standoff_w, motor_mount_d, standoff_h], center=true);
    }

    // Motor mount holes
    translate([-motor_mount_d / 2, motor_mount_d / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
    translate([-motor_mount_d / 2, -motor_mount_d / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
    translate([motor_mount_d / 2, -motor_mount_d / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
    translate([motor_mount_d / 2, motor_mount_d / 2, motor_mount_h / 2])
      cylinder(r=M2_screw_w / 2.0, h=motor_mount_h + 0.01, center=true);
  }

}

module roll_frame(has_encoders=0, show_pivot_frame=1, show_pitch_frame=1) {
  camera_mount_point = 59.0;

  motor_r = 35.0 / 2.0;
  motor_h = (has_encoders) ? 25.0 : 15.0;
  motor_mount_w = 13.2;
  motor_mount_d = (has_encoders) ? 14.2 : 20.5;
  motor_mount_h = 8.0;

  mount_w = roll_rod_mount_w;
  mount_d = roll_rod_mount_d;
  mount_h = roll_rod_mount_h;

  // Pitch frame
  if (show_pitch_frame) {
    translate([0.0, 0.0,  motor_h + motor_mount_h])
      pitch_frame(has_encoders);
  }

  // Pivot frame
  if (show_pivot_frame) {
    z = camera_mount_point * 2 + motor_h + roll_rod_mount_h + 2;
    translate([0.0, 0.0, z]) {
      rotate([180.0, 0, 0])
      roll_pivot_frame(has_encoders);
    }
  }

  // Roll rods
  if (show_pivot_frame) {
    color([0.4, 0.4, 0.4]) {
      translate([-mount_w, -mount_d, 150/2])
        cylinder(r=3 / 2, h=150, center=true);
      translate([-mount_w, mount_d, 150/2])
        cylinder(r=3 / 2, h=150, center=true);
    }
  }

  // Roll mount frame
  if (show_pitch_frame) {
    z = motor_mount_h + camera_mount_point;
    translate([-mount_w, 0.0, z])
      rotate([90, 0, 90])
        translate([0.0, 0.0, -4])
          roll_bar_frame();

    translate([-mount_w, 0.0, z])
      rotate([0, 90, 0])
        translate([0, 0, -4 - motor_h])
          gimbal_motor(has_encoders);
  }

  // Roll motor frame
  roll_motor_frame();
}

module top_stack(show_fcu=0, show_battery=1) {
  // FCU Frame
  fcu_frame(show_fcu);

  // Battery Frame
  translate([0.0, 0.0, fcu_standoff_h + fcu_h + 0.5])
    battery_frame(batt_frame_w, batt_frame_d, show_battery);
}

module bottom_stack() {
  rotate([180, 0, 180])
    rotate(0)
    odroid_frame(mav_mount_w, mav_mount_d, 1);

  translate([0.0, 0.0, -25.0])
    rotate([180, 0, 180])
    sbgc_frame(odroid_mount_w, odroid_mount_d);

  translate([0.0, 0.0, -50.0])
    gimbal_frame(odroid_mount_w, odroid_mount_d);

  translate([0.0, 0.0, -80 - standoff_h - 1])
    landing_frame(odroid_mount_w, odroid_mount_d);
}

module print() {
  // nut_tool();

  // // Top stack
  // // -- FCU frame
  // translate([0, 120, 0])
  //   fcu_frame();
  // // -- Battery frame
  // translate([80.0, 120.0, 0.0])
  //   battery_frame();
  // // -- Spacers
  // translate([120.0, 120 + 10.0, 0.0])
  //   stack_spacer(fcu_h, nut_counter_sink=1);
  // translate([120.0, 120 + 0.0, 0.0])
  //   stack_spacer(fcu_h, nut_counter_sink=1);
  // translate([120.0, 120 + -10.0, 0.0])
  //   stack_spacer(fcu_h, nut_counter_sink=1);
  // translate([120.0, 120 + -20.0, 0.0])
  //   stack_spacer(fcu_h, nut_counter_sink=1);

  // // Bottom stack
  // // -- Odroid Frame
  // rotate(90)
  //   odroid_frame(mav_mount_w, mav_mount_d, 0);
  // // -- SBGC Frame
  // translate([80, 0, 0])
  //   sbgc_frame(odroid_mount_w, odroid_mount_d, 0, 0);
  // // -- Landing frame
  // translate([170, 0, 0])
  //   landing_frame(odroid_mount_w, odroid_mount_d);
  // // -- Odroid Spacers
  // translate([220.0, 0.0, 0.0])
  //   stack_spacer(fcu_h, nut_counter_sink=1);
  // translate([220.0, 10.0, 0.0])
  //   stack_spacer(fcu_h, nut_counter_sink=1);
  // translate([220.0, -10.0, 0.0])
  //   stack_spacer(fcu_h, nut_counter_sink=1);
  // translate([220.0, -20.0, 0.0])
  //   stack_spacer(fcu_h, nut_counter_sink=1);

  // // -- Roll Frame
  // translate([0.0, -100.0, 0.0])
  //   rotate(90) roll_motor_frame();
  // translate([25.0, -100 - 0.0, 0.0])
  //   stack_spacer(10.0, nut_counter_sink=1);
  // translate([25.0, -100 - 10.0, 0.0])
  //   stack_spacer(10.0, nut_counter_sink=1);
  // // -- Pivot frame
  // translate([60.0, -100.0, 0.0])
  //   rotate(90) roll_pivot_frame();
  // translate([90.0, -100.0 - 0.0, 0.0])
  //   stack_spacer(10.0, nut_counter_sink=1);
  // translate([90.0, -100.0 - 10.0, 0.0])
  //   stack_spacer(10.0, nut_counter_sink=1);
  // -- Mount frame
  // translate([120.0, -110.0, 0.0])
  //   roll_bar_frame();
  // translate([150.0, -100.0 - 0.0, 0.0])
  //   stack_spacer(10.0, nut_counter_sink=1);
  // translate([150.0, -100.0 - 10.0, 0.0])
  //   stack_spacer(10.0, nut_counter_sink=1);
}

// Main
// print();

// Assembly Development
// top_stack();
// bottom_stack();

// Component Development
// battery_frame(batt_frame_w, batt_frame_d);
// fcu_frame(show_fcu);
// stack_spacer(batt_h + 2, nut_counter_sink=1);
// odroid_frame(mav_mount_w, mav_mount_d, 0);
sbgc_frame(odroid_mount_w, odroid_mount_d, 0, 0);
// stereo_camera_frame();
// pitch_frame();
// roll_pivot_frame();
// roll_motor_frame();
// roll_bar_frame();
// roll_mount_frame();
// roll_frame();
// gimbal_frame(odroid_mount_w, odroid_mount_d);

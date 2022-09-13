$fn = 50;
screw_w = 2.7;
standoff_w = 6.0;
standoff_h = 5.0;

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
sbgc_mount_w = 23.5;
sbgc_mount_d = 19.0;

batt_w = 76.0;
batt_d = 35.0;
batt_h = 26.0;
batt_frame_w = batt_d + 4;
batt_frame_d = batt_w + 6;

module lipo_battery(c=[0.4, 0.4, 0.4]) {
  color(c)
    cube([batt_w, batt_d, batt_h], center=true);
}

module stack_spacer(h, nut_counter_sink=1) {
  screw_size = 3.0;
  screw_hsize = screw_size / 2.0;
  nut_w = 6.5;
  nut_h = 2.5;
  tol = 0.2;

  translate([0.0, 0.0, h / 2.0]) {
    difference() {
      // Spacer body
      cylinder(h=h, r=screw_hsize + 2.5, center=true);

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
  // tool_h = 25.0;
  // screw_size = 3.0;
  // screw_hsize = screw_size / 2.0;
  // nut_w = 6.5;

  // M2 size
  tool_h = 20.0;
  screw_size = 2.0;
  screw_hsize = screw_size / 2.0;
  nut_w = 4.7;

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

module frame(w, d, screw_w, standoff_w, standoff_h, support_h, nut_cst=0, nut_csb=0,
             disable=[]) {
  nut_h = 2.0;
  nut_w = 5.2;
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
      z = positions[pos_idx][2] + (standoff_h - support_h) + 0.5;
      translate([x, y, z]) {
        cylinder(r=standoff_w / 2.0 + 0.01, h=support_h, center=true);
      }
    }
  }
}

module battery_frame(mount_w, mount_d, show_battery=1) {
  // Lipo battery
  if (show_battery) {
    translate([0.0, 0.0, -batt_h / 2.0 - 0.1])
      rotate([0.0, 0.0, 90.0])
      lipo_battery();
  }

  // FPV frame mount
  rotate([180.0, 0.0, 0.0])
    translate([0.0, 0.0, -standoff_h])
      frame(mount_w, mount_d, 3.2, standoff_w, standoff_h, standoff_h);

  // Battery frame mount
  frame(batt_frame_w, batt_frame_d, screw_w, standoff_w, standoff_h, standoff_h);

  // Supports
  support_w = standoff_h;
  support_l = 3.0;
  positions1 = [
    [batt_frame_w / 2 + 2, batt_frame_d / 7, support_w / 2],
    [batt_frame_w / 2 + 2, -batt_frame_d / 7, support_w / 2],
    [-batt_frame_w / 2 - 2, batt_frame_d / 7, support_w / 2],
    [-batt_frame_w / 2 - 2, -batt_frame_d / 7, support_w / 2],
  ];
  for (pos = positions1) {
    translate(pos)
      cube([support_w, support_l, support_w], center=true);
  }
}

module landing_frame(w, d) {
  leg_w = 5.0;
  leg_l = 25.0;
  leg_h = 5.0;

  difference() {
    union() {
      // Frame mount
      frame(w, d, screw_w, standoff_w, standoff_h, standoff_h);

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
        cylinder(r=screw_w / 2, h=10.0, center=true);
        cylinder(r=4/2, h=standoff_h / 2 + 0.01, center=true);
      }
    }

  }
}

module odroid_frame(mount_w, mount_d, show_odroid=0) {
  // Top frame
  frame(mount_w, mount_d, 3.2, standoff_w, standoff_h, standoff_h);

  // Show Odroid XU4
  if (show_odroid) {
    color([1, 0.0, 0.0])
      rotate([90.0, 0.0, 0.0])
        translate([-odroid_d / 2, 5, -odroid_w / 2])
          import("../proto_parts/Odroid_XU4/Odroid_XU4.STL");
  }

  // Odroid frame
  rotate([180.0, 0.0, 90.0])
    translate([0.0, 0.0, -standoff_h])
      frame(odroid_mount_w, odroid_mount_d, screw_w, standoff_w, standoff_h, standoff_h, 0, 0);

  // Supports
  positions = [
    [odroid_mount_d / 4.0, odroid_mount_w / 2.0 - 3.7, standoff_h / 2.0],
    [-odroid_mount_d / 4.0, odroid_mount_w / 2.0 - 3.7, standoff_h / 2.0],
    [odroid_mount_d / 4.0, -odroid_mount_w / 2.0 + 3.7, standoff_h / 2.0],
    [-odroid_mount_d / 4.0, -odroid_mount_w / 2.0 + 3.7, standoff_h / 2.0]
  ];
  for (pos = positions) {
    translate(pos) {
      cube([3.0, 6.0, standoff_h], center=true);
    }
  }

  positions2 = [
    [odroid_mount_d / 2.0 - 6.0, odroid_mount_w / 4.0, standoff_h / 2.0],
    [odroid_mount_d / 2.0 - 6.0, -odroid_mount_w / 4.0, standoff_h / 2.0],
    [-odroid_mount_d / 2.0 + 6.0, odroid_mount_w / 4.0, standoff_h / 2.0],
    [-odroid_mount_d / 2.0 + 6.0, -odroid_mount_w / 4.0, standoff_h / 2.0]
  ];
  for (pos = positions2) {
    translate(pos) {
      cube([12.0, 3.0, standoff_h], center=true);
    }
  }
}

module sbgc_frame(mount_w, mount_d, show_sbgc=1, show_pololu=1) {
  // Mount frame
  frame(mount_w, mount_d, screw_w, standoff_w, standoff_h, standoff_h);

  // Pololu PSU
  if (show_pololu) {
    color([0.0, 1.0, 0.0])
      translate([-pololu_w / 2, -pololu_d / 2 + mount_d / 2 - 12, standoff_h + 3])
        import("../proto_parts/Pololu_U3V50X/Pololu-U3V50X.STL");
  }

  // Simple BGC
  if (show_sbgc) {
    color([1.0, 0.5, 0.0])
        rotate(180)
      translate([-sbgc_w / 2, -sbgc_d / 2, standoff_h + 4.5])
        import("../proto_parts/SimpleBGC_Tiny/Tiny_revC_PCB.stl");
  }

  // Pololu PSU frame
  translate([0.0, mount_d / 2 - 12, 0.0])
    frame(pololu_mount_w, pololu_mount_d,
          screw_w, standoff_w, standoff_h + 3, standoff_h,
          1, 0,
          [1, 3]);
  translate([mount_w / 2.0 - 2, mount_d / 2 - 12, standoff_h / 2.0])
    cube([3.0, 3.0, standoff_h], center=true);
  translate([-mount_w / 2.0 + 2, mount_d / 2 - 12, standoff_h / 2.0])
    cube([3.0, 3.0, standoff_h], center=true);

  // SBGC frame
  translate([0.0, 0.0, 0.0])
  frame(sbgc_mount_w, sbgc_mount_d, 
        screw_w, standoff_w, standoff_h + 3, standoff_h, 
        1, 0,
        [1, 3]);

  support_positions_x = [
    [19, 5.0, standoff_h / 2.0],
    [19, -5.0, standoff_h / 2.0],
    [-19, 5.0, standoff_h / 2.0],
    [-19, -5.0, standoff_h / 2.0]
  ];
  for (pos = support_positions_x) {
    translate(pos) cube([12.0, 3.0, standoff_h], center=true);
  }

  support_positions_y = [
    [7.0, mount_d / 2 - 15.0, standoff_h / 2.0],
    [-7.0, mount_d / 2 - 15.0, standoff_h / 2.0],
    [-7.0, -mount_d / 2 + 15.0, standoff_h / 2.0],
    [7.0, -mount_d / 2 + 15.0, standoff_h / 2.0]
  ];
  for (pos = support_positions_y) {
    translate(pos) cube([3.0, 30.0, standoff_h], center=true);
  }
}

module prototype_stack() {
  battery_frame(mav_mount_w, mav_mount_d);
  translate([0.0, 0.0, -batt_h])
    landing_frame(batt_frame_w, batt_frame_d);
}

module compute_stack() {
  rotate([180, 0, 0])
    odroid_frame(mav_mount_w, mav_mount_d, 1);

  translate([0.0, 0.0, -40.0])
    rotate(-90)
    sbgc_frame(odroid_mount_w, odroid_mount_d);

  translate([0.0, 0.0, -40 - standoff_h - 1])
    rotate(90)
    landing_frame(odroid_mount_w, odroid_mount_d);
}

// Main
// prototype_stack();
// compute_stack();
// stack_spacer(batt_h + 2, nut_counter_sink=1);
sbgc_frame(odroid_mount_w, odroid_mount_d);

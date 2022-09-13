$fn = 50;
mount_w = 50.0;
mount_d = 37.5;
screw_w = 2.7;
standoff_w = 6.0;
standoff_h = 5.0;
odroid_w = 59.0;
odroid_d = 83.0;
odroid_mount_w = 52.0;
odroid_mount_d = 76.0;

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

module frame(w, d, screw_w, standoff_w, standoff_h, support_h, nut_cst=0, nut_csb=0) {
  nut_h = 2.0;
  nut_w = 5.2;
  positions = [
    [w / 2, d / 2, standoff_h / 2],
    [-w / 2, d / 2, standoff_h / 2],
    [w / 2, -d / 2, standoff_h / 2],
    [-w / 2, -d / 2, standoff_h / 2]
  ];

  difference() {
    union() {
      // Standoffs
      for (pos = positions) {
        translate(pos) {
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
  }
}

module odroid_frame() {
  // FPV frame
  frame(mount_w, mount_d, screw_w, standoff_w, standoff_h + 10, standoff_h, 0, 1);

  // Odroid frame
  rotate([180.0, 0.0, 90.0])
    translate([0.0, 0.0, -standoff_h])
      frame(odroid_mount_w, odroid_mount_d, screw_w, standoff_w, standoff_h + 5, standoff_h, 0, 0);

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

module landing_frame() {
  leg_w = 5.0;
  leg_l = 20.0;
  leg_h = 10.0;

  translate([0.0, 0.0, -standoff_h]) {
    // FPV frame mount
    frame(mount_w, mount_d, screw_w, standoff_w, standoff_h, standoff_h, 1);
    positions = [
      [mount_w / 2, mount_d / 2, -leg_h / 2],
      [-mount_w / 2, mount_d / 2, -leg_h / 2],
      [mount_w / 2, -mount_d / 2, -leg_h / 2],
      [-mount_w / 2, -mount_d / 2, -leg_h / 2]
    ];

    // Standoffs
    for (pos = positions) {
      translate(pos) {
        cylinder(r=standoff_w / 2.0, h=leg_h, center=true);
      }
    }

    // Feet
    N = sqrt(leg_w * leg_w + leg_w * leg_w);
    feet_h = 5.0;
    feet_positions = [
      [mount_w / 2 + N, mount_d / 2 + N, -leg_h + feet_h / 2, -45.0],
      [-mount_w / 2 - N, mount_d / 2 + N, -leg_h + feet_h / 2, 45.0],
      [mount_w / 2 + N, -mount_d / 2 - N, -leg_h + feet_h / 2, 45.0],
      [-mount_w / 2 - N, -mount_d / 2 - N, -leg_h + feet_h / 2, -45.0]
    ];
    for (feet_pos = feet_positions) {
      x = feet_pos[0];
      y = feet_pos[1];
      z = feet_pos[2];
      rotz = feet_pos[3];
      translate([x, y, z]) {
        rotate([0.0, 0.0, rotz]) {
          cube([leg_w, leg_l, feet_h], center=true);
        }
      }
    }
  }
}

// Main
odroid_frame();
// landing_frame();

color([1, 0.0, 0.0])
  rotate([-90.0, 0.0, 0.0])
    translate([-odroid_d / 2, 2.2, -odroid_w / 2])
      import("../proto_parts/Odroid_XU4/Odroid_XU4.STL");

$fn = 50;
mount_w = 50.0;
mount_d = 37.5;
screw_w = 2.7;
standoff_w = 6.0;
standoff_h = 5.0;
odroid_mount_w = 52.0;
odroid_mount_d = 76.0;

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
  frame(mount_w, mount_d, screw_w, standoff_w, standoff_h + 5, standoff_h, 0, 1);

  // Odroid frame
  rotate([180.0, 0.0, 90.0])
    translate([0.0, 0.0, -standoff_h])
      frame(odroid_mount_w, odroid_mount_d, screw_w, standoff_w, standoff_h + 5, standoff_h, 1);

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

// Main
odroid_frame();

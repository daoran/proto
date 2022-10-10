module frame(w, d, screw_w, nut_w, nut_h,
             standoff_w, standoff_h, support_w, support_h,
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
        cube([w, support_w, support_h], center=true);
      translate([0.0, -d / 2, support_h / 2.0])
        cube([w, support_w, support_h], center=true);
      translate([w / 2.0, 0.0, support_h / 2.0])
        cube([support_w, d, support_h], center=true);
      translate([-w / 2.0, 0.0, support_h / 2.0])
        cube([support_w, d, support_h], center=true);
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
      z = positions[pos_idx][2] + support_h;
      translate([x, y, z]) {
        cylinder(r=standoff_w / 2.0 + 0.01, h=standoff_h, center=true);
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

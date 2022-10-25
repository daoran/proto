module spacer(spacer_w,
              spacer_h,
              hole_w,
              nts=1,
              tcs=1,
              bcs=1) {
  nut_w = M3_nut_w;
  nut_h = M3_nut_h;
  tol = 0.2;

  translate([0.0, 0.0, spacer_h / 2.0]) {
    difference() {
      // Spacer body
      cylinder(h=spacer_h, r=spacer_w / 2, center=true);

      // Thread hole
      cylinder(h=spacer_h + tol, r=hole_w / 2 + tol, center=true);

      // Nut counter sinks
      if (nts) {
        // Top counter sink
        if (tcs) {
          translate([0.0, 0.0, h / 2.0 - nut_h / 2.0]) {
            cylinder(h=nut_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
          }
        }

        // Bottom counter sink
        if (bcs) {
          translate([0.0, 0.0, -h / 2.0 + nut_h / 2.0]) {
            cylinder(h=nut_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
          }
        }
      }
    }
  }
}

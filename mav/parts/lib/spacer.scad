module spacer(spacer_w,
              spacer_h,
              hole_w,
              nut_w=0,
              nut_h=0,
              nts=1,
              tcs=1,
              bcs=1) {
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
          translate([0.0, 0.0, spacer_h / 2.0 - nut_h / 2.0]) {
            cylinder(h=nut_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
          }
        }

        // Bottom counter sink
        if (bcs) {
          translate([0.0, 0.0, -spacer_h / 2.0 + nut_h / 2.0]) {
            cylinder(h=nut_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
          }
        }
      }
    }
  }
}

module M2_spacer(spacer_w,
                 spacer_h,
                 nut_w=0,
                 nut_h=0,
                 nts=1,
                 tcs=1,
                 bcs=1) {
  M2_screw_w = 2.7;
  spacer(spacer_w, spacer_h, M2_screw_w,
         nut_w, nut_h,
         nts, tcs, bcs);
}

module M3_spacer(spacer_w,
                 spacer_h,
                 nut_w=0,
                 nut_h=0,
                 nts=1,
                 tcs=1,
                 bcs=1) {
  M3_screw_w = 3.2;
  spacer(spacer_w, spacer_h, M2_screw_w,
         nut_w, nut_h,
         nts, tcs, bcs);
}

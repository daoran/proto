$fn = 40;

stack_w = 120;
stack_d = 120;
stack_h = 3.0;

module standoff(h=11, r=2) {
  nipple_h = h * 0.8;

	// Standoff
  difference() {
    cylinder(h=h, r=r, center=true);

    translate([0.0, 0.0, h / 2.0 - nipple_h / 2.0])
      cylinder(h=nipple_h + 0.1, r=0.8, center=true);
  }

	// Fillet
	fillet_h_inc = 0.1;
	fillet_h = 2.0;
	for (fillet_height = [fillet_h_inc:fillet_h_inc:fillet_h]) {
		translate([0.0, 0.0, -h / 2.0 + fillet_height])
			cylinder(h=0.1, r=r * 1.5 - fillet_height);
	}
}

module nuc_standoffs(offset_x, offset_y, offset_z, w, d, h, r) {
  x = offset_x;
  y = offset_y;
  z = offset_z + h / 2;

  translate([x, y, z]) {
    translate([-d / 2.0, -w / 2.0, 0.0])
      standoff(h, r);

    translate([d / 2.0, -w / 2.0, 0.0])
      standoff(h, r);

    translate([d / 2.0, w / 2.0, 0.0])
      standoff(h, r);

    translate([-d / 2.0, w / 2.0, 0.0])
      standoff(h, r);
  };
}

module stack_module(w, d, h) {
  screw_size = 5;
  tol = 1.1;

  // Stack plate
  color([1.0, 1.0, 1.0]) {
    translate([0.0, 0.0, -h / 2.0]) {
      cube([w - 7, d - 7, h], center=true);
    }
  }

  // Stack through-holes
  support_h = 8;
  color([0.0, 0.0, 1.0]) {
    for (i = [0 : 90 : 360] ){
      rotate([0, 0, i]) {
        difference() {
          // Ring
          translate([w / 2.0, d / 2.0, 0.0]) {
            cylinder(support_h, r=(screw_size / 2.0) * 2.3, center=true);
          }

          // Hole
          translate([w / 2.0, d / 2.0, 0]) {
            cylinder(support_h + 0.01, r=(screw_size / 2.0) * tol, center=true);
          }
        }
      }
    }
  }

  // Stack frame support
  color([0.0, 0.0, 1.0]) {
    for (i = [0 : 90 : 360] ){
      rotate([0, 0, i]) {
        translate([0.0, d / 2.0, 0.0]) {
          // Support
          cube([w - 12 + 3, 7, 7],  center=true);
        }
      }
    }
  }
}

module nuc_stack(w, d, h) {
  // Parameters
  standoff_h = 10.0;
  standoff_r = 2.5;
  standoff_w = 95.0;
  standoff_d = 90.4;

  color([0.0, 1.0, 0.0])
    nuc_standoffs(0.0, 0.0, 0.0, standoff_w, standoff_d, standoff_h, standoff_r);

  stack_module(stack_w, stack_d, stack_h);
}

// Intel NUC
// #translate([0, 0, -17.5]) {
//   rotate([90.0, 0.0, 90.0]) {
//     import("/home/chutsu/components/NUC7i5DN.stl");
//   }
// }

nuc_stack(stack_w, stack_d, stack_h);

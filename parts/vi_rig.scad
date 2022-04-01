$fn = 40;

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

module nuc_standoffs(w, d, h, r) {
  x = 0.0;
  y = 0.0;
  z = h / 2;

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

module stack_module(w, h, plate=1, plate_holes=1) {
  tol = 1.1;
  screw_size = 5;
  screw_hsize = screw_size / 2.0;
  support_h = 5;

  col_start = 30.0;
  col_end = w - col_start;
  dcol = (col_end - col_start) / 3.0;

  // Stack plate
  if (plate) {
    difference() {
      // Plate
      translate([0.0, 0.0, 0.0]) {
        cube([w, w, h], center=true);
      }

      // Frame holes
      for (i = [0 : 90 : 360] ){
        rotate([0, 0, i]) {
          translate([w / 2.0, w / 2.0, 0.0]) {
            cylinder(support_h + 0.01, r=screw_hsize * tol, center=true);
          }
        }
      }
      // Plate holes
      if (plate_holes) {
        hole_r = 10;
        a = w / 5.0;
        b = sqrt(a * a + a * a);

        // Center hole
        cylinder(support_h + 0.01, r=hole_r, center=true);

        // Middle holes
        for (i = [0 : 90 : 360] ){
          rotate([0, 0, i]) {
            translate([a, 0.0, 0.0]) {
              cylinder(support_h + 0.01, r=hole_r, center=true);
            }
          }
        }

        // Corner holes
        for (i = [45 : 90 : 360] ){
          rotate([0, 0, i]) {
            translate([b, 0.0, 0.0]) {
              cylinder(support_h + 0.01, r=hole_r, center=true);
            }
          }
        }
      }
      // Side holes
      for (i = [0 : 90 : 360] ){
        rotate([0, 0, i]) {
          translate([(w / 2.0) - 12.0, 0.0, 0.0])
            cube([12, w * 0.4, support_h + 0.01], center=true);
          translate([(w / 2.0) - 12.0, (w * 0.4) / 2, 0.0])
            cylinder(support_h + 0.01, r=6, center=true);
          translate([(w / 2.0) - 12.0, -(w * 0.4) / 2, 0.0])
            cylinder(support_h + 0.01, r=6, center=true);
        }
      }
    }
  }

  // Stack through-holes
  for (i = [0 : 90 : 360] ){
    rotate([0, 0, i]) {
      difference() {
        // Ring
        translate([w / 2.0, w / 2.0, 0.0]) {
          cylinder(support_h, r=screw_hsize * 2.3, center=true);
        }

        // Hole
        translate([w / 2.0, w / 2.0, 0.0]) {
          cylinder(support_h + 0.01, r=screw_hsize * tol, center=true);
        }
      }
    }
  }

  // Stack frame support
  for (i = [0 : 90 : 360] ){
    rotate([0, 0, i]) {
      translate([0.0, w / 2.0, 0.0]) {
        // Support
        cube([w - 12 + 3, support_h, support_h],  center=true);
      }
    }
  }
}

module nuc_stack(w, h) {
  // Parameters
  pcb_w = 95.0;
  pcb_d = 90.4;
  standoff_h = 10.0;
  standoff_r = 2.5;

  translate([0.0, 0.0, h / 2])
    nuc_standoffs(pcb_w, pcb_d, standoff_h, standoff_r);

  stack_module(w, h);
}

module realsense_holder() {
  screw_size = 3.0;
  tol = 0.2;

  holes_w = 45.0;
  mount_w = 65.0;
  mount_d = 20.0;
  mount_h = 3.0;

  // rotate([0.0, 180.0, 0.0])
  //   translate([0.0, 0.0, mount_h / 2.0 + 25])
  //   import("/home/chutsu/components/Intel_RealSense_Depth_Camera_D435.stl");

  // Main back plate
  difference() {
    cube([mount_w, mount_d, mount_h],  center=true);

    translate([holes_w / 2.0, 0.0, 0.0]) {
      cylinder(h=mount_h + 0.01, r=(screw_size / 2.0) + tol, center=true);
    }
    translate([-holes_w / 2.0, 0.0, 0.0]) {
      cylinder(h=mount_h + 0.01, r=(screw_size / 2.0) + tol, center=true);
    }
  }

  // // Cable management
  // translate([0.0, 0.0, mount_h / 2.0 + 6.0 / 2.0])
  //   cube([30, 15.0, 6],  center=true);

  // Lip
  lip_w = mount_w; 
  lip_d = 3.0;
  lip_h = 25.0;
  // x = -mount_w / 2.0 + lip_w / 2.0;
  x = 0.0;
  y = -mount_d / 2.0 + lip_d / 2.0;
  z = mount_h / 2.0 + lip_h / 2.0;

  difference() {
    translate([x, y, z])
      cube([lip_w, 3.0, lip_h],  center=true);

    translate([lip_w / 2.0 - 10, -mount_d / 2.0 + lip_d / 2.0, lip_h - 8.0])
      rotate([90.0, 0.0, 0.0])
        cylinder(h=lip_d + 0.01, r=(screw_size / 2.0) + tol, center=true);
    translate([-lip_w / 2.0 + 10, -mount_d / 2.0 + lip_d / 2.0, lip_h - 8.0])
      rotate([90.0, 0.0, 0.0])
        cylinder(h=lip_d + 0.01, r=(screw_size / 2.0) + tol, center=true);
  }
}

module assembly() {
  stack_w = 140;
  stack_d = 140;
  stack_h = 3.0;

  // // Intel NUC
  // #translate([0, 0, -16.5]) {
  //   rotate([90.0, 0.0, 90.0]) {
  //     import("/home/chutsu/components/NUC7i5DN.stl");
  //   }
  // }

  // // Intel RealSense D435i
  // translate([stack_w + 10, 0, 0]) {
  //   rotate([90.0, 0.0, 90.0]) {
  //     #import("/home/chutsu/components/Intel_RealSense_Depth_Camera_D435.stl");
  //   }
  // }

  // translate([120.0, 0.0, 0.0])
  //   rotate([90.0, 0.0, -90.0])
  // #realsense_holder();

  // // Top stack
  // translate([0.0, 0.0, 30.0])
  //   stack_module(stack_w, stack_h, 1, 0);

  // Intel NUC stack module
  nuc_stack(stack_w, stack_h);

  // // Bottom stack
  // translate([0.0, 0.0, -30.0])
  //   stack_module(stack_w, stack_h, 1);
}

// Main
assembly();

// realsense_holder();

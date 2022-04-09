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

module pololu_standoffs() {
  screw_size = 3.0;
  screw_hsize = screw_size / 2.0;
  tol = 1.2;

  mount_w = 43.2;
  mount_d = 10.2;
  mount_h = 5.0;

  translate([mount_w / 2.0, mount_d / 2.0, mount_h / 2.0])
    standoff(h=mount_h, r=screw_hsize * tol);
  translate([-mount_w / 2.0, -mount_d / 2.0, mount_h / 2.0])
    standoff(h=mount_h, r=screw_hsize * tol);
}

module stack_module(w, h, plate=1, plate_holes=1, side_holes=1) {
  tol = 1.1;
  screw_size = 3;
  screw_hsize = screw_size / 2.0;
  support_h = h;

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
        hole_r = 6;
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
      if (side_holes) {
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
        cube([w - screw_size - 1, support_h, support_h],  center=true);
      }
    }
  }
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

module top_stack(w, h, batt_w, batt_d, batt_h) {
  screw_size = 3.0;
  screw_hsize = screw_size / 2.0;
  tol = 1.2;

  // Stack module plate
  difference() {
    stack_module(w, h, 1, 0, 0);

    // Battery strap holes
    translate([0.0, batt_w / 2.0 + 3, 0.0])
      cube([25, 3, h + 0.01], center=true);
    translate([0.0, -batt_w / 2.0 - 3, 0.0])
      cube([25, 3, h + 0.01], center=true);
    // Extra mount holes
    translate([(w - 30) / 2.0, 35, 0.0])
      cylinder(h + 0.01, r=screw_hsize * tol, center=true);
    translate([-(w - 30) / 2.0, 35, 0.0])
      cylinder(h + 0.01, r=screw_hsize * tol, center=true);
    translate([(w - 30) / 2.0, -35, 0.0])
      cylinder(h + 0.01, r=screw_hsize * tol, center=true);
    translate([-(w - 30) / 2.0, -35, 0.0])
      cylinder(h + 0.01, r=screw_hsize * tol, center=true);
  }

  // Rear cable rails
  difference() {
    translate([-w / 2.0 - 4.5, 0.0, 0.0])
      cube([8.0, 90.0, h], center=true);

    translate([-w / 2.0 - 4.5, 0.0, 0.0])
      cube([4.0, 85.0, h + 0.1], center=true);
  }

  // Voltage-regulator standoffs
  translate([0.0, 40.0, h / 2.0])
    pololu_standoffs();
}

module nuc_stack(w, h) {
  // Parameters
  pcb_w = 95.0;
  pcb_d = 90.4;
  standoff_h = 12.0;
  standoff_r = 2.5;

  // NUC standoffs
  translate([0.0, 0.0, h / 2])
    nuc_standoffs(pcb_w, pcb_d, standoff_h, standoff_r);

  // Front cable rails
  difference() {
    translate([w / 2.0 + 4.5, 0.0, 0.0])
      cube([8.0, 50.0, h], center=true);

    translate([w / 2.0 + 4.5, 0.0, 0.0])
      cube([4.0, 45.0, h + 0.1], center=true);
  }

  // Rear cable rails
  difference() {
    translate([-w / 2.0 - 4.5, 0.0, 0.0])
      cube([8.0, 90.0, h], center=true);

    translate([-w / 2.0 - 4.5, 0.0, 0.0])
      cube([4.0, 85.0, h + 0.1], center=true);
  }


  stack_module(w, h);
}

module realsense_stack(w, h) {
  screw_size = 3.0;
  screw_hsize = screw_size / 2.0;
  tol = 0.2;

  holes_w = 45.0;
  mount_w = 65.0;
  mount_d = 25.0;
  mount_h = 3.0;

  // Stack frame top
  translate([0.0, 0.0, 0.0])
    stack_module(w, h, plate=0, plate_holes=0, side_holes=0);

  // Stack frame bottom
  translate([0.0, 0.0, mount_d + h / 2.0])
    stack_module(w, h, plate=0, plate_holes=0, side_holes=0);

  // Corner spacers
  for (i = [0 : 90 : 360] ){
    rotate([0, 0, i]) {
      difference() {
        // Spacer
        translate([w / 2.0, w / 2.0, mount_d / 2.0]) {
          cylinder(mount_d, r=screw_hsize * 2.3, center=true);
        }

        // Hole
        translate([w / 2.0, w / 2.0, mount_d / 2.0]) {
          cylinder(mount_d + 0.01, r=screw_hsize + tol, center=true);
        }
      }
    }
  }

  // Cable management
  difference() {
    cube([w, 20.0, mount_h],  center=true);

    translate([0.0, 5.0, 0.0])
      cube([10.0, 2.0, mount_h + 0.01],  center=true);
    translate([0.0, -5.0, 0.0])
      cube([10.0, 2.0, mount_h + 0.01],  center=true);
    translate([w * 0.25, 5.0, 0.0])
      cube([10.0, 2.0, mount_h + 0.01],  center=true);
    translate([w * 0.25, -5.0, 0.0])
      cube([10.0, 2.0, mount_h + 0.01],  center=true);
    translate([-w * 0.25, 5.0, 0.0])
      cube([10.0, 2.0, mount_h + 0.01],  center=true);
    translate([-w * 0.25, -5.0, 0.0])
      cube([10.0, 2.0, mount_h + 0.01],  center=true);
  }

  // Realsense back plate
  translate([w / 2.0, 0.0, mount_d / 2.0 + h / 2.0])
  rotate([90.0, 0.0, 90.0])
  difference() {
    cube([mount_w, mount_d, mount_h],  center=true);

    translate([holes_w / 2.0, 0.0, 0.0]) {
      cylinder(h=mount_h + 0.01, r=(screw_size / 2.0) + tol, center=true);
    }
    translate([-holes_w / 2.0, 0.0, 0.0]) {
      cylinder(h=mount_h + 0.01, r=(screw_size / 2.0) + tol, center=true);
    }
  }
}

module assembly(show_sbc=1, show_cam=1, show_voltreg=1, show_batt=1) {
  stack_w = 110;
  stack_d = 110;
  stack_h = 3.0;

  batt_w = 48.0;
  batt_d = 140.0;
  batt_h = 27.0;

  pololu_w = 48.3;
  pololu_d = 15.2;

  // Intel NUC
  if (show_sbc) {
    color([0.0, 0.0, 1.0])
    translate([0, 0, -14]) {
      rotate([90.0, 0.0, 90.0]) {
        import("/home/chutsu/projects/proto_parts/Intel_NUC7i5DN/NUC7i5DN.stl");
      }
    }
  }

  // Intel RealSense D435i
  if (show_cam) {
    color([1.0, 0.0, 0.0])
    translate([stack_w / 2.0 + 26.5, 0, -16]) {
      rotate([90.0, 0.0, 90.0]) {
        import("/home/chutsu/projects/proto_parts/Intel_RealSense_D435i/Intel_RealSense_Depth_Camera_D435.stl");
      }
    }
  }

  // Pololu Voltage Regulator U3V50X
  if (show_voltreg) {
    color([0, 1, 0])
    translate([-pololu_w / 2.0, (-pololu_d / 2.0) + 40.0, 38.5]) {
      rotate([0.0, 0.0, 0.0]) {
        import("/home/chutsu/projects/proto_parts/Pololu_U3V50X/Pololu-U3V50X.stl");
      }
    }
  }

  // Battery
  if (show_batt) {
    color([1.0, 0.0, 1.0])
    translate([0.0, 0.0, 33.0])
      rotate([0.0, 0.0, 90.0])
        translate([0.0, 0.0, batt_h / 2.0 + stack_h / 2.0])
          cube([batt_w, batt_d, batt_h], center=true);
  }

  // Top stack
  translate([0.0, 0.0, 33.0])
    top_stack(stack_w, stack_h, batt_w, batt_d, batt_h);

  // Intel NUC stack module
  nuc_stack(stack_w, stack_h);

  // NUC stack spacers
  for (i = [0 : 90 : 360] ){
    rotate([0, 0, i]) {
      translate([stack_w / 2.0, stack_d / 2.0, stack_h / 2.0])
        stack_spacer(30.0);
    }
  }

  // RealSense holder
  translate([0.0, 0.0, -30.0])
    realsense_stack(stack_w, stack_h);
}

// Main
assembly(show_sbc=1, show_cam=1, show_voltreg=1, show_batt=1);
// realsense_stack();

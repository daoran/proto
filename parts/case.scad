$fn = 30;

/* PCB Parameters */
pcb_width = 56.0;
pcb_depth = 85.0;
pcb_padding = 12.0;

/* Standoff Parameters */
standoff_r = 2.5;
standoff_w = 49.0;
standoff_d = 58.0;
standoff_h = 4;
standoff_offset_x = 9.0;
standoff_offset_y = -0.5;
standoff_offset_z = 0.0;

/* Panel Parameters */
panel_thickness = 2.5;
panel_indent = 3;

/* Case Parameters */
case_thickness = 3;
case_width = pcb_width + pcb_padding;
case_depth = pcb_depth + pcb_padding + panel_indent + panel_thickness;
case_height = 30;

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

module front_panel(case_width,
                   case_height,
                   case_thickness,
                   thickness = 2.5,
                   indent = 7) {
  x = case_depth - panel_indent - panel_thickness;
  y = case_thickness / 2.0;
  z = case_thickness / 2.0;

  difference() {
    // Plate
    translate([x, y, z])
    cube([thickness,
          case_width - case_thickness,
          case_height]);
  }
}

module back_panel(case_width,
                  case_height,
                  case_thickness,
                  thickness = 2.5,
                  indent = 3) {
  x = panel_indent;
  y = case_thickness / 2.0;
  z = case_thickness / 2.0;

  difference() {
    // Plate
    translate([x, y, z])
    cube([thickness,
          case_width - case_thickness,
          case_height]);
  }
}

module fixtures() {
  y = case_thickness + case_thickness / 4;
  z = case_height / 2.0 + case_thickness / 2;
  difference() {
    translate([case_depth * 0.25, y, z])
      translate([0, 0, 2])
      cube([20, case_thickness / 2.0, 20],
            center=true);

    translate([case_depth * 0.25, y, z + 6])
      rotate([0.0, 90.0, 90.0])
      cylinder(h=case_thickness * 1.01, r=0.7, center=true);
  }

  difference() {
    translate([case_depth * 0.75, y, z])
      translate([0, 0, 2])
      cube([20, case_thickness / 2, 20],
            center=true);

    translate([case_depth * 0.75, y, z + 6])
      rotate([0.0, 90.0, 90.0])
      cylinder(h=case_thickness * 1.01, r=0.7, center=true);
  }
}

module mount_holes() {
  y = case_width - case_thickness / 2;
  z = case_height / 2.0 + case_thickness / 2;
  translate([case_depth * 0.25, y, z - 6])
    rotate([0.0, 90.0, 90.0])
    cylinder(h=case_thickness * 1.01, r=1.0, center=true);

  translate([case_depth * 0.75, y, z - 6])
    rotate([0.0, 90.0, 90.0])
    cylinder(h=case_thickness * 1.01, r=1.0, center=true);
}

module case() {
  difference() {
    color([0.2, 0.2, 0.2]) {
      // Base
      translate([0.0, case_thickness / 2, 0.0])
      cube([case_depth, case_width - case_thickness, case_thickness]);

      // Right wall
      translate([0.0, 0.0, case_thickness / 2])
      cube([case_depth, case_thickness, case_height / 2]);
      // -- Fillet
      translate([case_depth/ 2.0, case_thickness / 2.0, case_thickness / 2.0])
      rotate([0.0, 90.0, 0.0])
      cylinder(h=case_depth, r=case_thickness / 2, center=true);

      // Left wall
      translate([0.0, case_width - case_thickness, case_thickness / 2])
      cube([case_depth, case_thickness, case_height / 2]);
      // -- Fillet
      x = case_depth / 2.0;
      y = case_thickness / 2.0 + case_width - case_thickness;
      z = case_thickness / 2.0;
      translate([x, y, z])
      rotate([0.0, 90.0, 0.0])
      cylinder(h=case_depth, r=case_thickness / 2, center=true);
    }

    // Front panel groove
    front_panel(case_width,
                case_height,
                case_thickness,
                panel_thickness + 0.3,
                panel_indent);
    back_panel(case_width,
               case_height,
               case_thickness,
               panel_thickness + 0.3,
               panel_indent);

    // Mount holes
    mount_holes();
  }

  fixtures();
}

module base_shell() {
  translate([-case_depth / 2.0, -case_width / 2.0, 0.0])
  case();
}

module top_shell() {
  translate([-case_depth / 2.0, -case_width / 2.0, 0.0])
  translate([0.0, case_width, case_height + case_thickness])
    rotate([180.0, 0.0, 0.0])
      case();
}

module standoffs() {
  x = standoff_offset_x;
  y = standoff_offset_y;
  z = standoff_offset_z + standoff_h / 2 + case_thickness;

  translate([x, y, z]) {
    translate([-standoff_d / 2.0, -standoff_w / 2.0, 0.0])
      standoff(standoff_h, standoff_r);

    translate([standoff_d / 2.0, -standoff_w / 2.0, 0.0])
      standoff(standoff_h, standoff_r);

    translate([standoff_d / 2.0, standoff_w / 2.0, 0.0])
      standoff(standoff_h, standoff_r);

    translate([-standoff_d / 2.0, standoff_w / 2.0, 0.0])
      standoff(standoff_h, standoff_r);
  };
}

// Main
// -- Pi
translate([-7.0, 2.0, 4.5 + case_thickness])
	rotate([0.0, 0.0, 180.0])
  import("/home/chutsu/FreeCAD/components/pi3.stl");

// -- Base shell
color("blue") base_shell();

// -- Standoffs
standoffs();

// -- Top shell
translate([0, 0, 0.1]) color("red") top_shell();

// // -- Front panel
// translate([-case_depth / 2.0, -case_width / 2.0, 0.0])
// front_panel(case_width, case_height, case_thickness);
//
// // -- Back panel
// translate([-case_depth / 2.0, -case_width / 2.0, 0.0])
// back_panel(case_width, case_height, case_thickness);

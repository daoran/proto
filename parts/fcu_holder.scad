$fn = 50;

// Parameters
frame_w = 30.5;
frame_d = 30.5;
frame_h = 2;
screw_w = 1.95;
screw_h = 2.0;
fcu_w = 56.0;
fcu_d = 36.0;
fcu_h = 2.0;
support_w = 3.0;
raise_h = frame_h + 8.0;

module standoff(standoff_thickness, screw_w, screw_h, cst=false, center=true) {
  standoff_r = (screw_w / 2.0) + standoff_thickness;
  difference() {
    cylinder(h=screw_h, r=standoff_r, center=center);
    cylinder(h=screw_h * 1.01, r=screw_w / 2.0, center=center);

    if (cst) {
      nut_h = 2.0;
      nut_w = 4.32;
      translate([0.0, 0.0, screw_h / 2.0 - nut_h / 2.0]) {
        cylinder(h=nut_h + 0.1, r=nut_w / 2.0, $fn=6, center=true);
      }
    }
  }
}

module base(frame_w, frame_d, frame_h, screw_w, screw_h) {
  // Stand-offs
  standoff_thickness = 1.0;
  translate([frame_w / 2.0, frame_d / 2.0, screw_h / 2.0])
    standoff(standoff_thickness, screw_w, screw_h);
  translate([-frame_w / 2.0, frame_d / 2.0, screw_h / 2.0])
    standoff(standoff_thickness, screw_w, screw_h);
  translate([frame_w / 2.0, -frame_d / 2.0, screw_h / 2.0])
    standoff(standoff_thickness, screw_w, screw_h);
  translate([-frame_w / 2.0, -frame_d / 2.0, screw_h / 2.0])
    standoff(standoff_thickness, screw_w, screw_h);

  // Supports
  standoff_r = (screw_w / 2.0) + standoff_thickness;
  bar_w = frame_w - (2.0 * standoff_r) + 1.0;
  bar_d = frame_d - screw_w;

  translate([frame_w / 2.0, 0.0, frame_h / 2.0])
    cube([support_w, bar_d, frame_h], center=true);
  translate([-frame_w / 2.0, 0.0, frame_h / 2.0])
    cube([support_w, bar_d, frame_h], center=true);

  translate([-0.0, frame_d / 2.0, frame_h / 2.0])
    cube([bar_w, support_w, frame_h], center=true);
  translate([0.0, -frame_d / 2.0, frame_h / 2.0])
    cube([bar_w, support_w, frame_h], center=true);
}

module fcu_holder(frame_w, frame_d, frame_h, screw_w, screw_h) {
  // Stand-offs
  standoff_thickness = 1.5;
  translate([frame_w / 2.0, frame_d / 2.0, screw_h / 2.0])
    standoff(standoff_thickness, screw_w, screw_h, 1);
  translate([-frame_w / 2.0, frame_d / 2.0, screw_h / 2.0])
    standoff(standoff_thickness, screw_w, screw_h, 1);
  translate([frame_w / 2.0, -frame_d / 2.0, screw_h / 2.0])
    standoff(standoff_thickness, screw_w, screw_h, 1);
  translate([-frame_w / 2.0, -frame_d / 2.0, screw_h / 2.0])
    standoff(standoff_thickness, screw_w, screw_h, 1);

  // Supports
  standoff_r = (screw_w / 2.0) + standoff_thickness;
  // bar_w = frame_w - (2.0 * standoff_r) + 1.0;
  bar_w = frame_w;
  bar_d = frame_d - screw_w;

  translate([frame_w / 2.0, 0.0, frame_h / 2.0])
    cube([support_w, bar_d, frame_h], center=true);
  translate([-frame_w / 2.0, 0.0, frame_h / 2.0])
    cube([support_w, bar_d, frame_h], center=true);

  translate([-0.0, frame_d / 4.0, frame_h / 2.0])
    cube([bar_w, support_w, frame_h], center=true);
  translate([0.0, -frame_d / 4.0, frame_h / 2.0])
    cube([bar_w, support_w, frame_h], center=true);
}

// Base
base(frame_w, frame_d, frame_h, 3.1, frame_h);

// FCU Holder
fcu_holder(fcu_w, fcu_d, frame_h, screw_w, 10.0);

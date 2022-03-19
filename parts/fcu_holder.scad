$fn = 50;

/* Parameters */
frame_w = 30.5;
frame_d = 30.5;
frame_h = 2.0;

screw_w = 2.0;
screw_h = 2.5;

fcu_w = 60.0;
fcu_d = 40.0;
fcu_h = 2.0;

module standoff(standoff_thickness, screw_w, screw_h, center=true) {
  standoff_r = (screw_w / 2.0) + standoff_thickness;
  difference() {
    cylinder(h=screw_h, r=standoff_r, center=center);
    cylinder(h=screw_h * 1.01, r=screw_w / 2.0, center=center);
  }
}

module frame(frame_w, frame_d, frame_h, screw_w, screw_h) {
  // Stand-offs
  standoff_thickness = 1.0;
  translate([frame_w / 2.0, frame_d / 2.0, frame_h / 2.0])
    standoff(standoff_thickness, screw_w, frame_h);
  translate([-frame_w / 2.0, frame_d / 2.0, frame_h / 2.0])
    standoff(standoff_thickness, screw_w, frame_h);
  translate([frame_w / 2.0, -frame_d / 2.0, frame_h / 2.0])
    standoff(standoff_thickness, screw_w, frame_h);
  translate([-frame_w / 2.0, -frame_d / 2.0, frame_h / 2.0])
    standoff(standoff_thickness, screw_w, frame_h);

  // Supports
  standoff_r = (screw_w / 2.0) + standoff_thickness;
  bar_w = frame_w - (2.0 * standoff_r) + 0.5;
  translate([frame_w / 2.0, 0.0, frame_h / 2.0])
    cube([1.0, bar_d, frame_h], center=true);
  translate([-frame_w / 2.0, 0.0, frame_h / 2.0])
    cube([1.0, bar_d, frame_h], center=true);

  bar_d = frame_d - screw_w;
  translate([-0.0, frame_d / 2.0, frame_h / 2.0])
    cube([bar_w, 1.0, frame_h], center=true);
  translate([0.0, -frame_d / 2.0, frame_h / 2.0])
    cube([bar_w, 1.0, frame_h], center=true);
}

// Bottom-frame
frame(frame_w, frame_d, frame_h, screw_w, screw_h);

// Top-frame
translate([0.0, 0.0, 5.0])
  frame(fcu_w, fcu_d, fcu_h, screw_w, screw_h);

// Support that joins the bottom and top frames together
translate([fcu_w * 0.2, 0.0, 5.0 + frame_h / 2.0])
  cube([1.0, fcu_d, frame_h], center=true);

translate([-fcu_w * 0.2, 0.0, 5.0 + frame_h / 2.0])
  cube([1.0, fcu_d, frame_h], center=true);

translate([fcu_w * 0.2 - 0.5, frame_d / 2.0 - 0.5, frame_h])
  cube([1.0, 1.0, 5.0 - frame_h], center=false);

translate([-fcu_w * 0.2 - 0.5, frame_d / 2.0 - 0.5, frame_h])
  cube([1.0, 1.0, 5.0 - frame_h], center=false);

translate([fcu_w * 0.2 - 0.5, -frame_d / 2.0 - 0.5, frame_h])
  cube([1.0, 1.0, 5.0 - frame_h], center=false);

translate([-fcu_w * 0.2 - 0.5, -frame_d / 2.0 - 0.5, frame_h])
  cube([1.0, 1.0, 5.0 - frame_h], center=false);

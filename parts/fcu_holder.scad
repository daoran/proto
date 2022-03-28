$fn = 50;

/* Parameters */
frame_w = 30.5;
frame_d = 30.5;
frame_h = 1.5;
screw_w = 1.95;
screw_h = 2.0;
fcu_w = 56.0;
fcu_d = 36.0;
fcu_h = 2.0;
support_w = 3.0;
raise_h = frame_h + 8.0;

module standoff(standoff_thickness, screw_w, screw_h, center=true) {
  standoff_r = (screw_w / 2.0) + standoff_thickness;
  difference() {
    cylinder(h=screw_h, r=standoff_r, center=center);
    cylinder(h=screw_h * 1.01, r=screw_w / 2.0, center=center);
  }
}

module bottom_frame(frame_w, frame_d, frame_h, screw_w, screw_h) {
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

module top_frame(frame_w, frame_d, frame_h, screw_w, screw_h) {
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

  translate([-0.0, frame_d / 4.0, frame_h / 2.0])
    cube([bar_w, support_w, frame_h], center=true);
  translate([0.0, -frame_d / 4.0, frame_h / 2.0])
    cube([bar_w, support_w, frame_h], center=true);
}

// Bottom-frame
bottom_frame(frame_w, frame_d, frame_h, 3.1, frame_h);

// Top-frame
translate([0.0, 0.0, 0.0])
  top_frame(fcu_w, fcu_d, frame_h, screw_w, 10.0);

// // Top-frame supports
// translate([fcu_w * 0.15, 0.0, raise_h + frame_h / 2.0])
//   cube([support_w, fcu_d, frame_h], center=true);
//
// translate([-fcu_w * 0.15, 0.0, raise_h + frame_h / 2.0])
//   cube([support_w, fcu_d, frame_h], center=true);
//
// // Support that joins the bottom and top frames together
// translate([fcu_w * 0.15 - support_w / 2.0, frame_d / 2.0 - support_w / 2.0, frame_h])
//   cube([support_w, support_w, raise_h - frame_h], center=false);
//
// translate([-fcu_w * 0.15 - support_w / 2.0, frame_d / 2.0 - support_w / 2.0, frame_h])
//   cube([support_w, support_w, raise_h - frame_h], center=false);
//
// translate([fcu_w * 0.15 - support_w / 2.0, -frame_d / 2.0 - support_w / 2.0, frame_h])
//   cube([support_w, support_w, raise_h - frame_h], center=false);
//
// translate([-fcu_w * 0.15 - support_w / 2.0, -frame_d / 2.0 - support_w / 2.0, frame_h])
//   cube([support_w, support_w, raise_h - frame_h], center=false);

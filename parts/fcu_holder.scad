$fn = 50;

// Parameters
frame_w = 30.5;
frame_d = 30.5;
frame_h = 3;
screw_w = 1.95;
screw_h = 2.0;
fcu_w = 56.0;
fcu_d = 36.0;
fcu_h = 2.0;
support_w = 3.0;
raise_h = frame_h + 8.0;
batt_w = 76.0;
batt_d = 35.0;
batt_h = 26.0;

module lipo_battery(c=[0.4, 0.4, 0.4]) {
  color(c)
    cube([batt_w, batt_d, batt_h], center=true);
}

module standoff(standoff_thickness, screw_w, screw_h, cst=false, center=true) {
  standoff_r = (screw_w / 2.0) + standoff_thickness;
  difference() {
    cylinder(h=screw_h, r=standoff_r, center=center);
    cylinder(h=screw_h * 1.01, r=screw_w / 2.0, center=center);

    if (cst) {
      nut_h = 2.0;
      nut_w = 5.2;
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
  standoff_thickness = 2.2;
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

module battery_holder() {
  padding = 5.0;

  translate([0.0, 0.0, (batt_h) / 2.0])
    difference() {
      // Body
      cube([batt_w + padding, batt_d + padding, batt_h + padding], center=true);

      // Hull cut
      cube([batt_w + padding / 3.0, batt_d + padding / 3.0, batt_h], center=true);
      translate([0.0, 0.0, batt_h / 2.0])
        cube([batt_w + padding + 1.0, batt_d + padding + 1.0, batt_h], center=true);

      // End cut
      translate([(batt_w + padding / 2.0) / 2.0, 0.0, -(batt_h / 2.0 + padding / 2.0) / 2.0])
        cube([12.0, batt_d + padding + 0.01, batt_h - padding], center=true);

      // Battery strap cut
      translate([0.0, -14.0, -batt_h / 2.0 - padding / 4.0])
        cube([18, 3, padding / 2.0 + 0.01], center=true);
      translate([0.0, 14.0, -batt_h / 2.0 - padding / 4.0])
        cube([18, 3, padding / 2.0 + 0.01], center=true);
    }

  // Battery Stand-offs
  standoff_thickness = 2.5;
  standoff_r = (screw_w / 2.0) + standoff_thickness;
  standoff_h = 30.0;
  positions = [
    [fcu_d / 2.0, fcu_w / 2.0, standoff_h / 2.0 - 20.0],
    [fcu_d / 2.0, -fcu_w / 2.0, standoff_h / 2.0 - 20.0],
    [-fcu_d / 2.0, -fcu_w / 2.0, standoff_h / 2.0 - 20.0],
    [-fcu_d / 2.0, fcu_w / 2.0, standoff_h / 2.0 - 20.0]
  ];
  for (pos = positions) {
    translate(pos) {
      difference() {
        cylinder(h=standoff_h, r=standoff_r, center=true);

        translate([0.0, 0.0, standoff_h / 2.0 - 10])
          cylinder(h=20 + 0.01, r=4.3 / 2.0, center=true);

        cylinder(h=standoff_h + 0.01, r=screw_w / 2.0, center=true);
      }
    }
  }

  // Battery supports for standoffs
  translate([fcu_d / 2, fcu_w / 2 - 6.0, 10.0 / 2.0])
    cube([standoff_thickness, 7.0, 10.0], center=true);
  translate([fcu_d / 2, -fcu_w / 2 + 6.0, 10.0 / 2.0])
    cube([standoff_thickness, 7.0, 10.0], center=true);
  translate([-fcu_d / 2, fcu_w / 2 - 6.0, 10.0 / 2.0])
    cube([standoff_thickness, 7.0, 10.0], center=true);
  translate([-fcu_d / 2, -fcu_w / 2 + 6.0, 10.0 / 2.0])
    cube([standoff_thickness, 7.0, 10.0], center=true);
}

// // Base
// base(frame_w, frame_d, frame_h, 3.1, frame_h);

// // FCU Holder
// fcu_holder(fcu_w, fcu_d, frame_h, screw_w, 10.0);

// // Battery Holder
// translate([0.0, 0.0, 40.0])
//   rotate([0.0, 0.0, 90.0])
//     battery_holder();

// // Battery
// translate([0.0, 0.0, 60.0])
//   rotate([0.0, 0.0, 90.0])
//     lipo_battery();

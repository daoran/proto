$fn = 50;

m2_screw_hole = 3.1;
rs_mount_width = 45.0;
bar_width = 6.0;
bar_height = 8.0;
mount_width = 175.0;


module mocap_bar(length, bar_width) {
  tip_length = 12.0;

  difference() {
    union() {
      // Main bar
      cube([length, bar_width + 2.0, bar_width], center=true);

      // Tip
      translate([-(length / 2.0 - bar_width / 2.0), 0.0, tip_length / 2.0])
        cube([bar_width, bar_width + 2.0, tip_length], center=true);
    }

    // Spherical cut-out
    translate([-(length / 2.0 + 4), 0.0, tip_length / 1.6])
      sphere(r=5.0);

    // Screw hole
    translate([-(length / 2.0 + 2.5) + 5.0, 0.0, tip_length / 1.6])
      rotate([0.0, 90.0, 0.0])
        cylinder(h=10.0, r=m2_screw_hole / 2.0, center=true);
  }
}

module mocap_mount() {
  difference() {
    union() {
      // Main bar
      cube([bar_height, mount_width, bar_width], center=true);

      // Vertical bar
      bar_length = 40.0;
      translate([-bar_length / 2.0, 20.0 + bar_width / 2.0, 0.0])
        mocap_bar(bar_length, bar_width);
      translate([-bar_length / 2.0, -(20.0 + bar_width / 2.0), 0.0])
        mocap_bar(bar_length, bar_width);
    }

    // RealSense mount holes
    translate([0.0, rs_mount_width / 2.0, 0.0])
      cylinder(h=bar_width + 0.1, r=m2_screw_hole / 2.0, center=true);
    translate([0.0, -rs_mount_width / 2.0, 0.0])
      cylinder(h=bar_width + 0.1, r=m2_screw_hole / 2.0, center=true);

    // Mocap marker mount holes
    translate([0.0, 50.0, 0.0])
      rotate([0.0, 90.0, 0.0])
        cylinder(h=10.0, r=m2_screw_hole / 2.0, center=true);

    translate([0.0, 65.0, 0.0])
      rotate([0.0, 90.0, 0.0])
        cylinder(h=10.0, r=m2_screw_hole / 2.0, center=true);

    translate([0.0, 80.0, 0.0])
      rotate([0.0, 90.0, 0.0])
        cylinder(h=10.0, r=m2_screw_hole / 2.0, center=true);

    translate([0.0, -50.0, 0.0])
      rotate([0.0, 90.0, 0.0])
        cylinder(h=10.0, r=m2_screw_hole / 2.0, center=true);

    translate([0.0, -65.0, 0.0])
      rotate([0.0, 90.0, 0.0])
        cylinder(h=10.0, r=m2_screw_hole / 2.0, center=true);

    translate([0.0, -80.0, 0.0])
      rotate([0.0, 90.0, 0.0])
        cylinder(h=10.0, r=m2_screw_hole / 2.0, center=true);
  }
}

// translate([25.0, 0.0, 0.0])
//   rotate([-90.0, 180.0, -90.0])
//     import("../proto_parts/Intel_RealSense_D435i/D435i.STL");

// translate([-bar_width / 2.0, 0.0, 0.0])
//   rotate([0.0, 90.0, 0.0])
//     mocap_mount();

mocap_mount();
// mocap_bar(100.0, bar_width);

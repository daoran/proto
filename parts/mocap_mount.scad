$fn = 50;

m2_screw_hole = 3.3;
m3_screw_hole = 4.3;
rs_mount_width = 45.0;
bar_width = 6.0;
bar_height = 8.0;
mount_width = 122.0;
cage_width = 120.0;
cage_length = 60.0;
spacer_length = 26 - (bar_height + bar_width);

module spacer(length) {
  difference() {
    cylinder(h=length, r=m2_screw_hole / 2.0 + 1.5, center=true);
    cylinder(h=length + 0.01, r=m2_screw_hole / 2.0 , center=true);
  }
}

module mocap_cage(bar_width, cage_width, cage_length) {
  difference() {
    union() {
      // Width bars
      rotate([90.0, 0.0, 0.0])
        translate([cage_length / 2.0 - bar_width / 2.0, 0.0, 0.0])
          cube([bar_width, bar_width, cage_width], center=true);
      rotate([90.0, 0.0, 0.0])
        translate([-(cage_length / 2.0 - bar_width / 2.0), 0.0, 0.0])
          cube([bar_width, bar_width, cage_width], center=true);

      // Length bars
      rotate([0.0, 90.0, 0.0])
        translate([0.0, -(cage_width / 2.0 - bar_width / 2.0), 0.0])
          cube([bar_width, bar_width, cage_length], center=true);
      rotate([0.0, 90.0, 0.0])
        translate([0.0, (cage_width / 2.0 - bar_width / 2.0), 0.0])
          cube([bar_width, bar_width, cage_length], center=true);
    }

    // Mount holes
    translate([0.0, cage_width / 2.0 - bar_width / 2.0, 0.0])
      cylinder(h=bar_width + 0.1, r=m2_screw_hole / 2.0, center=true);
    translate([0.0, -(cage_width / 2.0 - bar_width / 2.0), 0.0])
      cylinder(h=bar_width + 0.1, r=m2_screw_hole / 2.0, center=true);

    // Screw holes
    // -- Width holes
    for (spacing = [0.0:0.25:0.3]) {
      translate([cage_length / 2.0 - bar_width / 2.0, cage_width * spacing, 0.0])
        cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
      translate([cage_length / 2.0 - bar_width / 2.0, -cage_width * spacing, 0.0])
        cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);

      translate([-(cage_length / 2.0 - bar_width / 2.0), cage_width * spacing, 0.0])
        cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
      translate([-(cage_length / 2.0 - bar_width / 2.0), -cage_width * spacing, 0.0])
        cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
    }
    for (spacing = [0.1:0.25:0.4]) {
      translate([cage_length / 2.0 - bar_width / 2.0, cage_width * spacing, 0.0])
        rotate([0.0, 90.0, 0.0])
          cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
      translate([cage_length / 2.0 - bar_width / 2.0, -cage_width * spacing, 0.0])
        rotate([0.0, 90.0, 0.0])
          cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);

      translate([-(cage_length / 2.0 - bar_width / 2.0), cage_width * spacing, 0.0])
        rotate([0.0, 90.0, 0.0])
          cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
      translate([-(cage_length / 2.0 - bar_width / 2.0), -cage_width * spacing, 0.0])
        rotate([0.0, 90.0, 0.0])
          cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
    }
    // -- Length holes
    for (spacing = [0.2:0.2:0.3]) {
      translate([cage_length * spacing, cage_width / 2.0 - bar_width / 2.0, 0.0])
        cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
      translate([-cage_length * spacing, cage_width / 2.0 - bar_width / 2.0, 0.0])
        cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);

      translate([cage_length * spacing, -(cage_width / 2.0 - bar_width / 2.0), 0.0])
        cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
      translate([-cage_length * spacing, -(cage_width / 2.0 - bar_width / 2.0), 0.0])
        cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
    }
    for (spacing = [0.1:0.2:0.4]) {
      translate([cage_length * spacing, cage_width / 2.0 - bar_width / 2.0, 0.0])
        rotate([90.0, 0.0, 0.0])
          cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
      translate([-cage_length * spacing, cage_width / 2.0 - bar_width / 2.0, 0.0])
        rotate([90.0, 0.0, 0.0])
          cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);

      translate([cage_length * spacing, -(cage_width / 2.0 - bar_width / 2.0), 0.0])
        rotate([90.0, 0.0, 0.0])
          cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
      translate([-cage_length * spacing, -(cage_width / 2.0 - bar_width / 2.0), 0.0])
        rotate([90.0, 0.0, 0.0])
          cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
    }
    // -- Corner holes
    corner_w = (cage_width/ 2.0 - bar_width / 2.0);
    corner_l = (cage_length / 2.0 - bar_width / 2.0);
    translate([corner_l, corner_w, 0.0])
      cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
    translate([corner_l, -corner_w, 0.0])
      cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
    translate([-corner_l, -corner_w, 0.0])
      cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
    translate([-corner_l, corner_w, 0.0])
      cylinder(h=bar_width + 0.1, r=m3_screw_hole / 2.0, center=true);
  }
}

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

      // RealSense mount holes standoff
      union() {
      translate([0.0, rs_mount_width / 2.0, -(10.0 - bar_width)])
        cylinder(h=(10.0 - bar_width), r=m2_screw_hole, center=true);
      translate([0.0, -rs_mount_width / 2.0, -(10.0 - bar_width)])
        cylinder(h=(10.0 - bar_width), r=m2_screw_hole, center=true);
      }

      // // Vertical bar
      // bar_length = 40.0;
      // translate([-bar_length / 2.0, 20.0 + bar_width / 2.0, 0.0])
      //   mocap_bar(bar_length, bar_width);
      // translate([-bar_length / 2.0, -(20.0 + bar_width / 2.0), 0.0])
      //   mocap_bar(bar_length, bar_width);
    }

    // RealSense mount holes
    translate([0.0, rs_mount_width / 2.0, 0.0])
      cylinder(h=20.0 + 0.1, r=m2_screw_hole / 2.0, center=true);
    translate([0.0, -rs_mount_width / 2.0, 0.0])
      cylinder(h=20.0 + 0.1, r=m2_screw_hole / 2.0, center=true);

    // Mocap marker mount holes
    translate([0.0, cage_width / 2.0 - bar_width / 2.0, 0.0])
      rotate([0.0, 90.0, 0.0])
        cylinder(h=10.0, r=m2_screw_hole / 2.0, center=true);

    translate([0.0, -(cage_width / 2.0 - bar_width / 2.0), 0.0])
      rotate([0.0, 90.0, 0.0])
        cylinder(h=10.0, r=m2_screw_hole / 2.0, center=true);
  }

}

// // Intel RealSense D435i
// translate([25.0, 0.0, 0.0])
//   rotate([-90.0, 180.0, -90.0])
//     import("../proto_parts/Intel_RealSense_D435i/D435i.STL");

// // Mount
// translate([-bar_width / 2.0, 0.0, 0.0])
//   rotate([0.0, 90.0, 0.0])
//     mocap_mount();

// Cage
translate([-bar_width / 2.0, 0.0, 19.0])
  mocap_cage(bar_width, cage_width, cage_length);

// Spacers
translate([-bar_width / 2.0, cage_width / 2.0 - bar_width / 2.0, 10.0])
  spacer(spacer_length);
translate([-bar_width / 2.0, -(cage_width / 2.0 - bar_width / 2.0), 10.0])
  spacer(spacer_length);

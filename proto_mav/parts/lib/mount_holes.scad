module mount_holes(mount_w, mount_d, hole_w, hole_h, fn=30, wtol=0.0, htol=0.01) {
  x = mount_w / 2;
  y = mount_d / 2;
  z = hole_h / 2;

  hole_positions = [[x, y, z], [-x, y, z], [x, -y, z], [-x, -y, z]];
  for (i = [0:3]) {
    translate(hole_positions[i])
      cylinder(r=(hole_w + wtol) / 2, h=hole_h + htol, $fn=fn, center=true);
  }
}

// difference() {
//   translate([0, 0, 2.5])
//     cube([10, 10, 5], center=true);
//   #mount_holes(8, 8, 2, 5, fn=6);
// }

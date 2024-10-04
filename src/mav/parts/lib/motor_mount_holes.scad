module motor_mount_holes(pos, mount_d, hole_w, hole_h, offset_rot_z=0, fn=30) {
  translate([pos[0], pos[1], pos[2]])
    for (i = [1 : 4]) {
      rotate([0.0, 0.0, i * 90.0 + 45.0 + offset_rot_z])
        translate([mount_d / 2.0, 0.0, -hole_h / 2])
          cylinder(r=hole_w / 2, h=hole_h + 0.001, center=true);
    }
}

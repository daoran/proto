module MPU6050() {
  w = 21.0;
  d = 15.5;
  h = 4.0;
  mount_w = 15.5;
  mount_r = 3.0;

  rotate(90.0)
    difference() {
      color([0, 1, 0]) translate([0, 0, h / 2]) cube([w, d, h], center=true);

      translate([mount_w / 2, -d / 2 + 2.5, h / 2])
        cylinder(r=3.5 / 2, h=h + 0.1, center=true);
      translate([-mount_w / 2, -d / 2 + 2.5, h / 2])
        cylinder(r=3.5 / 2, h=h + 0.1, center=true);
    }
}

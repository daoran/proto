$fn = 100;

// Gimbal Motor
module gimbal_motor() {
  difference() {
    // Main body
    color([0.2, 0.2, 0.2])
      cylinder(r=35.0, h=25.0);

    // Top mount holes
    for (i = [1 : 4]) {
      rotate([0.0, 0.0, i * 90.0])
        translate([19.0, 0.0, 25.0 - 2.0 + 0.01])
          cylinder(r=1.0, h=2.0, center=false);
    }

    // Base mount holes
    for (i = [1 : 4]) {
      rotate([0.0, 0.0, i * 90.0 + 45.0])
        translate([20.0, 0.0, -0.01])
          cylinder(r=1.0, h=2.0, center=false);
    }

    // Wire hole
    translate([-35.0 + 2.5, 0.0, 2.0 - 0.01])
      cube([5.0, 9.0, 4.0], center=true);
  }
}

gimbal_motor();

$fn = 30;

// M5 Nut Wrench
wrench_h = 20.0;
lever_w = 10.0;
nut_w = 8.76;
nut_h = 8;

difference() {
  cylinder(h=wrench_h, r=(nut_w * 1.5) / 2.0, center=true);

  translate([0.0, 0.0, wrench_h / 2.0 - nut_h / 2.0])
    cylinder(h=nut_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
}

translate([0.0, 0.0, - wrench_h / 2.0 + lever_w / 2.0])
cube([nut_w * 0.8, nut_w * 6.0, lever_w], center=true);

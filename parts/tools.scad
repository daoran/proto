$fn=50;

module propeller_nut_wrench() {
  // M5 Nut Wrench
  wrench_h = 20.0;
  lever_w = 10.0;
  nut_w = 10.0;
  nut_h = 8;

  difference() {
    cylinder(h=wrench_h, r=(nut_w * 1.5) / 2.0, center=true);

    translate([0.0, 0.0, wrench_h / 2.0 - nut_h / 2.0])
      cylinder(h=nut_h + 0.01, r=nut_w / 2.0, $fn=6, center=true);
  }

  translate([0.0, 0.0, - wrench_h / 2.0 + lever_w / 2.0])
    cube([nut_w * 0.8, nut_w * 6.0, lever_w], center=true);
}

module hex_key_tool() {
  tool_w = 5.0;
  tool_h = 5.0;
  bit_w = 2.5;
  bit_h = 3.0;

  // Tool bit
  translate([0, 0, tool_h + bit_h / 2])
    cylinder(r=bit_w / 2, h=bit_h, $fn=6, center=true);

  // Tool connector
  for (i = [0.0:0.1:0.8]) {
    conn_r = (tool_w / 2) * (1.0 - i);
    translate([0, 0, tool_h + (bit_h * i) / 2])
      cylinder(r=conn_r, h=(bit_h * i), center=true);
  }

  // Tool handle
  translate([0, 0, tool_h / 2])
    cylinder(r=tool_w / 2, h=tool_h, center=true);
}

// propeller_nut_wrench();
hex_key_tool();

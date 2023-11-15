PROP_SIZE = 12.7;  // 5 inch
PROP_HEIGHT = 5.0;

FRAME_SIZE = 23.0; // Wheel base
FRAME_SUPPORT_W = 5;
FRAME_SUPPORT_H = 2;

BODY_W = 8.0;
BODY_D = 8.0;
BODY_H = 5.0;

// Props
translate([FRAME_SIZE / 2, FRAME_SIZE / 2, PROP_HEIGHT / 2])
  cylinder(r=PROP_SIZE / 2, h=PROP_HEIGHT, center=true);
translate([-FRAME_SIZE / 2, FRAME_SIZE / 2, PROP_HEIGHT / 2])
  cylinder(r=PROP_SIZE / 2, h=PROP_HEIGHT, center=true);
translate([FRAME_SIZE / 2, -FRAME_SIZE / 2, PROP_HEIGHT / 2])
  cylinder(r=PROP_SIZE / 2, h=PROP_HEIGHT, center=true);
translate([-FRAME_SIZE / 2, -FRAME_SIZE / 2, PROP_HEIGHT / 2])
  cylinder(r=PROP_SIZE / 2, h=PROP_HEIGHT, center=true);

// Frame
translate([0, 0, FRAME_SUPPORT_H / 2])
  rotate(45)
    cube([FRAME_SIZE, FRAME_SUPPORT_W, FRAME_SUPPORT_H], center=true);
translate([0, 0, FRAME_SUPPORT_H / 2])
  rotate(-45)
    cube([FRAME_SIZE, FRAME_SUPPORT_W, FRAME_SUPPORT_H], center=true);

// Body
translate([0, 0, BODY_H / 2])
  cube([BODY_W, BODY_D, BODY_H], center=true);

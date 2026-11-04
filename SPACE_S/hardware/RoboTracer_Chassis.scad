// =====================================================================
//  RoboTracer chassis v2  (SPACE-S + 16-ch line sensor + 2 marker sensors)
//  Rule : whole robot must fit inside a circle of Ø200 mm
//  Wheel: Ø32 x 20 mm, two N20 gear motors, coaxial
//
//  Origin = centre of wheel axle,  +Y = forward,  units = mm
//  Open in OpenSCAD -> F6 -> Export STL (3D print) or set
//  EXPORT_2D = true -> F6 -> Export DXF/SVG (laser / CNC carbon)
// =====================================================================

/* [Export] */
EXPORT_2D   = false;   // true = flat outline only (DXF/SVG)
SHOW_PARTS  = true;    // ghost wheels, motors, battery, board, sensor
SHOW_CIRCLE = true;    // show the Ø200 rule circle

/* [Rule] */
RULE_D  = 200;         // max robot diameter
MARGIN  = 4;           // safety margin per side (assert uses this)

/* [Wheels & motors] */
WHEEL_D   = 32;
WHEEL_W   = 20;
TRACK     = 112;       // wheel centre to wheel centre  (outer width = TRACK + WHEEL_W)
WHEEL_GAP = 2;         // gap wheel inner face <-> plate edge
MOTOR_W   = 12;        // N20 body width  (along Y)
MOTOR_H   = 10;        // N20 body height (along Z)
MOTOR_L   = 34;        // motor + gearbox length (along X)
CLAMP_PITCH = 18;      // N20 bracket hole spacing (along Y)

/* [Plate] */
PLATE_T = 3;           // 3 mm PLA/PETG, or 2 mm carbon for EXPORT_2D
ARM_W   = 7;           // width of sensor boom arms
RIB_H   = 5;           // extra rib height on boom arms (3D print only, 0 = off)
CORNER_R = 3;          // outer fillet radius

/* [Line sensor bar 16 ch] */
SENSOR_W    = 124;     // PCB width
SENSOR_D    = 16;      // PCB depth (along Y)
SENSOR_Y    = 88;      // sensor row distance in front of axle
SENSOR_HOLE_PITCH = 114; // screw holes at both ends of the PCB
SENSOR_SLOT = 4;       // +/- adjust of SENSOR_Y using slots

/* [Marker sensors (left / right)] */
MARKER_X   = 62;       // lateral position from centre line
MARKER_Y   = 30;       // in front of axle
MARKER_PAD = [14, 18];
MARKER_SLOT = 4;       // +/- lateral adjust

/* [Battery 2S LiPo] */
BATT   = [70, 25, 14]; // X (across), Y, Z
BATT_Y = 24;           // battery centre in front of axle

/* [SPACE-S board] */
BOARD       = [56, 38];
BOARD_HOLES = [48, 30]; // hole rectangle (measure your board!)
BOARD_Y     = -41;      // board centre behind axle

/* [Tail skid] */
TAIL_Y = -64;          // tail skid screw position
TAIL_R = 5;

M3 = 3.3;
M2 = 2.3;
$fn = 48;

// ---------------------------------------------------------------------
BODY_HX  = TRACK/2 - WHEEL_W/2 - WHEEL_GAP;   // half width of plate = 44
PAD_X    = SENSOR_HOLE_PITCH/2;
BODY_FRONT = BATT_Y + BATT[1]/2 + 7;
BODY_REAR  = BOARD_Y - BOARD[1]/2 - 2;

// ===================== envelope check (Ø200 rule) ====================
function pts() = [
  for (s=[-1,1]) each [
    [s*SENSOR_W/2, SENSOR_Y + SENSOR_D/2 + SENSOR_SLOT],      // sensor front corners (slot fully forward)
    [s*(TRACK/2 + WHEEL_W/2),  WHEEL_D/2],                   // wheel corners
    [s*(TRACK/2 + WHEEL_W/2), -WHEEL_D/2],
    [s*(MARKER_X + MARKER_PAD[0]/2 + MARKER_SLOT), MARKER_Y + MARKER_PAD[1]/2],
    [s*BOARD[0]/2, BOARD_Y - BOARD[1]/2],
    [s*BATT[0]/2,  BATT_Y + BATT[1]/2]
  ],
  [0, TAIL_Y - TAIL_R]
];
// best circle centre on the Y axis between sensor front corner and tail tip
_sx = SENSOR_W/2;  _sy = SENSOR_Y + SENSOR_D/2 + SENSOR_SLOT;  _t = -(TAIL_Y - TAIL_R);
CY  = (_sx*_sx + _sy*_sy - _t*_t) / (2*(_sy + _t));
R_ENV = max([for (p = pts()) norm(p - [0, CY])]);
echo(str("Envelope Ø = ", 2*R_ENV, " mm  (rule ", RULE_D, ")   centre Y = ", CY));
assert(2*R_ENV <= RULE_D - 2*MARGIN, str("Robot too big: Ø", 2*R_ENV));

// ============================ 2D outline =============================
module rrect(size, r, center=true) {
  offset(r) square([size[0]-2*r, size[1]-2*r], center=center);
}
module bar(p1, p2, w) { hull() { translate(p1) circle(d=w); translate(p2) circle(d=w); } }
module slot(p, len, d, dir=[1,0]) {
  hull() { translate(p - dir*len) circle(d=d); translate(p + dir*len) circle(d=d); }
}

module outline_solid() {
  offset(r=-CORNER_R) offset(r=CORNER_R) union() {
    // centre body: motors + battery
    translate([-BODY_HX, -MOTOR_W/2 - 8]) square([2*BODY_HX, BODY_FRONT + MOTOR_W/2 + 8]);
    // rear body: SPACE-S board
    translate([-BOARD[0]/2 - 3, BODY_REAR]) square([BOARD[0] + 6, -BODY_REAR]);
    // tail skid tab
    hull() { translate([0, TAIL_Y]) circle(r=TAIL_R);
             translate([-12, BODY_REAR + 1]) square([24, 1]); }

    for (s=[-1,1]) {
      // sensor boom: triangulated (outer + inner arm) -> stiff, no flex like rods
      bar([s*(BODY_HX - 4), BODY_FRONT - 4], [s*PAD_X, SENSOR_Y], ARM_W);
      bar([s*12,            BODY_FRONT - 2], [s*PAD_X, SENSOR_Y], ARM_W);
      // sensor pad
      translate([s*PAD_X, SENSOR_Y]) rrect([12, 2*SENSOR_SLOT + 12], 3);
      // marker outrigger
      bar([s*(BODY_HX - 6), MARKER_Y], [s*MARKER_X, MARKER_Y], 10);
      translate([s*MARKER_X, MARKER_Y]) rrect([MARKER_PAD[0] + 2*MARKER_SLOT, MARKER_PAD[1]], 3);
    }
    // cross beam just behind the sensor bar
    bar([-PAD_X, SENSOR_Y - SENSOR_D/2 - 6], [PAD_X, SENSOR_Y - SENSOR_D/2 - 6], 6);
  }
}

module holes() {
  for (s=[-1,1]) {
    // sensor bar: slot along Y to tune sensor distance
    slot([s*PAD_X, SENSOR_Y], SENSOR_SLOT, M3, [0,1]);
    // marker sensor: slot along X to tune marker position
    slot([s*MARKER_X, MARKER_Y], MARKER_SLOT, M3, [1,0]);
    // N20 bracket holes at gearbox + zip-tie slots over motor can
    for (dy=[-1,1]) {
      translate([s*(BODY_HX - 6), dy*CLAMP_PITCH/2]) circle(d=M2);
      translate([s*(BODY_HX - 24), dy*(MOTOR_W/2 + 2.5)]) rrect([4, 2.2], 1);
    }
    // battery zip-tie slots
    for (dy=[-1,1]) translate([s*22, BATT_Y + dy*(BATT[1]/2 + 2.5)]) rrect([9, 3], 1.4);
    // board screws
    for (dy=[-1,1]) translate([s*BOARD_HOLES[0]/2, BOARD_Y + dy*BOARD_HOLES[1]/2]) circle(d=M3);
  }
  // wire pass between the two motors
  rrect([14, 10], 3);
  // lightening pockets
  translate([0, BATT_Y]) rrect([48, BATT[1] - 12], 3);
  translate([0, BOARD_Y]) rrect([BOARD_HOLES[0] - 12, BOARD_HOLES[1] - 10], 3);
  // tail skid screw
  translate([0, TAIL_Y]) circle(d=M3);
}

module plate2d() { difference() { outline_solid(); holes(); } }

// ============================ 3D =====================================
PLATE_Z = WHEEL_D/2 - MOTOR_H/2 - PLATE_T;   // plate sits under the motors (low CoG)

module plate3d() {
  translate([0,0,PLATE_Z]) {
    linear_extrude(PLATE_T) plate2d();
    if (RIB_H > 0) difference() {
      for (s=[-1,1]) linear_extrude(PLATE_T + RIB_H) {
        bar([s*(BODY_HX - 4), BODY_FRONT - 4], [s*(PAD_X - 6), SENSOR_Y - 8], 2);
        bar([s*12,            BODY_FRONT - 2], [s*(PAD_X - 6), SENSOR_Y - 8], 2);
      }
      translate([0,0,-1]) linear_extrude(PLATE_T + RIB_H + 2) holes();
    }
  }
}

module parts() {
  z = WHEEL_D/2;
  for (s=[-1,1]) {
    color("#222") translate([s*TRACK/2, 0, z]) rotate([0,90,0]) cylinder(d=WHEEL_D, h=WHEEL_W, center=true);
    color("#b87333") translate([s*(BODY_HX - MOTOR_L/2), 0, z]) cube([MOTOR_L, MOTOR_W, MOTOR_H], center=true);
    color("red") translate([s*MARKER_X, MARKER_Y, PLATE_Z - 2]) cube([10, 16, 2], center=true);
  }
  color("silver", 0.8) translate([0, BATT_Y, PLATE_Z + PLATE_T + BATT[2]/2]) cube(BATT, center=true);
  color("#1a1a1a") translate([0, BOARD_Y, PLATE_Z + PLATE_T + 6]) cube([BOARD[0], BOARD[1], 1.6], center=true);
  color("#111") translate([0, SENSOR_Y, PLATE_Z - 6]) cube([SENSOR_W, SENSOR_D, 1.6], center=true);
}

if (EXPORT_2D) {
  plate2d();
} else {
  color("white") plate3d();
  if (SHOW_PARTS) %parts();
  if (SHOW_CIRCLE) %translate([0, CY, 0]) difference() {
    cylinder(d=RULE_D, h=0.4, $fn=180);
    translate([0,0,-1]) cylinder(d=RULE_D - 1, h=2, $fn=180);
  }
}

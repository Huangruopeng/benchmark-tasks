// Real-trial fixture for "Load a flask onto a rotary evaporator: seat its socket on the tilted cone, clamp it,
// lower it into the bath": the 3D-printed parts. Millimetres throughout. PETG, 4 perimeters, 40 % infill.
// Render one part at a time, e.g.   openscad -D 'part="head"' -o head.stl rotavap_fixture.scad
//   part = "head" | "lever" | "carriage" | "foot" | "stop" | "keybox" | "keycap" | "assembly"
//
// Where the parts go (task frame: x forward from the line between the arm bases, y to the left of their midpoint,
// z above the arms' deck; the fixture's base board top, the "bench", is at z = -60):
//   cone base (wide end of the NS 24/40 ground zone), lift up:   x 362.9, y 0, z 149.0; lift down: z 54.0
//   joint axis: 30 degrees below horizontal, pointing at the arms (cone tip at x 328.3, z 129.0, lift up)
//   clamp hinge, lift up:                                         x 375.9, y 0, z 186.5 (axis along y)
//   lift travel 95 mm, top = the actuator's own end of stroke, bottom = the immersion-stop switch
//   cake pan (bath): centre x 291.9, y 0; rim 60 above the deck (120 above the bench)
//   key box: centre x 175, y 235, on the deck; DOWN key x 145, y 257; UP key x 145, y 213
//   cork ring: centre x 320, y 220, on the deck
//   column front face: x 600; drawer slide on it, centred on y 0; actuator beside it at y -45

part = "assembly";
$fn = 72;

// ---- the glass ---------------------------------------------------------------------------------------------------
TUBE_D = 22.0;      // Chemglass CG-100-06: 24/40 inner joint, full length, on 22 mm OD tubing
LINER = 0.5;        // a strip of 0.5 mm silicone or rubber sheet wrapped once round the tube where it is gripped
NECK_D = 30.0;      // the flask's socket, outside, just behind its bead (Chemglass CG-1506-05): measure and set
BEAD_D = 34.0;      // the bead round the flask's mouth: measure and set
BEAD_T = 4.0;       // the bead's length along the axis: measure and set
SNAP = 1.0;         // the fork's mouth is this much narrower than NECK_D: it snaps round the socket

// ---- the clamp: ring, 36 mm lever hinged on top, hook at the lever's tip (the sim's EASY CLIP) ------------------
// Joint frame for "head" and "lever": origin at the cone base on the axis; +X along the axis away from the flask
// (up the slope, toward the column); +Z square to the axis, upward; +Y to the left.
RING_R = 21;   RING_U0 = 6;   RING_U1 = 26;      // the ring round the tube, 6 to 26 mm behind the cone base
SLEEVE_R = 17; SLEEVE_U1 = 70;                    // the pinch sleeve that grips the tube
HINGE_U = 30;  HINGE_Z = 26;  PIN_D = 3.2;        // hinge axis; pin: an M3 x 25 screw with a locknut
LEVER_L = 36;                                     // hinge to tip
LEVER_W = 12;  LEVER_T = 5;                       // the lever bar: 12 wide, 5 thick; shut it lies z 23.5 to 28.5
LEVER_U0 = HINGE_U - LEVER_L - 3;                 // -9: the bar runs from 3 mm past the tip ...
LEVER_U1 = HINGE_U + 6;                           // ... to a 6 mm tail behind the hinge (it hits the open stop)
OPEN_DEG = 35;                                    // left open at 35 degrees
FORK_L = 4;                                       // the hook (a fork round the socket) is 4 mm thick along the axis
FORK_U1 = -(BEAD_T + 2);                          // its front face 2 mm behind the bead when the socket is home
FORK_WALL = 2.4;
R_IN = NECK_D / 2 + 0.25;                         // the fork's inside: 0.25 mm clear of the socket
GAP_HALF = asin((NECK_D - SNAP) / 2 / R_IN);      // half-angle of the fork's mouth, measured from straight down
SPRING_U = 22; SPRING_D = 5.5; SPRING_DEPTH = 8;  // pocket for the opening spring (5 mm OD x 16 mm compression)
SQ = 20.4;     SQ_Z = 32;                         // socket for the 20 x 20 mm aluminium square tube, above the axis
SOCK_U0 = 48;  SOCK_U1 = 80;

module xcyl(r, u0, u1) { translate([u0, 0, 0]) rotate([0, 90, 0]) cylinder(r = r, h = u1 - u0); }
module ycyl(r, y0, y1, u, z) { translate([u, y1, z]) rotate([90, 0, 0]) cylinder(r = r, h = y1 - y0); }

module head() {
    difference() {
        union() {
            xcyl(RING_R, RING_U0, RING_U1);                                   // the clamp ring
            xcyl(SLEEVE_R, RING_U1 - 0.01, SLEEVE_U1);                        // the pinch sleeve
            for (y0 = [6.5, -10.5]) hull() {                                  // the two hinge ears
                translate([HINGE_U - 4, y0, 15]) cube([8, 4, 1]);
                ycyl(4, y0, y0 + 4, HINGE_U, HINGE_Z);
            }
            translate([HINGE_U + 1, -6, 15]) cube([8, 12, 5.5]);              // open stop, top 20.5 above the axis
            translate([SOCK_U0, -13, 15]) cube([SOCK_U1 - SOCK_U0, 26, SQ_Z + SQ / 2 + 3 - 15]);  // tube socket
            for (u = [34, 56]) translate([u, -28, -8]) cube([10, 17, 16]);   // pinch bosses, right side
        }
        xcyl(TUBE_D / 2 + LINER, -1, SOCK_U1 + 1);                           // the bore
        translate([30, -30, -1]) cube([SOCK_U1 - 30 + 1, 30 - (TUBE_D / 2 + LINER) + 0.5, 2]);   // pinch slit
        for (u = [39, 61]) {                                                  // M4 pinch screws and nut traps
            translate([u, -21, -9]) cylinder(d = 4.5, h = 18);
            translate([u, -21, -8.01]) cylinder(d = 8.1, h = 3.5, $fn = 6);
        }
        ycyl(PIN_D / 2, -12, 12, HINGE_U, HINGE_Z);                          // hinge pin
        translate([SPRING_U, 0, RING_R - SPRING_DEPTH]) cylinder(d = SPRING_D, h = SPRING_DEPTH + 1);
        translate([SOCK_U0 - 1, -SQ / 2, SQ_Z - SQ / 2]) cube([SOCK_U1 - SOCK_U0 + 2, SQ, SQ]);
        for (u = [56, 72]) ycyl(1.7, 0, 14, u, SQ_Z);                       // M4 screws grip the square tube
    }
}

module fork() {   // a C round the socket just behind its bead, open downward: it snaps on only over the socket
    translate([FORK_U1 - FORK_L, 0, 0]) rotate([0, 90, 0])   // local +x = joint frame -Z (down)
        difference() {
            cylinder(r = R_IN + FORK_WALL, h = FORK_L);
            translate([0, 0, -1]) cylinder(r = R_IN, h = FORK_L + 2);
            translate([0, 0, -1]) linear_extrude(height = FORK_L + 2)
                polygon([[0, 0], [40 * cos(GAP_HALF), -40 * sin(GAP_HALF)], [40, -40 * sin(GAP_HALF)],
                         [40, 40 * sin(GAP_HALF)], [40 * cos(GAP_HALF), 40 * sin(GAP_HALF)]]);
        }
}

module lever() {   // drawn shut, in the joint frame
    difference() {
        union() {
            translate([LEVER_U0, -LEVER_W / 2, HINGE_Z - LEVER_T / 2]) cube([LEVER_U1 - LEVER_U0, LEVER_W, LEVER_T]);
            ycyl(4.5, -LEVER_W / 2, LEVER_W / 2, HINGE_U, HINGE_Z);                       // hinge eye
            translate([-8, -10, HINGE_Z + LEVER_T / 2 - 0.01]) cube([16, 20, 4]);         // thumb pad 16 x 20
            for (u = [-5, 0, 5])                                                           // its ribs
                translate([u - 0.75, -10, HINGE_Z + LEVER_T / 2 + 4 - 0.01]) cube([1.5, 20, 0.8]);
            translate([FORK_U1 - FORK_L, -LEVER_W / 2, R_IN + 1])                         // the hook ...
                cube([FORK_L, LEVER_W, HINGE_Z - R_IN - 1]);
            fork();                                                                         // ... and its fork
        }
        ycyl(PIN_D / 2, -LEVER_W, LEVER_W, HINGE_U, HINGE_Z);
    }
}

// ---- the carriage: on the drawer slide's moving member; holds the square tube's upper end, 30 degrees ---------
// Frame: origin where the square tube's centre line meets the plate's front face; +X toward the column (task +x),
// +Z up, Y as the task's y. Lift up, the origin is at task x 579, z 311 (with the slide's moving member at x 587).
// The square tube (20 x 20 x 1.5 mm aluminium, 210 mm long) runs 32 mm above the joint axis, parallel to it.
module carriage() {
    difference() {
        union() {
            translate([0, -35, -110]) cube([8, 70, 140]);                           // plate on the slide
            rotate([0, -30, 0]) translate([-45, -14, -14]) cube([41, 28, 28]);     // socket, along the axis
            translate([0, -55, -100]) cube([8, 20.01, 34]);                        // actuator ear
        }
        rotate([0, -30, 0]) translate([-46, -SQ / 2, -SQ / 2]) cube([41, SQ, SQ]);
        rotate([0, -30, 0]) for (s = [-36, -20]) translate([s, 0, 0]) cylinder(d = 3.4, h = 20);  // M4 grip screws
        hull() for (z = [-92, -74]) translate([-1, -45, z]) rotate([0, 90, 0]) cylinder(d = 4.3, h = 10);  // M4 pin slot
        // (the actuator's rod end, at task z 230 with the lift up: the slot sets the cone's height)
        for (y = [-12, 12], z = [-66, -26]) hull() {                                // M4 slots onto the slide
            translate([-1, y, z]) rotate([0, 90, 0]) cylinder(d = 4.5, h = 10);
            translate([-1, y, z + 12]) rotate([0, 90, 0]) cylinder(d = 4.5, h = 10);
        }
    }
}

// ---- the actuator's foot on the base board: its lower hole 33 mm above the board --------------------------------
module foot() {
    difference() {
        union() {
            cube([40, 30, 6]);                                                     // screwed to the board
            translate([17, 9, 0]) cube([6, 12, 41]);                               // the ear
        }
        translate([16, 15, 33]) rotate([0, 90, 0]) cylinder(d = 4.3, h = 8);      // M4 pin, along task x
        for (x = [6, 34]) translate([x, 22, -1]) cylinder(d = 4.5, h = 8);        // wood screws
    }
}

// ---- the immersion stop: holds the roller-lever switch (Pololu 1404) under the carriage, slotted for height ------
module stop() {
    difference() {
        union() {
            cube([4, 24, 60]);                                                     // on the column face
            translate([-26, 0, 0]) cube([30, 24, 4]);                              // shelf under the carriage
        }
        for (y = [6, 18]) hull() {                                                 // vertical slots, 20 mm of travel
            translate([-1, y, 14]) rotate([0, 90, 0]) cylinder(d = 4.5, h = 6);
            translate([-1, y, 34]) rotate([0, 90, 0]) cylinder(d = 4.5, h = 6);
        }
        for (x = [-20.75, -11.25]) translate([x, 12, -1]) cylinder(d = 2.2, h = 6);   // M2 switch screws, 9.5 apart
    }
}

// ---- the key box (the evaporator's panel): 140 x 120 x 45, two keys -----------------------------------------------
// Frame: origin at the box's centre on the deck; +x task x, +y task y. Keys at (-30, +22) DOWN and (-30, -22) UP.
BOX = [140, 120, 45];  WALL = 2.5;
KEYS = [[-30, 22], [-30, -22]];
module keybox() {
    difference() {
        translate([-BOX[0] / 2, -BOX[1] / 2, 0]) cube(BOX);
        translate([-BOX[0] / 2 + WALL, -BOX[1] / 2 + WALL, -1]) cube([BOX[0] - 2 * WALL, BOX[1] - 2 * WALL, BOX[2] - WALL + 1]);
        for (k = KEYS) translate([k[0], k[1], BOX[2] - WALL - 1]) cylinder(d = 7.4, h = WALL + 2);
        translate([20, -BOX[1] / 2 - 1, -1]) cube([14, WALL + 2, 10]);               // cable notch, toward the fixture
        translate([30, BOX[1] / 2 + 1, 20]) rotate([90, 0, 0]) cylinder(d = 11, h = WALL + 2);   // DC jack
    }
    for (k = KEYS) translate([k[0], k[1], BOX[2]]) difference() {                  // travel stops: 3.5 mm of travel
        cylinder(d = 14, h = 0.5);
        translate([0, 0, -1]) cylinder(d = 11, h = 3);
    }
    for (k = KEYS) translate([k[0] + 9, k[1] - 13, BOX[2] - WALL - 30]) difference() {   // switch tabs (Pololu 1402)
        cube([3, 26, 30.01]);
        for (y = [8.25, 17.75]) hull() {
            translate([-1, y, 6]) rotate([0, 90, 0]) cylinder(d = 2.2, h = 5);
            translate([-1, y, 14]) rotate([0, 90, 0]) cylinder(d = 2.2, h = 5);
        }
    }
}

// a key: 16 mm cap, 8 mm tall, standing 4 mm above the box (top 12 mm proud); 7 mm stem through the lid,
// held by an M3 x 8 screw and a 9 mm washer under the lid; a 10 mm OD spring in the cap's recess pushes it up
module keycap() {
    difference() {
        union() {
            cylinder(d = 16, h = 8);
            translate([0, 0, -6.5]) cylinder(d = 7, h = 6.51);
        }
        difference() {                                       // the spring's seat, round the stem
            translate([0, 0, -0.01]) cylinder(d = 10.5, h = 3);
            translate([0, 0, -1]) cylinder(d = 7, h = 5);
        }
        translate([0, 0, -7]) cylinder(d = 2.5, h = 7);    // M3 screw
    }
}

// ---- a look at the clamp: head, lever (shut), the glass cone and the flask's socket, home -----------------------
module assembly() {
    color("SteelBlue") head();
    color("RoyalBlue") lever();
    color("WhiteSmoke", 0.5) {                                                       // the cone and its tube
        translate([-40, 0, 0]) rotate([0, 90, 0]) cylinder(r1 = 10, r2 = 12, h = 40);
        xcyl(TUBE_D / 2, 0, 140);
    }
    color("LightCyan", 0.4) difference() {                                           // the socket, home
        union() { xcyl(NECK_D / 2, -60, -BEAD_T); xcyl(BEAD_D / 2, -BEAD_T, 0); }
        xcyl(12.5, -61, 1);
    }
}

if (part == "head") head();
else if (part == "lever") lever();
else if (part == "carriage") carriage();
else if (part == "foot") foot();
else if (part == "stop") stop();
else if (part == "keybox") keybox();
else if (part == "keycap") keycap();
else assembly();

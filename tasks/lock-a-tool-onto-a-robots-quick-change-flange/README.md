# Fit the blue suction-cup tool to a robot's manual quick-change flange and lock it on

An inert wrist stand holds the robot side of a manual quick-change tool changer low over the table, its face toward the
right arm, where a parked robot's wrist would hold it. Beside it, a tool rack holds the robot's gripper tool and a blue
suction-cup tool. One hand takes the suction tool by its handle and holds its plate on the flange's two locating dowels,
its three lugs between the three claws of the flange's locking ring. The other presses the ring's lever down, turning the
ring until its claws have taken the plate, drawn it onto the flange and locked it. Then both hands let go, the tool
hanging on the flange, and both arms go back to where they started.

Files:

- [`lock_a_tool_onto_a_robots_quick_change_flange.py`](lock_a_tool_onto_a_robots_quick_change_flange.py): the task for
  the `yam_arena` embodiment (scene, scoring and the scripted demo). It imports [`bodykit.py`](bodykit.py) from the same
  folder.
- [`drawings/`](drawings): six dimensioned drawings (SVG) of the stand, the changer, the cam, the tools and the rack.
- [`cad/`](cad): STL files of every printed part, each in its print orientation.

What it tests: precise alignment held against gravity while the other hand works a lock. The plate must sit on two
6 mm dowels, within about 1.1 mm and half a degree, with its three lugs between the claws, until the turning ring takes
it. The task also tests picking the right tool and finishing with both hands free. Each tool carries a square handle on
top, an aid for the parallel grippers; it does not change the alignment the task is about.

Why two hands: nothing holds the tool on the dowels' tips until the ring has taken it, and the ring takes only a tool
that is already seated on the dowels while the ring is still open. With one hand (the right taking the tool and holding
it on the dowels' tips as in the demo, the left never moving) the demo scored 0.25: the tool sat seated with the ring
open for 5.5 s, and fell 54 mm as soon as the hand let go. (A real plate could also be pushed fully onto its dowels,
where it would hang; see "How it is modelled".)

## Why it matters

Robot cells change end effectors on tool changers, and the people who build, commission and re-tool robot cells fit
those tools by hand on manual changers. This is hands-on robot building work, so the task is in the Impact on AI timelines
domain. On a manual changer, the tool's plate is held up square on the robot side's locating pins while a lever or
locking ring is worked. Let go too early and the tool drops, damaging it or the fixture. Manual changers of this kind are
standard cobot hardware:

- ATI's MC-50 manual tool changer has "(2) Alignment Pins" (spare part 3700-20-11539, "6 mm Diamond-head Alignment Pin")
  and a hand lever: "As the user closes the lever, the spring plunger moves a tapered cam that pushes the ball bearings
  against the Tool's bearing race". Its master plate mounts on "a robot arm that has an ISO 9409-1-50-4-M6 mounting
  pattern" ([manual](https://www.ati-ia.com/app_content/Documents/9620-20-B-MC-50.pdf)).
- SCHUNK's CMS 050 cobot changer: "Actuation: Manual via locking lever", on ISO 9409-1-50-4-M6
  ([SCHUNK](https://schunk.com/us/en/automation-technology/tool-changer/cms/cms-050-k/p/000000000001545289)).

## The real trial's hardware

The real trial needs no robot arm and no machined steel. The changer is printed in PETG round bought precision parts, and
an inert printed stand stands in for the robot's wrist. Every part has a drawing, an STL file or a part number below.

### No affordable catalogue changer fits

We looked for a real manual changer with both two-dowel alignment and a hand-worked lock for a few hundred dollars.
Prices were read from the vendors' pages on 2026-11-16 (US Pacific).

| Product | Two dowels | Hand-worked lock | Price |
|---|---|---|---|
| ATI MC-50 (9122-050M-000 + 9122-050T-000) | yes | yes, cam lever | quote only |
| SCHUNK CMS 050 (basic head and adapter) | not stated | yes, locking lever | EUR 741 (2024 list) |
| Destaco TC1 (TC1-050M-01 + TC1-050T-02) | no: tapered slot guide | no: "passive locking and push button release" | $343.65 + $121.80 = $465.45 |
| Milibar manual tool changer (Vention PR-MB-001-0109) | no evidence | yes, cam lever | $905.15 |
| OnRobot Quick Changer (robot and tool sides) | not stated | partly: sprung, with a release | $1,244 |
| OPT OC90 (made-in-china listing) | none shown | yes, ring with a lever | US$150, mounting pattern not stated |

None has both features at an affordable price, so the changer is specified to be built. Every dimension, spring
setting and mass in this README is a design value for this build. Masses are computed from the STL volumes (PETG,
1.27 g/cm³ from its data sheet) and the bought parts, and the build checks the finished tools on a scale.

### The inert wrist stand

- **Board:** 18 mm plywood, 200 x 180 mm, from 250 to 450 mm forward of the line between the arm bases and from 40 to
  220 mm left of the centre line. Two 100 mm C-clamps hold it to the table at its far edge, away from both arms.
- **Pedestal** (`wrist_pedestal.stl`, grey PETG, printed standing on its foot): a dummy wrist 68 mm across (a small
  cobot's last link), its axis 300 mm forward, 85 mm above the table, pointing right. Its front face, 26 mm behind the
  flange's face, is an ISO 9409-1-50-4-M6 interface: four M6 heat-set inserts on a 50 mm circle, a 31.6 mm centring
  recess 6.5 mm deep, and a 6 mm pin hole at 3 o'clock. Under the wrist a web runs down to a 100 x 100 x 8 mm foot,
  bolted through the board by four M6 x 40 bolts (6.6 mm holes at x ±42 mm, 10 and 90 mm back from the foot's front
  edge).
- The stand is unpowered and does not move. It presents the changer at the pose a parked robot's wrist did in earlier
  versions of this task. The lever swings clear of it: an interference check of the STL solids finds no overlap between
  the ring, claws, lever and pad and the stand at 0, 8, 20, 25 or 35 degrees.

### Drawings and files

Six drawings, made from the same constants as the simulation (mm): [`layout.svg`](drawings/layout.svg) (plan and
elevation on the table), [`changer_section.svg`](drawings/changer_section.svg) (section along the axis),
[`changer_front.svg`](drawings/changer_front.svg) (every angle), [`cam_profile.svg`](drawings/cam_profile.svg) (a claw
lip's cam, developed), [`tools.svg`](drawings/tools.svg) (tool plate, bushing detail, both tools and their ballast) and
[`stand_and_rack.svg`](drawings/stand_and_rack.svg).

Printed parts: PETG at 100 % infill (the pedestal and rack may be 30 %; their mass does not matter), 0.2 mm layers (0.1
mm for the claw ring, whose cam faces print facing up):

| STL file | Qty | Size (mm) | Mass (g) | Colour, print orientation |
|---|---|---|---|---|
| `flange_body.stl` | 1 | 74 x 74 x 32 | 98.3 | grey; mating face down |
| `retaining_disc.stl` | 1 | 76 x 76 x 4 | 18.1 | grey |
| `ring.stl` | 1 | 108 x 88 x 18 | 65.4 | black; back face down, no supports |
| `claw_ring.stl` | 1 | 88 x 88 x 15 | 29.2 | black; claw fronts down, no supports |
| `lever.stl` | 1 | 72 x 62 x 12 | 20.2 | orange; lap face down |
| `pad.stl` | 1 | 45 x 30 x 14 | 10.4 | black |
| `tool_plate.stl` | 2 | 69 x 66 x 12 | 21.1 | one blue, one grey; mating face down |
| `suction_body.stl` | 1 | 44 x 44 x 75 | 22.4 | blue |
| `suction_cap.stl` | 1 | 44 x 44 x 28 | 11.8 | black; with supports |
| `gripper_body.stl` | 1 | 44 x 44 x 62 | 23.7 | grey |
| `gripper_cap.stl` | 1 | 44 x 44 x 41 | 12.8 | black |
| `handle.stl` | 2 | 16 x 16 x 35 | 4.8 | black |
| `wrist_pedestal.stl` | 1 | 100 x 100 x 101 | 563.7 at 100 % | grey; foot down |
| `tool_rack.stl` | 1 | 200 x 60 x 73 | 480.6 at 100 % | grey |

### Bought parts and cost

Prices were read from the vendors' pages on 2026-11-16 (US Pacific) except where marked.

| Part | Qty | Source and part number | Price |
|---|---|---|---|
| Round dowel and the flange's orientation pin: 6 x 14 mm, ISO 2338 h8 (5.978 to 6.000 mm), 300 stainless | 2 | Olander [.6R14DOWS](https://www.olander.com/items/.6R14DOWS) | $0.891 each |
| Diamond pin: DIN 6321 form C, 6 g6 x 7 mm head, 1 mm land, 4 n6 x 6 mm press spigot, hardened tool steel | 1 | Halder [22630.0040](https://www.halder.com/PM/Standard-Parts/Machine-and-Fixture-Elements/Locating-Seating-Elements/Locating-Pins-DIN-6321/22630.0040), sold by SFS as [162629](https://www.sfs.ch/CH/en/dl/p/162629) | CHF 8.50 each, at least 5 |
| (US alternative) diamond pin, 5.954 to 5.961 mm head 8.6 mm long, 5 mm press-fit shank; seat it 2.6 mm deep | 1 | Carr Lane [CLM-60-DPY](https://www.carrlane.com/product/locating-pins/locating-pins/round-diamond-pins/diamond-pins/clm-60-dpy) | $9.17 (maker's listing) |
| Bushings: DIN 179 type A, 6 F7 bore (6.010 to 6.022 mm), 10 n6 outside, 10 mm long, hardened (HRC 62) | 4 | JW Winco [DIN 179-B6-10-A](https://www.jwwinco.com/en-us/products/quick-finder/DIN-179-Steel-Press-Fit-Drill-Bushings?q=179) | on request from the maker |
| Springs: Lee Spring LC 016A 01, music wire: 3.05 mm outside, 0.41 mm wire, 6.35 mm free, 2.90 mm solid, 17.50 lb/in (3.065 N/mm) | 6 (10 minimum) | Lee Spring 2014 catalogue, standard series, page 57 ([catalogue copy](https://www.rcfastener.com/documents/2014_Lee_Spring_Catalog.pdf)) | $5.99 each for 10 to 19 (2014 catalogue list) |
| Stop pin: 4 x 8 mm, DIN 6325 m6, hardened | 1 | Olander [.4R8DOWA](https://www.olander.com/items/.4R8DOWA) | $0.439 |
| M6 x 30 low-head socket cap screws, DIN 7984 | 4 | Olander [.6C30LHCA](https://www.olander.com/items/.6C30LHCA) | $0.642 each |
| M6 x 40 socket cap screws, M6 nuts, M6 washers (stand to board) | 4, 4, 8 | Olander [.6C40SHCA](https://www.olander.com/items/.6C40SHCA), [.6CHNTS](https://www.olander.com/items/.6CHNTS), [.6NFLWS](https://www.olander.com/items/.6NFLWS) | $0.509, $0.210, $0.127 each |
| M3 x 10 socket cap screws | 7 | Olander [.3C10SHCS](https://www.olander.com/items/.3C10SHCS) | $0.155 each |
| Heat-set inserts M6 (hole 8.0, 12.7 long) and M3 (hole 4.0, 5.7 long) | 4 and 7 | ruthex RUT-GE-M6x127-002 ([3DJake](https://www.3djake.com/ruthex/threaded-inserts-m6-25-pieces)) and RUT-GE-M3x57-001 ([3DJake](https://www.3djake.com/ruthex/threaded-insert-m3-100-pieces)) | $9.17 per 25, $8.25 per 100 |
| PETG filament in grey, black, orange and blue (about 1.4 kg in all at 100 % infill) | 4 spools | e.g. [Prusament PETG 1 kg](https://www.prusa3d.com/product/prusament-petg-jet-black-1kg/) (1.27 g/cm³, data sheet) | $23.99 per kg |
| Neoprene sheet, 60 Shore A, 3 mm, self-adhesive (the pad's 39 x 30 mm top) | 1 | e.g. Rubber-Cal 22-P04-C-125-04-020, 4 x 20 in ([Rubber-Cal](https://rubbercal.com/product/neoprene-60a-custom-rubber-strips-psa)) | $42.31 |
| Two-part epoxy (bushings, ballast, handles) | 1 | e.g. J-B Weld Twin Tube 8265S ([J-B Weld](https://www.jbweld.com/product/j-b-weld-twin-tube)) | $7.99 |
| Bright steel flat bar 16 x 8 mm (ballast), 18 mm plywood offcut 200 x 180, two 100 mm C-clamps | 160 mm, 1, 2 | any hardware store | ordinary stock |

The priced items come to $233.35: four spools of filament ($95.96, of which about 1.4 kg is used), ten springs at the
2014 price ($59.90), the neoprene sheet ($42.31, of which a small piece is used), the epoxy ($7.99), inserts ($17.42) and
pins, screws, nuts and washers ($9.77). Add the diamond pin (Carr Lane $9.17, or CHF 42.50 for five Halder pins), the
four bushings, the plywood, the steel bar and the clamps, and the whole rig costs about $300. No part is machined: the
tools needed are a 3D printer, a soldering iron for the inserts, 5.9 and 3.9 mm drills, a hacksaw and a file.

### Masses

| Body | Mass | Made of |
|---|---|---|
| Ring and lever (moves) | 140.4 g | ring 65.4, claw ring 29.2, lever 20.2, pad 10.4, neoprene 4.9 (1.40 g/cm³, typical of 60 Shore A neoprene), six springs 0.4, seven M3 inserts 3.2, seven M3 x 10 screws 6.7 (0.95 g each, computed from nominal sizes) |
| Blue suction-cup tool | 120 g | printed 60.1 (plate 21.1, body 22.4, cap 11.8, handle 4.8), two bushings 7.9, epoxy 1.0 (design allowance), a steel bar 16 x 8 x 50.8 mm 51.0 |
| Grey gripper tool | 140 g | printed 62.4 (plate 21.1, body 23.7, cap 12.8, handle 4.8), two bushings 7.9, epoxy 1.0, two steel bars 16 x 8 x 34.2 mm 68.7 |

The ring and lever's centre of mass is 30.1 mm from the axis toward the lever, so the lever's weight turns the ring the
locking way with 0.041 N m. The suction tool's centre of mass is 45.1 mm from the plate's face and 4.4 mm below the axis
(the gripper tool's 42.9 and 6.4 mm), so each lies handle up. The simulation uses these masses and centres of mass.

### Building it

1. Print the parts and press in the heat-set inserts: four M6 in the wrist's face; three M3 in the ring's front face (in
   the gaps), two through its lever stub, two in the lever's second arm.
2. Flange: drill the round dowel's and orientation pin's 5.8 mm printed holes to 5.9 mm and the diamond pin's spigot hole
   to 3.9 mm. Press in the round dowel (8 mm deep, 6 mm out) and the diamond pin (head in its 1 mm pocket, flats facing
   12 and 6 o'clock), each with a drop of epoxy.
3. Ring: a spring in each pocket; the claw ring on its front (three M3 x 10); slide it onto the journal; fit the disc
   with the stop pin (4 mm into the disc, into the ring's groove) and the orientation pin; bolt flange and disc to the
   wrist (four M6 x 30). Bolt on the lever and the pad (M3 x 10) and stick the neoprene on.
4. Tool plates: wax the dowels and lock a plate on with the ring (its cones centre it within 0.2 mm). From behind, put
   epoxy in each 10.4 mm hole and slide a bushing, filleted end first, over its dowel onto the printed shoulder. Once
   cured, each bushing sits on the axis of its dowel within the fit's clearance. Bond the body onto the plate and glue in
   the handle.
5. Ballast: bond the steel bar(s) on the cavity floor against the cap's end, then glue the cap on. Each tool must weigh
   120 g or 140 g (kitchen scale, ±3 g) and balance on a pencil laid across under it 45 mm (43 mm) from the plate's face.
6. Paint the disc's notches and the ring's index line white. Check: the lugs pass the claws with the index line at OPEN
   and not at 8; locked, a 0.2 mm feeler gauge does not enter between plate and face anywhere round the rim and the tool
   hangs; the ring stays where it is left, and a 200 g bag on the pad's middle (0.22 N m, lever horizontal) turns it. If
   not, drill the spring pockets deeper: each 0.1 mm lowers the preload by 1.84 N.

### Real-trial safety and rig protection

- No robot arm is involved apart from the two YAM arms: the stand is unpowered and has no moving parts but the ring.
- Nothing can contaminate the shared rig: the parts are dry PETG, steel and neoprene. There is no grease, oil, liquid or
  food, and the changer runs unlubricated.
- Nothing sharp meets the gripper pads. The pad is rubber-topped, the handles are square printed blocks, the dowels'
  tips are rounded and lie inside the plate, and the printed edges are deburred before the first trial.
- The largest force the task needs is about 3.5 N on the pad at the end of the clamp ramps. The tools weigh 120 g and
  140 g. Both are far inside the roughly 40 N a YAM finger gives.
- A dropped tool falls at most 85 mm (from the flange's axis to the table). PETG does not shatter, and the stand is
  clamped to the table, so nothing can tip or fly off.

## The changer as built

The changer is a manual locking-ring changer, a breech-lock. A ring with three claws turns on the flange, and the cam
faces on the claws' lips hook three lugs on the tool plate and pull the plate onto the flange's face. Every size is the
one the simulation uses (the task file's constants). Rows marked "build only" are parts the simulation does not draw.

Conventions:

- Positions along the axis are measured from the flange's mating face, positive into the stand. Negative means in front
  of the face, toward the tool.
- Angles round the axis are seen facing the flange from the tool side, counterclockwise from 12 o'clock. That is the
  way the ring turns when the lever is pressed down. "Out" is the distance from the axis.

### Section along the axis (above the axis)

| Along the axis (mm) | Out (mm) | Part |
|---|---|---|
| -11.0 to 0 | 38 to 44 | Claw walls (3 x 80 degrees of arc), part of the claw ring |
| -11.0 to -2.8 | 32 to 38 | Claw lips. Their back faces are the cams |
| 0 to +4.0 | 38 to 44 | The claw ring's band, bolted to the ring's front face |
| -8.0 to 0 | 0 to 30 | Tool plate |
| -3.0 to 0 | 30 to 36 | The tool plate's lugs (3 x 24 degrees) |
| -6.0 to 0 | 22 (6 mm pins) | Dowels standing out of the face |
| 0 to 3.0 | 0 to 37 | The flange's front lip. Its front is the mating face |
| 3.0 to 8.75 | 35 (3.05 mm springs) | Six springs, from the lip's back into 4.75 mm pockets in the ring's front (build only) |
| 3.0 to 22.0 | 0 to 32 | The flange's journal |
| 4.0 to 22.0 | 32.2 to 44 | Locking ring |
| 22.0 to 26.0 | 15.9 to 38 | Retaining disc |
| 22.0 to 32.0 | 0 to 15.75 | The flange's spigot, through the disc into the wrist's recess |
| 26.0 | | The wrist's ISO 9409-1-50-4-M6 interface |

### Round the axis (ring open)

| Angle | Robot side | Tool side |
|---|---|---|
| 0 (12 o'clock) | Round dowel (22 mm out); stop pin (35 mm out, in the disc); the ring's index line; OPEN mark; claw-ring screw (40.5 mm out) | Lug; bushing for the round dowel |
| 8 | Mark: end of the open window | |
| 25 | LOCK mark | |
| 30, 90, 150, 210, 270, 330 | Springs (35 mm out) | |
| 45, 135, 225, 315 | M6 mounting screws on a 50 mm circle | |
| 60, 180, 300 (10, 6 and 2 o'clock) | Claw centres. Each claw spans 40 degrees either side (20 to 100, 140 to 220, 260 to 340), leaving 40-degree gaps centred on 0, 120 and 240 | |
| 90 (9 o'clock) | Lever | |
| 120, 240 (8 and 4 o'clock) | Claw-ring screws | Lugs |
| 180 (6 o'clock) | Diamond pin (22 mm out) | Bushing for the diamond pin |
| 270 (3 o'clock) | Orientation pin (25 mm out, behind the flange) | |

### Parts

| Part | Size and shape | Fit, fixing, material |
|---|---|---|
| Flange body | Lip 74 mm across, 3 mm thick; journal 64 mm across, to 22 mm behind the face; spigot 31.5 mm across, to 32 mm. Four 6.4 mm holes, 11 x 4 mm counterbores, on a 50 mm circle | PETG. Four M6 x 30 low-head screws through it and the disc into the wrist; the orientation pin sets its turn |
| Dowels | Round ISO 2338 6 h8 x 14 at 0 degrees; diamond DIN 6321 form C (6 g6, 7 mm head sunk 1 mm, 1 mm lands facing 90 and 270 degrees) at 180; both stand 6 mm out, 44 mm apart | Pressed and bonded into drilled printed holes |
| Locking ring | 88 mm across, 4 to 22 mm behind the face, 64.4 mm bore. Lever stub at 90 degrees: 12 mm wide, 7 to 22 mm behind the face, its lower 6 mm half out to 64 mm | PETG. 0.4 mm running clearance on the journal |
| Springs | Six LC 016A 01 in 3.3 x 4.75 mm pockets, 35 mm out, 5.75 mm long between the pockets' floors and the lip's back | 0.60 mm compressed: 1.84 N each, 11.0 N in all, holding the ring against the disc |
| Claw ring | Band 38 to 44 mm out, 0 to 4 mm behind the face; claws centred on 60, 180, 300 degrees, 80 degrees of arc each: wall 38 to 44 mm out to 11 mm in front of the face, lip 32 to 38 mm out. Lips' inner front edges chamfered 4 x 45 degrees | PETG. Three M3 x 10 through the band (in the gaps) into inserts in the ring |
| Cam (each lip's back) | From the leading end (at 100, 220, 340 degrees): 10.0 mm in front of the face at the end, meeting 1 mm of arc on a lead-in ramp, 9.0 to 3.6 mm over 12 degrees (37 degrees at 34 mm out); a clamp ramp to 2.8 mm over 5 degrees (15 degrees); a dwell at 2.8 mm for 63 degrees | Home lug fronts are 3.0 mm out: the dwell overlaps them 0.2 mm, the ring moves 0.2 mm forward, and the springs press 14.7 N |
| Retaining disc | 76 mm across, 22 to 26 mm behind the face, 31.8 mm bore; rim notches at 0 (OPEN), 8 and 25 degrees (LOCK) | PETG, clamped under the mounting screws; the ring's rear thrust face |
| Stops | A hardened 4 x 8 mm pin, half in the disc, 35 mm out at 0 degrees, in a groove in the ring's back (4.2 wide, 4.5 deep, r 35, ends at -35 and +1 degrees) | Stops the ring at 1 degree past open and at 35 (STL check: no overlap from -1 to 35, overlap at -1.3 and 35.3) |
| Lever | 12 x 12 mm bar, half-lapped onto the stub from 44 to 64 mm out, to 104 mm; a second arm, its centre line 110 mm out, from 7 to 69 mm behind the face | Orange PETG; two M3 x 10 into the stub |
| Pad | 45 x 30 x 8 mm (5 mm printed, 3 mm neoprene), its middle 110 mm out and 63 mm behind the face, its top 14 mm above the lever's line; a 6 x 6 mm lip on its outer edge | Two M3 x 10 into the second arm, under the neoprene |
| Tool plate | 60 mm across, 8 mm thick, lugs 30 to 36 mm out at 0, 120, 240 degrees (24 degrees wide, 3 mm thick, flush with the face); back pocketed 5 mm inside the body | PETG; the body is bonded onto its back |
| Bushings | DIN 179, 6 F7 x 10 n6 x 10, at 0 and 180 degrees, 22 mm out, 1.5 to 11.5 mm behind the face, in 14 mm bosses; a printed 45-degree cone in front, 9.4 to 6.4 mm across | Bonded in 10.4 mm holes on the flange's own dowels |
| Tools | 120 g and 140 g; a 16 mm square handle on top, 50 mm from the face; 1.6 mm walls, a printed end cap | Steel ballast bars inside ("Masses") |

### What the dowels capture

The simulation's seating rule (`_seat_check` and `_dowel_fit` in the task file) accepts a plate pose only where the
full-size dowels physically fit in the bushings, so the offset and tilt it allows shrink as the dowels go in. For each
dowel, **d** is how far its tip (its deepest edge) has gone past the plate's face, **e** how far its axis is off the
bushing's where it crosses the plate's face, and **t** its tilt. At depth d its widest point reaches
e + d tan t + 3 mm / cos t from the bushing's axis, and the hole there has a radius of 3 mm + 0.005 mm + (1.5 mm - d) in
the 45-degree chamfer, 3 mm + 0.005 mm in the bore. So the dowel is clear while

> e + d tan t + 3 mm x (1/cos t - 1) <= 0.005 mm + max(0, 1.5 mm - d)

0.005 mm is the fit's smallest radial clearance: the round dowel (5.978 to 6.000 mm) in the bushing's F7 bore (6.010 to
6.022 mm). Both dowels are checked in every direction as full 6.000 mm pins (the diamond pin, 5.988 to 5.996 mm and
relieved, only adds room), their own end chamfers are left out, and the printed lead-in cone is 0.2 mm larger in radius
than the modelled chamfer (covering the printer's size and position error), its narrow end meeting the bushing's
filleted entry. So every accepted pose fits every build. Seated also needs both tips past the plate's face, the plate's
face nowhere past the flange's, every lug front within the lips' 10.0 mm reach, and, to go on, the lugs between the lips
(within the 8 degrees the gaps leave, counting asin(offset / 30 mm)).

The largest offset accepted for a square, unturned plate, printed from the task file (at zero tilt the rule is exact:
the brute-force clearance is 0 at the limit, and 0.01 mm further intersects by 10 µm):

| Gap at the middle (mm) | 5.90 | 5.75 | 5.60 | 5.50 | 5.25 | 5.00 | 4.75 | 4.60 | 4.50 and less |
|---|---|---|---|---|---|---|---|---|---|
| d (mm) | 0.10 | 0.25 | 0.40 | 0.50 | 0.75 | 1.00 | 1.25 | 1.40 | 1.50 and more |
| Largest offset e (mm) | 1.405 | 1.255 | 1.105 | 1.005 | 0.755 | 0.505 | 0.255 | 0.105 | 0.005 |

- **Every pose checked:** of 20,520 grid poses (gap 0 to 6.25 mm, offset 0 to 1.6 mm, tilt 0 to 3 degrees), the rule
  accepts 4,318 and none intersects in a brute-force test of the dowel's surface against the bushing's chamfer and bore;
  the 424 accepted with a dowel in the bore are within 0.005 mm and 0.1 degrees. Of 20,000 random poses (also turned,
  ring at 0 to 8 degrees), 2,926 are accepted and none intersects. Tilted, the rule is conservative.
- **An earlier review's example:** a plate 1 mm from the face has its dowels 5 mm in, so each must be within 0.005 mm. A
  1 mm offset is rejected (it would intersect by 995 µm), and so are 0.01 mm (-5 µm) and 0.006 mm (-1 µm).
- **In the simulation** the plate rests on the stops with the dowels' tips 0.4 mm into its chamfers, so each dowel may be
  up to 1.105 mm off. In a debug run of the final demo the plate sat seated with the ring open for about 0.4 s before the ring
  took it: d 0.34 to 0.35 mm, e at most 0.86 mm against at least 1.15 mm of room, tilt 0.19 degrees.
- **Claws and lugs:** in the simulation the plate rests 5.6 to 5.7 mm out, its lug fronts 8.6 to 8.7 mm out, inside the
  lips' 10.0 mm reach; the ramp falls 0.45 mm per degree, so it meets them at about 8.8 degrees. The plate's rim (30 mm
  out) clears the lips (32 mm) at any offset under 2 mm. Checked on the STL solids: a plate moved straight on and off
  clears the claws with the ring at 0 and 7.9 degrees and hits them at 8.5.
- **Taking the tool:** past 8 degrees the lips are over the lugs. On the real changer the lead-in ramps draw the plate
  home by 20 degrees, the chamfers centring it, and the clamp ramps pull it tight by 25. Checked on the STL solids: with
  the plate home and the ring at 25, 30 and 35 degrees, lugs and lips overlap by 0.902, 8.024 and 15.146 mm³, and by 0
  with the claw ring moved 0.2 mm forward (the clamp travel). The simulation does this at once at 8 degrees: a fixed
  joint holds the plate on the face, centred and square.

### Lever movement

The pad's top middle at each ring angle, printed from the task file:

| Ring turned (degrees) | Pad top drop | Pad top height above the table | What happens |
|---|---|---|---|
| -1 | 1.9 mm up | 100.9 mm | Open stop |
| 0 (at rest it sits at 0.7, under the lever's weight) | 0 | 99.0 mm | Open: the lugs pass between the lips |
| 8 | 15.4 mm | 83.6 mm | The lips reach the lugs. From here a plate can no longer go on or come off |
| 8 to 20 | to 38.5 mm | to 60.5 mm | The lead-in ramps draw the plate home |
| 20 to 25 | to 47.8 mm | to 51.2 mm | The clamp ramps pull it tight: 0.2 mm, 14.7 N |
| 25 to 35 | to 65.6 mm | to 33.4 mm | On the flat dwell: locked (LOCK mark at 25). Hard stop at 35 |

To release, lift the pad. Below 25 degrees the lugs ride down the clamp ramps and the plate is no longer pulled tight.
Below 8 degrees the lugs line up with the gaps and the tool comes off.

### Forces

Computed with a friction coefficient of 0.15 for PETG on PETG and on spring steel (design value).

- **Holding the ring:** the springs' 11.0 N acts on the lip's back and on the disc, both 35 mm out: 0.12 N m. The
  lever's weight gives 0.041 N m toward locking. The setup check in "Building it" holds a build between 0.041 and
  0.22 N m.
- **Turning it:** pressing down takes 0.7 N on the pad, rising to 3.5 N at the end of the clamp ramps (the ramps'
  15-degree wedge and their friction add 0.25 N m). Lifting takes 1.5 N.
- **Holding the tool:** the 14.7 N clamp, acting 33 mm out, gives 0.49 N m against tipping. The 120 g tool,
  its weight 45 mm out, needs 0.053 N m.

The trial is judged from the marks, the feeler gauge and the video; the changer needs no electronics.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The stand and the tool rack are kinematic, fixed where
they are put; the ring with its lever and the two tools are loose rigid bodies made of shapes from `bodykit.py`
(importing it hooks `yam_arena.scene` without changing yam_arena, and lets the task file act after every control step).
Every changer size above is a constant in the task file. The simplifications, as in the fidelity note:

- **The ring:** a hinge about the flange's axis, between its stops at 1 degree past open and at 35 degrees. Ring and
  lever weigh 140.4 g, their centre of mass 30.1 mm toward the lever, so the lever's weight (0.041 N m) turns the ring
  0.7 degrees at rest. A spring (0.06 N m per degree) holds the ring where it last stopped; pushed more than 2 degrees
  past its hold, it gives and the task file moves the hold there. So it gives at 0.12 N m and stays where it is left,
  standing in for the springs' friction. The clamp ramps' extra torque (about 0.25 N m) is not modelled.
- **No contact:** the claws, lips, lugs, dowels and bushings are drawn at their build sizes but collide with nothing (the
  lips without their chamfers, the lead-in ramp as two steps). The tool plates have no bores, so two collision-only stops
  (`dowel_stops`), 6 mm across and 5.6 mm out of the face inside the drawn dowels, hold a pushed plate with the dowel
  tips 0.4 mm into its chamfers. A real plate pushed on harder goes past the chamfers, which centre it, and then hangs on
  its dowels; in the simulation it cannot. The stops move away while the ring holds a tool.
- **Seating and lock:** after every control step the task file checks each tool against the flange (`_seat_check`).
  Because the drawn dowels do not centre it, a held plate keeps whatever offset and tilt it has inside that check. A tool
  seated while the ring is within 8 degrees of open is taken as the ring passes 8 degrees: a fixed joint (a `Glue` to a
  hidden fixed body) holds it drawn home onto the face, centred and square, with no play (a real ring's ramps take it
  home by 20 degrees). From 25 degrees it counts as locked; turned back below 25 it does not; the joint switches off
  once the ring is back within 8 degrees of open.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the suction tool still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps the layout fixed. Dream-1 (needs a key in `~/.dreamlabs/credentials`); for the
scripted demo use `--policy yam_scripted`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lock_a_tool_onto_a_robots_quick_change_flange.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the blue suction-cup tool to the tool flange on the wrist stand and lock it on, leaving the gripper tool in the rack. Then let go of everything, with the tool hanging locked on the flange, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flange-dream
```

The scripted demo:

1. The right hand comes straight down on the suction tool's handle, closes across it and lifts it out of the rack.
2. It holds the plate on the dowels' tips (on the stops), its lugs between the claws. The wrist is turned 3.4 degrees
   about the vertical against the finger pads' twist, pitched back 0.85 degrees against the pitch from the tool's weight
   (its centre of mass 4.9 mm toward the plate from the handle) and the push at the handle, and rolled back 0.65
   degrees about the tool's axis against the roll the pads give it (all measured in debug runs). It aims 4.6 mm high
   (the measured hold offset) and 2.5 mm past the tips (the arm stops short of a pushed target).
3. The left hand comes down onto the lever's pad as the plate reaches the tips, presses it at once to 12 degrees, then
   on to 32: past 8 the claws take the plate; from 25 it is locked.
4. The right lets go and draws back, the left rises, the tool hangs on the flange, and both arms go home.

`FLANGE_DEBUG=1` prints, twice a second, the ring's angle, the suction tool's gap, offset, tilt and turn, each dowel's
d, e, need and room, which seating checks pass, the lock state, whether the hands are clear, and each TCP's distance
from the handle and the pad.

## Runtime per trial

On one RTX 5090: about 10 minutes for a Dream-1 trial of 1,200 steps (573 s on the clock, with Dream answering in 0.75 s on average), and about 3 minutes for the scripted demo (358 steps, 17.9 s of simulated time, 202 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Third-party assets

None. Every object in the scene is built from boxes, cylinders and spheres in the task file, and the drawings and STL
files are our own.

## Changes in this revision

This is a revision of TK-PEDV27. Its review said:

> **Not accepted** (TK-PEDV27).
>
> Feasibility (not met): The trial specifies a full robot arm and a custom precision-machined steel changer without an
> affordable sourcing or fabrication route.
>
> What to change: The previous feedback on the seating window is addressed in the submitted description and demo. New
> point, not raised before: the real trial calls for a full robot arm and a custom precision-machined changer without
> an affordable sourcing or fabrication route. Please provide a low-cost inert wrist stand and either an affordable
> compatible changer identified by part number or fabrication-ready files and a low-cost build route for the specified
> changer. Preserve the two-dowel alignment and manual locking action.

The review before that (TK-Z5S8AY) asked for a seating window coupled to insertion depth, so that every accepted pose is
physically achievable; that was done and is kept (now at this build's fit). The review before that (TK-UU8KFN) asked for
a seating window achievable with the specified dowels and bushings; that is kept. The review before that (TK-TECUBS)
asked for the dowel locations, the cam and lug profiles, and how the ring is mounted, retained and stopped; that is
kept and is now drawn. The first review (TK-S6GB3S) asked for a buildable flange, tool plate and lock, with the
simulation approximation kept in the fidelity note; that is kept too.

What changed in response:

- **Feasibility (not met), a full robot arm and a precision-machined steel changer with no affordable route:** the real
  trial now needs neither. An unpowered printed wrist stand replaces the arm, and the changer is printed in PETG round
  bought precision parts. The priced parts come to $233.35 and the whole rig to about $300; nothing is machined.
- **"Provide a low-cost inert wrist stand":** a printed pedestal with a 68 mm dummy wrist whose ISO 9409-1-50-4-M6 face
  (four M6 inserts, a 31.6 mm recess, an orientation pin) holds the changer at the task's pose (axis 300 mm forward,
  85 mm up), bolted through a plywood board held by two C-clamps (`layout.svg`, `stand_and_rack.svg`,
  `wrist_pedestal.stl`). The simulated scene now shows this stand in place of the arm.
- **"Either an affordable compatible changer identified by part number":** we checked catalogue manual changers (ATI
  MC-50, SCHUNK CMS 050, Destaco TC1, Milibar, OnRobot, OPT OC90 and others). The only one shown to have both two
  alignment pins and a hand lever, the ATI MC-50, is sold on quote; the priced ones cost $465 to $1,244 or show no
  dowels.
- **"Or fabrication-ready files and a low-cost build route for the specified changer":** six dimensioned SVG drawings
  made from the task file's constants; 14 STL files covering all 16 printed pieces in their print orientations; a parts
  list with part numbers, links and prices; masses; build steps and setup checks. The bushings are bonded in place on
  the flange's own dowels, so printing tolerance does not set the fit.
- **"Preserve the two-dowel alignment":** kept. A round dowel (ISO 2338, 6 h8) at 12 o'clock and a diamond pin (DIN 6321
  form C, 6 g6) at 6, 44 mm apart, in DIN 179 bushings (6 F7) behind 1.5 x 45 degree lead-ins. The seating rule now uses
  this fit's smallest radial clearance, 0.005 mm (0.004 mm before). Re-checked against the brute-force geometry test:
  4,318 of 20,520 grid poses and 2,926 of 20,000 random poses accepted, none intersecting.
- **"And manual locking action":** kept, with the same geometry: the three-claw ring and lever, the same cam (lead-in
  9.0 to 3.6 mm over 12 degrees, clamp ramp to 2.8 mm over 5, dwell) and stops (1 degree past open, 35). No wave washer
  of the earlier size is sold at its load (the nearest gives about 200 N), so six Lee Spring LC 016A 01 springs now
  preload the ring: 11.0 N at rest, 14.7 N clamping. Checked on the STL solids: a plate passes the claws at 0 and 7.9
  degrees and is blocked at 8.5; locked, the lips overlap the lugs by exactly the 0.2 mm clamp travel.
- **The simulation follows the build, re-run:** the stand replaces the arm; the lip is 74 mm (the springs bear on it);
  ring and lever weigh 140.4 g (printed; 661 g of steel before), centre of mass 30.1 mm toward the lever; the tools'
  centres of mass come from their parts and ballast; the seating rule uses 0.005 mm; the status light is gone (the LOCK
  mark is the indicator); the instruction names "the tool flange on the wrist stand"; the demo's hold was retuned
  (pitched back 0.85 degrees and rolled back 0.65 against the new centre of mass, aimed 4.6 mm high, and the lever
  pressed as the plate reaches the tips). Demo 1.0 (358 steps); Dream-1 0.0 (it moves the right hand to the rack and
  lifts neither tool); one-hand check 0.25 (seated with the ring open for 5.5 s, then the tool fell 54 mm).
- **Also:** a "Why it matters" section ties the task to how robot cells are built and re-tooled on manual changers
  (Impact on AI timelines: robot building), and a "Real-trial safety and rig protection" section was added.

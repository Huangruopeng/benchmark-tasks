# Load a flask onto a rotary evaporator: seat its socket on the tilted cone, clamp it, lower it into the bath

A benchtop rotary evaporator (Heidolph Hei-VAP Core type, motor lift, vertical condenser) stands on the lab bench in
front of the arms, its lift up and its EASY CLIP flask clamp open. A 100 mL round-bottom flask with an NS 24/40 socket,
holding 40 mL of extract, stands in a cork ring beside it. The job is the step every chemist does by hand before each
evaporation, done the maker's way: put the flask on the vapour tube's ground cone, press the clamp's lever over the
flask's collar until it clicks, then lower the flask into the water bath with the lift. The task is one task file,
[`load_a_flask_onto_a_rotary_evaporator_and_lower_it_into_the_bath.py`](load_a_flask_onto_a_rotary_evaporator_and_lower_it_into_the_bath.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder. The real trial runs on a low-cost fixture with the same geometry, a real NS 24/40 cone,
a printed clamp and a keyed 12 V lift (section "Real-trial fixture", printed parts in
[`rotavap_fixture.scad`](rotavap_fixture.scad)), so it needs no evaporator.

Why it matters (category Other, laboratory research automation): a rotary evaporator is standard equipment in
synthetic, medicinal, natural-product and analytical chemistry labs, used to strip the solvent after nearly every
reaction work-up, extraction and column; the flask goes on by hand before every run. Loading an instrument the way its
maker says, with the instrument's own clamp and lift, is the kind of wet-lab step that research-lab automation has to
cover.

The instruction gives the goal and its constraints only ("Put the flask of extract on the rotary evaporator, clamped,
and lower it into the water bath, ready to run; then return both arms to where they started."). It says neither how to
hold the flask, nor which way the joint goes on, nor where the lift's keys are.

What it tests: an insertion along a tilted axis, a fixed order, and a hand-off of the load. The vapour tube's cone
points toward the arms and down at 30 degrees. The flask's socket goes onto it only moved straight along that axis,
lined up within a few millimetres and degrees; the flask has to be turned from standing upright in its ring to lying
along the slope on the way. Once home it is not yet carried: a dry ground joint does not hold a flask on that slope,
so it slides off the cone and falls into the bath as soon as it is let go. The clamp closes, with its click, only
when its hook can drop behind the flask's collar, that is, once the socket is fully home. Only a clamped flask may be
let go, and only a clamped flask may go down into the bath.

Why two hands: one hand has to keep the flask pressed home on the tilted cone while the clamp is shut, and the clamp's
lever lies on top of the joint, where the holding hand can't press it. Let go first and the flask slides off; press the
lever first and it springs back open (no flask home to click over). The demo's right hand holds the flask by its neck,
from the side; the left presses the lever, square to the joint, and then works the lift's key; the wrists stay more
than 20 cm apart. With one hand the job fails. The right arm alone, doing exactly its part of the demo with the left
at home, slid the socket home on the cone; when it let go, the unclamped flask slid off the cone and fell into the bath
(score 0.0). The left arm alone pressed the lever, which sprang back open with nothing to click over, and ran the lift
down; the flask never left its cork ring (0.0).

## Real-world basis

- **The procedure (maker's manual):** Heidolph's operating instructions for the Hei-VAP Core, section "Mount/remove
  evaporation flasks": the evaporation flask is fixed to the vapor tube with a special retort clamp (EASY CLIP). Move
  the flask lift to the upper end position; "Place the flask in the EASY CLIP so that the ground joints of the flask
  and the vapor tube are fully aligned next to each other in a straight line and press the clamping lever of the EASY
  CLIP over the flask collar until the clamping lever audibly clicks into place"; then move the flask lift to the lower
  end position. The same section warns: "In the event of improper handling, the evaporation flask can fall to the
  floor." The motor-lift variant moves the lift while its arrow key is held ("The movement stops as soon as you release
  the button!"); the immersion depth is set with a stop on the lift base, and the lift moves down only to it.
- **The machine (maker's data):** Hei-VAP Core: 739 x 845 x 432 mm with motor lift and G3 glassware, about 15 kg
  without glassware, 10 to 280 rpm, flask lift stroke 155 mm at 25 mm/s, heating bath 253 mm across inside and 291 mm
  outside, 20 to 100 degrees C with water; EASY CLIP flask clamps for NS 24 and NS 29 joints come with it (manual,
  technical data and scope of delivery). The product page gives the flask's inclination as adjustable from 20 to 80
  degrees, a 4.5 L bath, and NS 24/40 joints on the North American models.
- **The joint:** NS 24/40 is the standard taper: the cone is 24 mm across at its wide end and 40 mm long, tapering
  1:10 to 20 mm (computed).
- **Other makers clamp the same way:** BUCHI's Rotavapor R-300 page names "fast flask handling with Combi-Clip"; the
  flask goes on the vapour duct's joint and a clip holds it.

Sources (all read for this task): Heidolph Hei-VAP Core operating instructions,
https://www.heidolph.com/Documents/Operation%20manuals/rotary%20evaporator/Operation-Manual-Rotary%20Evaporator-Hei-VAP-Core.pdf
(English part); Heidolph Hei-VAP Core product page,
https://www.heidolph.com/america/en/hei-vap-core-hand-lift-model-with-g3-vertical-glassware~p227; BUCHI Rotavapor
R-300 product page, https://www.buchi.com/en/products/instruments/rotavapor-r-300.

## How it is modelled

- **Scene, at real size.** The station table is cut out where the lab bench is: the bench top is 60 mm below the
  arms' deck, which puts the flask's working height, lift up, inside the arms' two-handed reach. The evaporator's base
  unit, lift column and condenser are boxes and cylinders, the condenser's top about 80 cm above the bench (maker:
  845 mm overall); the bath stands in front of the base unit, so that the flask hangs facing the arms. The heating bath is the maker's
  253/291 mm pot with 3.0 L of water (the water is a surface, drawn only). The head on the lift carries the drive
  housing, the glass vapour tube, the EASY CLIP ring and the NS 24/40 cone, its axis 30 degrees below horizontal.
- **The flask** is one rigid body: a 64 mm bulb, a hollow neck (25 mm bore, 30 mm outside), a 34 mm bead round its
  mouth; 0.10 kg with its 40 mL of extract, the centre of mass 8 mm below the bulb's centre. In the real trial
  dry glass beads bring the real flask to the same 100 g.
- **The ground joint.** The cone's collision core (14 mm square, its last 12 mm a 9 mm square tip) fits inside the
  socket's 25 mm bore, so a socket pushed at the cone well off its axis meets the cone's tip and does not go on. Once
  the socket's mouth is within 5.5 mm of the cone's axis and its axis within 10 degrees of the cone's (the 20 mm tip
  inside the 24 mm mouth, with the hand's give), and the cone is at least 8 mm inside it, a PhysX D6 joint
  takes over: the flask slides along the axis and turns about it (the joint's taper keeps it centred) and nothing
  else. Unclamped, nothing but gravity and a little joint friction (3 N s/m) acts along the axis, so a flask let go
  slides off and falls: in the no-clamp check it was off the cone and in the bath within a second of being let go.
  Home means within 4 mm of fully on.
- **The clamp.** The EASY CLIP's lever is its own body on a hinge across the joint, sprung open (0.004 N m per degree,
  about 5 N on its thumb pad to shut it). Its hook collides with the flask's collar, so with the socket short of home
  the lever can't reach its click. Pressed to within 3 degrees of shut with the socket home, it
  clicks: a stiff spring then holds it shut, its hook behind the collar keeps the socket from sliding out, and the
  joint's drive draws the socket against the end of the cone.
- **The lift and its keys.** The head rides on a vertical slide driven at 25 mm/s (the maker's figure) while the DOWN
  key (or UP) is held in past its switch point (2 mm of its 3.5 mm travel; each key is a 10 g cap on a 6 N preloaded
  spring), stopping at the immersion stop 95 mm down, where the bulb's centre is 12 mm under the water.

## Scoring

The score follows `score()` in the task file; `task.json` has the same text.

- Latched for the rest of the trial: **0.0** once the flask has been dropped: off the cone and fallen below 35 mm
  above the deck outside its cork ring (into the bath, onto the bench or the deck).
- Otherwise judged at the end of the trial, on live states:
  - **1.0**: the flask home on the vapour tube's cone (its mouth within 4 mm of the cone's base), the EASY CLIP clicked
    shut over its collar, the lift down at its immersion stop (within 4 mm), so the bulb sits in the bath's water, and
    both arms back where they started (each gripper's fingertip centre within 5 cm of its start);
  - **0.75**: the same with an arm not back;
  - **0.5**: the flask home and clamped, the lift not down;
  - **0.25**: the flask on the cone (its socket onto it), not clamped;
  - else **0.0**.
- The trial ends at the run limit, or once the full score has held for 1 s with the flask, the head and the lever
  still.

In a real trial (on the fixture) each tier is read with simple checks. Home: no frosted cone shows between the
flask's bead and the clamp ring, the mouth within 10 mm of the ring (ruler; 6 mm when fully on). Clamped: the lever
lies shut on the joint with its hook snapped round the socket behind the bead, which it does only with the socket
home, and it stays shut when let go (photo; the snap is heard). Lift down: the carriage on its immersion-stop switch,
the cone's base 54 mm above the deck (ruler) and the bulb inside the pan below its rim (photo). On the cone: the socket
over the cone (photo). Dropped: the trial video shows the flask off the cone, falling. The arms are checked against
marks at their start.

## Checks

Each check is the demo changed in one way (`RV_ONEHAND`, `RV_CHECK` or an aim offset), run in the simulator:

| Check | What happened | Score |
|---|---|---|
| Full demo, three runs | the socket met the cone 1.3 and 1.9 mm off its axis (the two runs with debug output), slid home and was drawn 0.7 mm onto the cone's end by the clamp; clamp clicked, flask lowered 95.9 mm into the bath, arms home | 1.0 each |
| Grip aimed 2 mm or 4 mm off the axis sideways, or 3 mm low (three runs) | the socket met the cone 1.3 to 4.3 mm off its axis and went on; home, clamped, lowered | 1.0 each |
| Right arm alone (`RV_ONEHAND=right`) | slid the socket home on the cone and let go: the unclamped flask slid off and fell into the bath | 0.0 |
| Left arm alone (`RV_ONEHAND=left`) | pressed the lever (it sprang back open: no flask home to click over) and ran the lift down; the flask never left its cork ring | 0.0 |
| No clamp (`RV_CHECK=noclip`) | socket home, then let go: off the cone and in the bath within a second | 0.0 |
| Lever pressed first (`clipfirst`) | the lever, pressed shut with no flask on the cone, sprang back open; the flask then went home and was let go: it slid off into the bath | 0.0 |
| Socket held 50 mm out, never slid on (`noseat`) | the socket never went on the cone; the lever sprang back open; the open hand dragged the loose flask away with it | 0.0 |
| Socket pushed only to 8 mm short of home (`short`) | the socket caught but stopped 16 mm short; the lever stopped at 4 degrees, no click; let go, the flask slid off into the bath | 0.0 |
| Socket aimed 8 mm off the axis (`RV_AIMY=0.008`) | the socket went 10 mm onto the cone's narrow tip 7 mm off its axis and stuck there, never caught; the lever didn't click; let go, the flask fell into the bath | 0.0 |
| Lift never lowered (`nolower`) | flask home and clamped, left up above the bath | 0.5 |

The socket can't be pushed onto the cone from the side: off its axis it meets the cone's tip. A flask set down
anywhere else, or let go before the clamp clicks, ends on the bath's floor or the bench, which is a drop.

## Real trial

- **Instrument:** the fixture below, built to the simulated sizes. A lab's own motor-lift rotary evaporator set the
  same way (vacuum, rotation and heating off, tilt 30 degrees, immersion stop 95 mm below the top, bath dry) is an
  equivalent.
- **Flask:** Chemglass CG-1506-05 (100 mL heavy-wall round-bottom, NS 24/40) holding dry 4 mm glass beads, added
  until the flask weighs 100 g on a kitchen scale (the simulated flask's 0.10 kg). No liquid anywhere in the trial.
- **Layout:** the arms' deck (the arm bases, the cork ring and the key box) is 60 mm above the fixture's base board,
  as the lab bench is 60 mm below the deck in the simulation.
- **Measuring:** a ruler, the overhead and side cameras, and marks for the arms' start poses.
- **Safety and rig protection:** nothing is hot, wet, under vacuum or turning; the fixture runs on 12 V; the grippers
  touch only the flask's outside, the lever's thumb pad and the keys; the pan's floor carries a 3 mm rubber mat.
  Reset as in the reset notes.

## Real-trial fixture

A stand-in for the evaporator that keeps everything the robot works with: the real flask, a real NS 24/40 ground cone
held at 30 degrees, a clamp sized and placed as the EASY CLIP that snaps over the flask only with its socket home, a
lift worked by two keys that stops at its immersion stop, and a pan where the bath is. Every part is a catalogue part
with its number and price, a printed part in [`rotavap_fixture.scad`](rotavap_fixture.scad) (every dimension a named
parameter, with the parts' positions in its header), or ordinary hardware. The named parts come to **US$263.89**
(prices from the pages below, November 2026).

### Parts

| Part | Number | US$ | What it is (seller's page) |
|---|---|---|---|
| Flask | Chemglass CG-1506-05 | 37.95 | 100 mL heavy-wall single-neck round-bottom, 24/40 socket |
| Ground cone | Chemglass CG-100-06 | 15.35 | 24/40 inner joint, full length, on 22 mm OD x 1.8 mm tubing |
| Lift actuator | Pololu 4463 (Glideforce GF01-120510-1-66) | 74.95 | 12 V, 100 mm stroke, limit switches at both ends (reverses away from them), 28 mm/s unloaded, 21 mm/s at its 2.2 kgf rating, holds position unpowered; 157 / 257 mm between its 4.25 mm holes |
| Speed trim | Pololu 2572 (S18V20ALV) | 32.95 | regulator, output set from 4 to 12 V with a trimmer |
| Power | Pololu 1471 | 12.95 | 12 V, 1 A wall adapter, 5.5 x 2.1 mm plug |
| Keys, 2 | Pololu 1402 | 4.74 | snap-action switch, SPDT, 16.7 mm lever (2.37 each) |
| Immersion stop | Pololu 1404 | 2.51 | snap-action switch, SPDT, 16.3 mm roller lever |
| Lift guide | South Main Hardware SLIDE-BB-10 (Zoro G905299001) | 25.35 | 10 in full-extension ball-bearing slide, 45 mm wide (sold in 3s, 8.45 each) |
| Bath | Fat Daddio's PRD-104 (WebstaurantStore 627PRD104) | 20.99 | straight-sided anodized aluminium pan, 10 in (254 mm) across at the top, 4 in deep, 11 in outside |
| Cork ring | Chemglass CG-3060-100 | 10.65 | 100 mm outside, 50 mm hole, for 150-250 mL flasks |
| Mass stand-in | Lab Alley 11-312B | 25.50 | solid soda-lime glass beads, 4 mm |
| Printed parts | `rotavap_fixture.scad` | | head (clamp ring, hinge ears, pinch sleeve, tube socket), lever with its hook, carriage, actuator foot, stop bracket, key box, 2 key caps; PETG |
| Ordinary hardware | | | 18 mm plywood (base board 710 x 380, column 120 x 420, pan board 280 x 280, deck block 280 x 135 x 60 high); 20 x 20 x 1.5 mm aluminium square tube, 210 mm; M3 x 25 screw and locknut (hinge); M4 screws, nuts; wood screws, 2 angle brackets; springs 5 mm OD x 16 mm (lever), 2 of 10 mm OD x 12 mm (keys); 2 M3 x 8 screws, 9 mm washers; panel DC jack 5.5 x 2.1 mm; wire; 0.5 mm rubber strip; 3 mm rubber mat |

Pages: https://www.pololu.com/product/4463, /product/2572, /product/1471, /product/1402, /product/1404;
https://chemglass.com/flasks-heavy-wall-round-bottom-single-neck-100ml-to-300ml,
https://chemglass.com/ground-joints-inner-full-length, https://chemglass.com/flask-support-rings-cork;
https://www.webstaurantstore.com/fat-daddios-prd-104-proseries-10-x-4-round-anodized-aluminum-straight-sided-cake-pan/627PRD104.html;
https://laballey.com/products/economical-solid-glass-beads;
https://www.zoro.com/south-main-hardware-10-in-side-mount-stainless-steel-ball-bearing-full-extension-drawer-slides-slide-bb-10/i/G905299001/.

### Drawing

Task frame as in `task.json` (mm): x forward from the line between the arm bases, y to the left, z above the arms'
deck; the base board's top is z -60. Side view from the right (y = 0), lift up:

```
  z
 311 |                                       carriage [=]|  column: 18 mm plywood,
     |                        20 x 20 tube         /     |  front face x 600, centred
 187 |         hinge o (376, 187)             /          |  y 0; slide on its face;
 149 |  cone base * (363, 149)=ring=head====/            |  actuator (y -45) from a
 129 |   tip (328, 129), 30 deg below horizontal         |  foot on the board up to
  60 | +-------- pan rim, 279 outside ----------+        |  the carriage
     | |  PRD-104 pan, 254 across, centre x 292 |        |
 -42 | +--- on an 18 mm pan board, mat inside --+        |
 -60 +=====================================================  base board top
       152            292                    432  579 600   x
```

Lift down: everything on the carriage 95 mm lower (cone base z 54; the bulb's lowest point z -19, 18 mm above the mat,
computed).
Top view: the base board spans x 90 to 800, y -220 to 160 (the simulation's bench). On the deck block left of it:
the key box centred at x 175, y 235 (DOWN key x 145, y 257; UP key x 145, y 213; caps 16 mm across, tops z 57) and
the cork ring centred at x 320, y 220.

The clamp, in the joint's own frame (u along the axis from the cone base, up the slope; n square to it, up):

| Piece | Size and place |
|---|---|
| Ring | 42 mm across, u 6 to 26, round the glass tube |
| Hinge | 3 mm pin across the joint at u 30, n 26 |
| Lever | 12 x 5 mm bar, u -9 to 36 (36 mm from hinge to tip); shut it lies at n 23.5 to 28.5; rests open at 35 degrees on a stop |
| Thumb pad | 16 x 20 mm, u -8 to 8, on top of the lever |
| Hook | a fork 4 mm thick (u -10 to -6) shaped as a C round the flask's socket: inside 30.5 mm, mouth 29 mm, opening downward |
| Spring | 5 mm OD, in a pocket in the ring at u 22, pushing the lever open |

The hook goes down behind the bead only with the socket within 2 mm of home, and snaps round the socket there (the
mouth is 1 mm narrower than the socket); with the socket 2 to 12 mm short it lands on the bead; further out, or with no
flask, it closes on nothing and the spring opens it again (computed from the file's geometry). Once snapped, the bead
can't pass it, so the flask can't slide off.

### Build

1. Measure the flask's socket just behind its bead, the bead, and the bead's length along the axis, and set `NECK_D`,
   `BEAD_D` and `BEAD_T` in the file (30, 34 and 4 mm as simulated). Print the parts in PETG (4 perimeters, 40 %
   infill), one `part` at a time (header of the file).
2. Lay the base board on the rig's table and mark the task frame on it. Raise the arms on 60 mm risers above the
   board's top and set the deck block under the key box's and cork ring's places.
3. Stand the column at x 600 with two angle brackets. Screw the slide's fixed member upright to its front face,
   centred on y 0, 160 to 414 mm above the board, opening downward; with the lift up it stays 20 mm open.
4. Bolt the carriage to the slide's moving member (four M4 screws in its slots). Screw the actuator's foot to the
   board under the carriage's ear (y -45) and pin the actuator between the foot and the ear with M4 screws, plumb.
5. Wrap the glass tube once with the rubber strip where the head grips it, push it tube first into the ring's end of
   the head until the cone's base is 6 mm ahead of the ring, and tighten the two M4 pinch screws gently. Fit the lever on the M3 pin with its
   spring in the pocket.
6. Join the head and the carriage with the square tube (M4 grip screws in both sockets).
7. Screw the stop bracket to the column beside the slide with the switch's roller under the carriage's lower edge.
8. Set the pan board centred at x 292, y 0, the pan on it, the rubber mat on the pan's floor.
9. Fit the keys (cap, spring, stem through the lid, M3 screw and washer under it), the key switches on their tabs,
   the regulator and the DC jack in the box; wire as below; set the box at its place on the deck block.
10. Set the cork ring; add glass beads to the flask until it weighs 100 g.

### Wiring

```
 wall adapter 12 V -> DC jack -> regulator VIN, GND;  regulator VOUT = V+ (trimmed for the speed)
 UP key   (1402):  common -> actuator red     NC -> GND   NO -> V+
 DOWN key (1402):  common -> actuator black   NC -> GND   NO -> stop NC
 stop     (1404):  common -> V+               NC -> DOWN key NO   (the carriage opens it at 95 mm down)
```

Keys released, both actuator leads sit on GND and the lift stays put. UP held: the lift rises to the actuator's own
end-of-stroke switch, the top. DOWN held: the lift goes down until the carriage opens the stop; UP still works from
there. If UP lowers the lift, swap the actuator's two leads.

### Calibration

| Check | Tool | Target |
|---|---|---|
| Tilt | inclinometer app on the glass tube, or a protractor | 30 ± 1 degrees below horizontal |
| Cone base, lift up | ruler from the frame marks | x 363, y 0, z 149 (± 2 mm): slide the head along the square tube for x, then the carriage's ear slot for z |
| Travel | ruler, DOWN held to the stop | the cone base goes down 95 ± 1 mm, to z 54 (stop bracket's slots) |
| Lift speed | stopwatch, DOWN held from the top | 3.8 ± 0.2 s for the 95 mm, 25 mm/s (regulator's trimmer) |
| Keys | ruler, kitchen scale under the box | caps at their places, tops z 57; each switches at 2 mm of its 3.5 mm travel and 0.6 to 0.9 kg (switch tab's slots, springs) |
| Lever | protractor, luggage scale on the thumb pad | rests open at 35 ± 3 degrees; with no flask it pulls shut at 0.4 to 0.6 kg (about 5 N) and springs back open when let go |
| Hook | the flask held home by hand, then 3 mm short (the mouth 9 mm from the ring) | home: the hook snaps round the socket at up to 1.2 kg on the pad and the lever stays shut; 3 mm short: it stops on the bead and springs back open. Set by sliding the glass in the head (then recheck the cone base); if it needs more, reprint the lever with `SNAP = 0.6` |
| Bath | ruler | pan centred x 292, y 0; rim z 60; lift down, the bulb inside the pan below its rim |
| Flask, cork ring | kitchen scale, ruler | flask with beads 100 g; ring centred x 320, y 220 on the deck |

## Assets and licences

None from third parties. Every object is built from boxes, cylinders and spheres in the task file; `bodykit.py` and
`rotavap_fixture.scad` are our own. The NVIDIA SimReady set has no rotary evaporator, round-bottom flask or cork ring, so everything is built from
primitives at the parts' sizes.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the flask, the head and the lever still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=load_a_flask_onto_a_rotary_evaporator_and_lower_it_into_the_bath.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the flask of extract on the rotary evaporator, clamped, and lower it into the water bath, ready to run; then return both arms to where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/rv-dream
```

The scripted demo, which reaches full score: the right hand comes in from the right, in front of the evaporator, and
takes the flask by its neck just above the bulb, from the side (fingers closing front and back). It lifts the flask out
of the cork ring and carries it over to the bath, rolling its wrist 60 degrees on the way so that the neck comes to lie
along the vapour tube's slope, the mouth below the cone. It brings the socket in line with the cone 50 mm out, slides
it straight up the cone along the axis (aiming 10 mm past home for the arm's lag) and keeps it pressed home. The left
hand comes down on the clamp's thumb pad with its closed fingers square to the joint (the gripper tilted 30 degrees)
and presses the lever down past its click, then lifts off. The right hand opens and draws back off the neck; the
clamp holds the flask. The left hand presses the lift's DOWN key and holds it for 5 s while the lift takes the flask
95 mm down into the bath, to its stop, then lets go. Both arms go back to where they started. It runs with the
command above and `--policy yam_scripted --log-dir ~/yam-eval/rv-demo`.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/rv-dream/*.json; do inspect-robots video "$log"; done
```

`RV_DEBUG=1` prints what the score is built from four times a second (socket offset and distance from home, catch,
lever angle and click, lift, keys, TCPs). `RV_ONEHAND=left` (or `right`) replays only that arm's part of the demo;
`RV_CHECK=noclip`, `noseat`, `short`, `clipfirst` and `nolower` run the checks under "Checks". All of these are
checks, not part of the task.

## Runtime per trial

On one RTX 5090: about 10 minutes for a Dream-1 trial of 1,600 steps (596 s on the clock, with Dream answering in 0.72 s on average), and about 5 minutes for the scripted demo (754 steps, 37.7 s of simulated time, 318 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

The review (TK-38GRFR), verbatim:

> Flags: Note for the team: Protect the shared rig and work surface against accidental spills of the tinted water.
>
> Feasibility (not met): A motor-lift Hei-VAP rotary evaporator costs well over a few hundred dollars, and borrowing one
> is not a reliably available low-cost setup.
>
> What to change: Please provide a reproducible low-cost real-trial alternative to the motor-lift rotary evaporator;
> borrowing a working instrument is not a reliable provision for repeated trials. A fixture should retain the real
> flask, compatible ground joint and clamp, the tilted insertion, and a functional lift, with part numbers or build
> files and instructions sufficient to reproduce it.

- **Low-cost, reproducible alternative, nothing borrowed:** the real trial runs on a fixture of catalogue and printed
  parts, US$263.89 for the named parts plus ordinary hardware; the "borrow" sentence is gone (a lab's own motor-lift
  evaporator set the same way is named only as an equivalent). Where: `task.json` setup; README "Real trial",
  "Real-trial fixture". Check: the parts table, each price from its linked page.
- **Real flask:** Chemglass CG-1506-05, 100 mL heavy-wall round-bottom, NS 24/40 (setup; Parts).
- **Compatible ground joint:** a real NS 24/40 cone, Chemglass CG-100-06, held in the printed head (setup; Parts;
  `rotavap_fixture.scad`).
- **Compatible clamp:** printed with the EASY CLIP's ring, 36 mm lever (open 35 degrees) and hook; the hook is a C
  that snaps round the socket behind its bead only with the socket within 2 mm of home and lands on the bead from 2
  to 12 mm short (computed from the file's geometry). Where: setup; README "Drawing"; the file's `lever`. Check: the
  "Hook" calibration (home: snaps and stays shut; 3 mm short: springs back open).
- **Tilted insertion:** the cone at 30 degrees below horizontal, its base at x 363, z 149 with the lift up, as
  simulated. Check: inclinometer app or protractor, ruler.
- **Functional lift:** a drawer slide driven by a 12 V actuator with built-in end-of-stroke switches (Pololu 4463),
  an immersion-stop switch at 95 mm, speed trimmed to 25 mm/s, DOWN and UP keys at the simulated places reversing it.
  Where: setup; README "Wiring", "Calibration". Check: ruler (95 ± 1 mm), stopwatch (3.8 ± 0.2 s).
- **Part numbers, build files, instructions:** parts table with numbers, prices and pages; `rotavap_fixture.scad`
  (every dimension a named parameter, the parts' places in its header); drawing; build steps; wiring; calibration
  with a ruler, protractor or inclinometer app, stopwatch, kitchen and luggage scales.
- **Flag, spills of tinted water:** no liquid anywhere. The flask holds dry 4 mm glass beads (Lab Alley 11-312B) to
  100 g, the simulated flask's 0.10 kg; the pan is dry, a 3 mm rubber mat on its floor. Where: setup, fidelity; README
  "Real trial".
- **Simulation unchanged:** the fixture reproduces the simulated sizes and places, so the task file, the demo (1.0)
  and the Dream-1 run (0.0) stand as submitted.

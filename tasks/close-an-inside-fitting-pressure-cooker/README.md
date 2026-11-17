# Close an inside-fitting pressure cooker

A 1.5 litre inside-fitting (inner-lid) aluminium pressure cooker stands on the kitchen counter with rice and water in
it. Its long black body handle points left and toward the robot. Its oval lid lies top up on the counter to the right,
with the lid handle toward the robot. The lid is 4 mm larger than the cooker's oval mouth all round, so it cannot be put
on from above: it goes in through the mouth tilted and crosswise, is turned in line inside, and is pulled up against
the underside of the rim. A steel locking loop hangs from the lid handle's end. Unless that loop is pushed in under the
body handle's end, the lid drops back into the cooker as soon as it is let go.

The robot is told: "Close the pressure cooker, ready to go on the stove."

In the scripted demo:
1. The right hand takes the lid by its handle bar, 32 mm from the vent tube, lifts it and carries it crosswise over
   the mouth, its handle at right angles to the body handle.
2. Slowly, pausing at each of eight planned poses, it tilts the lid's far end down into the mouth and under the far
   side of the rim, and turns the lid 90 degrees into line with the body handle while lowering it inside.
3. It sets the handle bar on the pivot post, the lid hanging about 22 degrees, then turns the lid up about the pivot
   until the gasket is against the rim's underside all round. The lid handle comes down to 2 mm above the body handle.
4. While the right holds the lid there, the left hand pushes the locking loop in under the body handle's end.
5. The right lets go; the loop holds the lid up against the rim. Both arms head back to where they started.
6. Half a second after both hands are clear, the end state is read: four depths, the vent tube's place, the handles'
   line and gap, the loop. Only then does the scorer pull the lid handle's end up with 1 N for 1 s, the way a person
   tests the latch. The lid stays shut, and the trial ends.

The task is one task file, [`close_an_inside_fitting_pressure_cooker.py`](close_an_inside_fitting_pressure_cooker.py),
for the `yam_arena` embodiment (its scoring section comes right after the dimensions, then the scene and the scripted
demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: getting a lid larger than the mouth through it (only tilted about 35 degrees, crosswise, and turned 90
degrees on the way in; the offline geometry check over the real shapes found about 3 mm of room at the narrowest
point), its edge hidden under the rim while it is worked in; then, in a fixed order, the bar onto the pivot, the lid
levered up against the rim's underside (which brings the lid handle down onto the body handle), and the loop pushed in
while the handles are held together. Holding one part while the other hand works is common on the board and is not
the hard part here: the constrained, partly blind insertion and the ordered closing are.

## Realism check

- **The real product.** A Hawkins Classic CL15, the 1.5 litre size of India's best-selling inside-fitting pressure
  cooker range ("ideal for 1 or 2 persons"; the same design is sold from 1.5 to 12 litres). The maker's 2022
  catalogue and product page give a 130 mm flat base and 3.25 mm aluminium. Price: ₹1,175 list (₹1,250 on the
  maker's page now), ₹1,119 on Flipkart, US$55.67 at a US Indian-grocery store; links below.
  - Weight: the maker gives 1.28 kg as the "product weight in carton". The model splits it into a 1.04 kg body, a
    0.21 kg lid and an 8 g loop (design values; a 3.25 mm aluminium shell of this shape computes to 0.60 kg before its
    handle, bracket and pivot). A real trial uses the real cooker as bought.
  - Sizes: carton 320 x 170 x 135 mm; the model is 315 mm from the body handle's end to the far side of the body.
    Modelled: a 159 mm body, the rim's top 105 mm above the counter, a 120 x 138 mm oval mouth and a 128 x 146 mm oval
    lid (design values from the maker's photos and overall sizes, the lid 4 mm larger all round). Hawkins' own 11 cm
    grid is sold as fitting any Hawkins cooker "where the wider part of the mouth is 12 cm or more", the 1.5 litre
    Classic among them; the 138 mm mouth meets that.
- **The real place.** The manual's trial run says "Close cooker (without vent weight). Place cooker on high heat", so
  the cooker is closed before it goes on the stove: here, on the kitchen counter, at full size. The body handle points
  left and toward the robot at 45 degrees, as a cook would set it down; that also keeps the right wrist's roll joint
  inside its range for the lid's 90-degree turn (with the handle pointing straight left the turn lagged 8-14 degrees).
- **Every real step.** The maker's manual for the Classic (pages 12-14) has six closing steps:
  1. "Hold lid just above cooker body. Lid handle must be at right angle to body handle."
  2. "Tilt lid into cooker and move lid away from you to wall of cooker body."
  3. "Move lid handle towards long body handle."
  4. "Lower rest of lid into cooker body by slight twist of wrist of hand holding lid handle."
  5. "Place the lid handle bar on the pivot. Centralise lid. Squeeze lid handle and body handle together."
  6. "Push the locking loop to latch securely."

  Its tips add: "centralise the lid so that the gasket is in even contact with the rim of the cooker body", and "after
  releasing locking loop, do not let go of the lid handle or the lid will fall into the cooker". All six steps are
  physical here, none done by a rule: the lid passes the mouth only by tilting and turning, the bar rests on a steel
  pivot post, the lid falls back inside if nothing holds it up, and only the pushed-in loop keeps it up once let go.
  The scoring reads the manual's own closed position: the lid centralised and up against the rim, the handles in line
  and together, the loop in.
- **The YAM's limits.** Only the lid's 10 mm handle bar is gripped (jaw 93.9 mm), 32 mm from the vent tube, where the
  manual's step-2 photo holds it: the lid and loop (0.218 kg, centre of mass 46 mm out) twist that grip by 0.03 N m,
  against 0.22-0.46 N m if held on the bakelite handle (a top-down grip holds 0.1-0.4 N m). Holding the lid up against
  the rim takes 0.10 N m about the pivot (the lid's 0.113 N m less the loop's 0.013 N m): 0.9 N at the middle of the lid
  handle, 0.6 N at its end. The way in has about 3 mm of room against the arm's 5-19 mm placement error, so the demo
  moves slowly, pauses at each pose, and aims against the arm's placement errors (2-6 mm, measured in debug runs).
- **What is simplified.**
  - The rubber gasket is rigid; a real one is squeezed onto the rim (step 5). The model's latched lid hangs up to 1 mm
    below its seat, inside the 2 mm depth window.
  - The latched lid is held only by friction where its bar rests on the pivot: a steady pull at the lid handle's end
    held up to 1.7 N, and at 1.85 N it slid the lid and pulled the loop out, so the latch-hold pull is 1 N. Taking
    moments about the pivot, the loop's pull is 0.74 N from the lid's weight and 1.9 N with the 1 N pull 157.5 mm out;
    the simulation's contact sensor read 0.70 N and 1.94 N.
  - The loop's catch is a pocket shaped as an arc round the loop's hinge, so it latches without a squeeze; a robot can
    squeeze the handles' stacked ends (50 mm, within the jaw) in one gripper, as the manual's tip 4 advises a person.
  - The rice and water are a rigid layer of the right mass.
  - The handle bar is rigid on the lid; a real one has deliberate play at the vent tube (manual, page 10). The two
    readings that place the gasket, depth and centring, are read on the lid itself, so the play does not enter them.
  - Design values: the lid handle's 110 mm length (25 mm past the body handle's end, so the hanging loop clears it),
    the pivot post's flat 10.5 mm top, the loop's 60 mm length.

## How this differs from the board

- **#1902** (accepted) rolls a shelf through a face frame narrower than the shelf: also a part through a smaller
  opening by tilting, but into an open cabinet, where it stays. Here a lid goes through a closed mouth into the
  vessel, is turned in line, hung on a pivot and levered up against the inside of the rim, and falls back unless the
  locking loop goes in.
- **#1819** (final) vents a pressure pan and takes off an ordinary outside lid, a safety-order task; **#1443** (final)
  presses a rice cooker's start button. Other lid tasks (#32, #42, #1363, #2047, #2021) put a lid on top of a vessel.
  None puts a lid in through a mouth smaller than itself and latches it from inside.

## One hand or two

Two hands are needed for a physical reason: the lid's weight turns it down about the pivot (0.10 N m), so it drops back
into the cooker as soon as it is let go, and one gripper cannot hold it up and push the loop in at the same time. A
person squeezes the handles with one hand and pushes the loop with the other, or with the thumb of the same hand.

In the one-hand check (`PC_ONEHAND=1`) the right hand does its part exactly as in the demo, holds the seated lid for
4 s and lets go, while the left never moves. The lid swings back down inside the cooker (tilted 28 degrees, the handles
67 mm apart): 0.25.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres, so everything is built from shapes with `bodykit.py`,
at the real sizes and masses given in the setup, as three loose rigid bodies and one hinge: the cooker body (1.44 kg
with the rice and water; its 3 mm lip is 72 pieces, each square to the oval mouth's edge), the lid (0.21 kg, one body
with its gasket, vent tube, valve, bar and handle; its centre of mass 38 mm out from the vent tube toward the handle,
so, released, it swings down inside about the pivot) and the loop (8 g, on a hinge at the lid handle's end). The pocket
under the body handle's end has a roof shaped as an arc round the loop's hinge, 0.3 mm clear of the crossbar, ending in
a stop; pushed in 36-48 degrees, the crossbar enters it, the lid's weight pulls it up against the roof, and friction
holds it. Seated, the lid's top face is 4.5 mm below the rim's top (D0 in the model). **Third-party assets:** none;
every shape is a primitive built in the task file and `bodykit.py`.

## Real sizes and their sources

- **Cooker.** Hawkins Classic CL15, 1.5 litres.
  - [Hawkins Pressure Cooker Catalog 2022](https://www.hawkinscookers.com/downloads/Pressure_Cooker_Catalog_2022.pdf):
    CL15, 1.5 litres, a 130 mm flat base, 3.25 mm thick, MRP ₹1,175; inside-fitting lid; "fixed sturdy pivot" that
    is "a part of the metal handle itself"; safety valve under the lid handle bar.
  - [Maker's Hawkins Classic page](https://www.hawkinscookers.com/WebPCMain.aspx?cat=HCL): CL15, 130 mm base,
    3.25 mm, "1-2 Persons", 1.28 kg "product weight in carton", carton 320 x 170 x 135 mm, MRP ₹1,250.
  - [Flipkart](https://www.flipkart.com/hawkins-classic-1-5-l-pressure-cooker/p/itm6d80ae6b2e410): "Hawkins Classic
    (CL15) 1.5 L Pressure Cooker (Aluminium)", inner lid, ₹1,119 (MRP ₹1,175).
  - [TheDesiFood (US)](https://thedesifood.com/products/hawkins-aluminium-inner-lid-pressure-cooker-bakelite-handle,-silver,-classic,-cl15):
    Hawkins Classic CL15, US$55.67, "a thick 3.25 mm base".
- **Closing steps and the closed position.** The maker's
  [Hawkins Pressure Cooker Instruction Manual with Tested Recipes](https://www.hawkinscookers.com/Cookbooks/AllIndianCookbook.pdf)
  (Hawkins Classic and Contura): page 8, the parts (the pivot on the handle bracket, the loop at the lid handle's end);
  page 10, the bar's deliberate play, which "helps in centralising the lid"; pages 12-13, the six closing steps and
  opening ("Squeeze handles together. Release locking loop."); page 14, the tips; page 15, the trial run and its
  steam-tight check ("If steam comes out around the edge of the lid, check the centralisation of the lid"). The
  [maker's "How to close & open" page](https://www.hawkinscookers.com/1.4.1.pc_close_open.html) gives the same steps.
  The cooker has no printed alignment marks and no pressure-indicator pin; its closed position, as the manual shows it,
  is the lid centralised and up against the rim, the lid handle on the body handle in line with it, and the loop over
  the handles' end. The scoring reads exactly those.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s (the pull passed, with the cooker, the lid
and the loop still). Otherwise it gives `score()` at the end of the episode: 0, 0.25, 0.5 or 1.0.

### What is read, and when

Every reading is taken on the run's **end state**, before anyone touches the cooker, the lid, the loop or the robot,
and no reading needs the cooker opened. The run ends once the robot has let go with the loop hooked under the body
handle's end and kept both hands off for 0.5 s (no part of either hand touching the cooker, the lid or the loop; each
gripper at least 5 cm from the lid's handle bar and handle and from the loop), or else at the run limit, with the robot
stopped where it is (still holding the lid, if it holds it). The simulation reads the same quantities at the same
places, in the same order (`_readings`, then `_pull`, in the task file).

| Check | Simulation | Real trial (end state, nothing touched) |
|---|---|---|
| **Handles together** | The lid handle's underside is within 4 mm of the body handle's top, 6 mm in from the body handle's end (2 mm when seated). | Side camera: a phone on a small stand, square to the handles' end at their height, a steel rule standing on the counter behind it. One frame. |
| **In line** | The middle of the lid handle's end is within 13 mm sideways of the body handle's centre line (2.9 degrees of turn). | A photo from directly above (13 mm is half the body handle's 26 mm width). |
| **Centred** | The vent tube's axis, where it meets the lid's face, is within 2.5 mm of the mouth's middle. | The same top photo: the vent tube's centre against the mouth's edges, the mouth's own 120 x 138 mm as the scale. |
| **Depth** | At four marks 5 mm in from the mouth's edge, on the rays 45 degrees either side of the body handle and of the far side, the lid's top face is within 2 mm of D0 = 4.5 mm below the rim's top. | A caliper rested on the rim's top at each of four tape marks, its depth rod lowered onto the lid: each reading within 2 mm of that mark's D0. A photo per reading. |
| **Latched** | The loop is swung in 34-50 degrees, its crossbar bearing on the cooker in the pocket under the body handle's end. | The side-camera frame: the loop over the handles' end, its crossbar hooked under the body handle's end. |
| **Hands off** | As defined for the end state. | Trial video. |
| **Latch holds** (last) | On a seated, latched lid with the hands off, the scorer pulls the lid handle straight up with 1 N, 10 mm in from its end, for 1 s; throughout, the handles' gap stays within 4 mm, the loop stays hooked and the hands stay off. | A spring scale hooked under the lid handle 10 mm from its end, pulled straight up to 100 g (1 N) and held 1 s, filmed by the side camera: the gap stays within 4 mm on the rule and the crossbar stays hooked. |

Seated means the first four checks. **D0** is a calibration, made once per cooker before its first trial and never
during or after one: close the cooker by hand as the manual says (lid in, bar on the pivot, lid centralised, handles
squeezed, loop pushed in) and read the depth at each tape mark. It stays with that cooker.

**Tiers.** 1.0: seated and latched with the hands off at the end state, and the latch held the pull. 0.5: seated at the
end state without that (for example the robot still holding the lid up at the run limit, or a failed pull). 0.25: the
lid was at some time wholly inside the cooker, below the rim (real trial: the video). Otherwise 0.0, and 0.0 for good if
the cooker tips more than 25 degrees, or the cooker or the lid falls off the counter, before the trial ends. An
unlatched lid swings back into the cooker as soon as it is let go, so it cannot be seated with the hands off.

### Why these readings show the gasket seated

The lid is a rigid plate, and nothing in the cooker lets it rise above its seat. So:
- **Depth.** The four readings fix the plane of the lid's face. Each within 2 mm of D0 puts every point of the gasket
  within 2.5 mm of the rim's underside (computed over every tilt direction).
- **A cocked lid shows in the four readings.** Resting cocked on part of its gasket, a lid reads D0 on its high side
  and deeper on its low side; the marks are 83 mm apart across each pair. Swept through 24 tilt directions with the
  scorer's own readings, any cock over 1.1 degrees puts a mark more than 2 mm deep. In a simulation run the robot held
  the seated lid (depths 5.7/5.1/4.6/5.2 mm, gap 3.0 mm) and then cocked it: at the first trace with the lid tilted 1.1
  degrees it read 7.1/5.5/5.2/6.8 mm with a 4.6 mm gap, no longer seated, and held cocked to the run limit (6 degrees
  once the arm settled) it scored 0.25.
- **Centred and in line.** Together they keep the gasket's outer edge at least 1.0 mm in under the rim all round
  (computed; seated, 4 mm). A lid slid 2.6 mm off centre fails.
- **Nothing the tester does can make a reading.** The photos and the side camera touch nothing. The caliper touches
  only the rim's top and, with its rod, the lid's face; the rod can only push the lid down, away from the rim, so it
  can fail a reading but never create one. The pull comes after every reading, and nothing is read after it.

### The real trial, step by step

Kit, beyond the cooker: a 150 mm digital caliper with a depth rod (about US$20), a hanging spring or luggage scale
reading to 10 g (about US$10), a phone and a small phone stand (about US$10), a 150 mm steel rule, masking tape.

Once per cooker: tape the four depth marks on the outside of the body just below the rim, 45 degrees either side of the
body handle and of the far side, and read D0 at each on the cooker closed by hand. Each session: 95 g of uncooked white
rice and 300 ml of room-temperature tap water in the cooker (0.40 kg, about 21 mm deep), thrown away at the end; the
cooker is never heated. Before each trial: the lid at its marks on the counter, its handle toward the robot, the loop
folded out.

At the end state, with the robot stopped, in this order:
1. Set the phone on its stand square to the handles' end, the rule standing behind it, and record. This frame gives
   the gap and the loop. The stand and the rule touch only the counter.
2. A photo from directly above: in line and centred.
3. The four depth readings, a photo of each.
4. Only on a lid latched with the hands off: the pull with the spring scale, on the side camera.
5. Open the cooker as the manual says (the tester may now take the lid handle; nothing more is read), and reset.

### Commands

`-E object_jitter=0` keeps every object where the task file puts it. The cameras render at 1280 x 704 for sharp
videos; Dream-1's client scales every frame to 320 x 176 before sending it. The scripted demo is the same command with
`--policy yam_scripted`.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=close_an_inside_fitting_pressure_cooker.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Close the pressure cooker, ready to go on the stove." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/cooker-dream
```

The scorer always prints the end state's readings and the pull's result. `PC_DEBUG=1` adds a trace four times a
second (including, never scored, the gasket's distance from the rim); `PC_LIFT_RAMP=0.5` ramps the pull at 0.5 N/s to
find what the latch holds; `PC_ONEHAND=1` runs the one-hand check.

## Real-trial safety and rig protection

- The trial is cold: the cooker never goes on a stove, holds no pressure, and its vent weight is left off.
- The grippers touch only the lid's steel handle bar, its bakelite handle and the steel loop, all clean and dry.
- The rice and water stay inside the cooker. A cooker that tips more than 25 degrees ends the trial at 0, and the
  counter can carry a tray to catch a spill.
- The lid weighs 0.21 kg and the pull is 1 N. The mouth's edge is rolled and the loop is round wire.

## Runtime per trial

On one RTX 5090: about 15 minutes for a Dream-1 trial of 1,800 steps (904 s on the clock, with Dream answering in 0.83 s
on average), and about 10 minutes for the scripted demo (1,298 steps, 64.9 s of simulated time, 576 s on the clock).
Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-LPW36Z. Its review said:

> **Not accepted** (TK-LPW36Z).
>
> Scoring (not met): Squeezing the handles to take over or open the cooker can create the gasket witness ring after the scored state, obscuring whether the robot seated it.
>
> What to change: The earlier seating-check issue remains open: squeezing the handles to take over a held lid or open a latched cooker can create a witness ring after the trial. Please use a repeatable check that establishes gasket seating at the actual end state without tester handling creating or altering the evidence, for both the held-lid partial score and the latched full score; retain the practical latch-hold test.

The review before that (TK-JGRP2P) asked to replace the hidden all-round gasket photo and feeler gauge with checks made
on the closed cooker, for a held and a latched lid, and to keep a practical latch-hold test. That was done in the
previous revision; its in-line, handles-together, depth and latch-hold checks are kept here.

What changed in response:
- **Scoring (not met), evidence made or altered by handling.** The witness ring, the cornflour paste and the hex key
  are gone (scoring, setup, reset_notes, scorer). Seating is read only on the end state, before anyone touches the
  cooker, the lid, the loop or the robot, without opening it: the gap from a side-camera frame against a steel rule, in
  line and centred from a top photo, and the depth at four tape marks with a caliper resting on the rim, whose rod can
  only push the lid down ("What is read, and when"). Demo: depths 5.4/4.9/4.7/5.1 mm against D0 = 4.5 mm, vent tube 1.9
  mm off centre, in line 4.0 mm, gap 2.8 mm: 1.0.
- **Held-lid partial score (0.5).** Read the same way with the robot stopped, still holding the lid; the tester takes
  the lid only after every reading ("The real trial, step by step"). The scorer reads the same quantities on the last
  step. Check: the lid seated and held to the run limit, never latched: depths 5.8/5.1/4.6/5.2 mm, gap 3.1 mm, 0.5.
- **Latched full score (1.0).** The end state is the same in both: the loop hooked and both hands off for 0.5 s. The
  readings come then and the pull after them; the scorer keeps that order (`_follow_inner` records the end state, then
  `_pull`), and a failed pull leaves 0.5. Check: the demo, 1.0.
- **Latch-hold test kept, and last.** 100 g (1 N) under the lid handle 10 mm from its end for 1 s, on the side camera:
  the gap stays within 4 mm and the crossbar hooked. The scorer checks exactly those during its pull (gap, loop hooked,
  hands off). Check: the demo's pull held, the gap at most 2.9 mm.
- **D0 is a one-time calibration** per cooker, read before its first trial on the cooker closed by hand (setup,
  scoring).
- **A cocked lid shows in the four readings** (D0 on its high side, deeper on its low side): over 24 tilt directions
  of the scorer's own readings, any cock over 1.1 degrees fails. Check: the seated lid held and then cocked fails from
  the first trace at 1.1 degrees of tilt (depths 7.1/5.5/5.2/6.8 mm, gap 4.6 mm); held cocked to the run limit, 0.25.
- **Centring, the manual's "centralise", is a new check:** the vent tube within 2.5 mm of the mouth's middle (top
  photo). With in line it keeps the gasket at least 1.0 mm in under the rim all round (computed); a 2.6 mm slide fails.
- **Seated but not latched:** held, 0.5 (above); let go, the lid falls back to 28 degrees: 0.25 (the one-hand check).
- **Hands off tightened:** no part of either hand (fingers, gripper, wrist) may touch the cooker, the lid or the loop,
  and each gripper must be 5 cm from the whole bar and handle and from the loop (it was the finger pads, and 5 cm from
  the handle's end).
- **The simulation reads exactly the real quantities:** "seated" no longer needs a contact force the real trial cannot
  see, and the depth marks sit on the exact 45-degree rays of the tape marks.

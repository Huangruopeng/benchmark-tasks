# Close an inside-fitting pressure cooker

A 1.5 litre inside-fitting (inner-lid) aluminium pressure cooker stands on the kitchen counter with rice and water in
it. Its long black body handle points left and toward the robot. Its oval lid lies top up on the counter to the right,
with the lid handle toward the robot. The lid is 4 mm larger than the cooker's oval mouth all round, so it cannot be put
on from above: it goes in through the mouth tilted and crosswise, is turned in line inside, and is pulled up against
the underside of the rim. A steel locking loop hangs from the lid handle's end. Unless that loop is pushed in under the
body handle's end, the lid drops back into the cooker as soon as it is let go.

The robot is told: "Close the pressure cooker, ready to go on the stove."

In the scripted demo:
1. The right hand takes the lid by its handle bar, 32 mm from the vent tube, and lifts it off the counter. It carries
   the lid crosswise over the mouth, its handle at right angles to the body handle.
2. Slowly, pausing at each of eight planned poses, it tilts the lid's far end down into the mouth and under the far
   side of the rim. It turns the lid 90 degrees into line with the body handle while lowering it inside.
3. It sets the handle bar on the pivot post with the lid hanging about 22 degrees, lid side down.
4. Keeping the bar on the pivot, it turns the lid up about the pivot until the gasket is against the rim's underside
   all round. This brings the lid handle down to 2 mm above the body handle.
5. While the right holds the lid there, the left hand pushes the locking loop in under the body handle's end.
6. The right lets go. The loop now holds the lid up against the rim. Both arms head back to where they started.
7. Half a second after both hands are clear, the scorer tests the latch the way a person would: it lifts the lid
   handle's end with 1 N for 1 s. The lid stays shut. The trial ends once the full score has then held for 1 s with
   the lid, the loop and the cooker still.

The task is one task file, [`close_an_inside_fitting_pressure_cooker.py`](close_an_inside_fitting_pressure_cooker.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests:
- Getting a lid that is larger than the mouth through it. The lid only passes tilted about 35 degrees, crosswise,
  and turned 90 degrees on the way in. The offline geometry check over the real shapes found about 3 mm of room at
  the narrowest point.
- Setting the lid's handle bar on its pivot with the lid hanging inside.
- Turning the lid up about the pivot until it is against the rim's underside, which brings the lid handle down onto the
  body handle (the lever the maker designed).
- Pushing the locking loop in under the body handle's end while the handles are held together. Let go before the
  loop is in, and the lid drops back into the cooker.

Holding one part while the other hand works on it is common on the board. Here that step is not the hard part. The
hard part is the constrained, partly blind insertion: a lid larger than the hole has to pass through it with about
3 mm of room. The lid's edge is hidden under the rim while it is worked in. And the steps come in a fixed order:
through, turned in line, onto the pivot, up against the rim, latched.

## Realism check

- **The real product.** A Hawkins Classic CL15, the 1.5 litre size of India's best-selling inside-fitting pressure
  cooker range ("ideal for 1 or 2 persons"; Hawkins sells the same design from 1.5 to 12 litres).
  - The maker's 2022 catalogue and its product page give a 130 mm flat base and 3.25 mm aluminium.
  - Price: ₹1,175 list in the 2022 catalogue (₹1,250 on the maker's page now). Flipkart sells it for ₹1,119, and a US
    Indian-grocery store lists it at US$55.67. Links are under "Real sizes and their sources".
  - Weight: the maker gives 1.28 kg, but as the "product weight in carton", and no one figure for the cooker itself
    was found. The simulation uses 1.26 kg in all: a 1.04 kg body, a 0.21 kg lid and an 8 g loop.
    - That split came from treating the 1.28 kg as the cooker's own weight. So the body is probably modelled too
      heavy.
    - A 3.25 mm aluminium shell of this shape is 0.60 kg (at 2.70 g/cm³). With an estimated 0.12 kg of handle,
      bracket and pivot, the real body may be nearer 0.72 kg.
    - We have not weighed one. A real trial uses the real cooker, whatever it weighs.
  - Its carton is 320 x 170 x 135 mm. The model is 315 mm long from the body handle's end to the far side of the body.
  - Modelled: a 159 mm body, the rim's top 105 mm above the counter, an oval mouth of 120 x 138 mm, and an oval lid
    of 128 x 146 mm. The lid and mouth sizes are our estimates: no maker figure was found, so we gave the lid a
    4 mm overlap all round. Retail listings for Hawkins' 11 cm grid say it fits any Hawkins cooker "where the wider
    part of the mouth is 12 cm or more", the 1.5 litre Classic among them. That is only a lower bound, and our 138 mm
    meets it.
- **The real place and height.** The manual's trial run says "Close cooker (without vent weight). Place cooker on high
  heat", so the cooker is closed before it goes on the stove. Here it is closed on the kitchen counter (the station
  table) at full size. The body handle points left and toward the robot at 45 degrees, as a cook would set it down.
  That angle also keeps the right wrist's roll joint inside its range for the lid's 90-degree turn: with the handle
  pointing straight left, the lid's turn lagged 8-14 degrees at the joint's limit.
- **Every real step.** The maker's manual for the Classic ("Hawkins Pressure Cooker Instruction Manual with Tested
  Recipes", pages 12-14) has six closing steps:
  1. "Hold lid just above cooker body. Lid handle must be at right angle to body handle."
  2. "Tilt lid into cooker and move lid away from you to wall of cooker body."
  3. "Move lid handle towards long body handle."
  4. "Lower rest of lid into cooker body by slight twist of wrist of hand holding lid handle."
  5. "Place the lid handle bar on the pivot. Centralise lid. Squeeze lid handle and body handle together."
  6. "Push the locking loop to latch securely."

  Its tips add: "Before latching the handles together, centralise the lid so that the gasket is in even contact with
  the rim of the cooker body", and "after releasing locking loop, do not let go of the lid handle or the lid will fall
  into the cooker". All six steps are physical here, and none is done by a rule:
  - the lid passes the mouth only by tilting and turning;
  - the handle bar rests on a steel pivot post on the body handle's bracket;
  - the lid falls back inside if nothing holds it up;
  - only the pushed-in loop keeps it up once it is let go.
- **The YAM's limits.**
  - Only the lid's 10 mm handle bar is gripped. The jaw opens to 93.9 mm, and the 159 mm body is never gripped.
  - The hand holds the bar 32 mm from the vent tube, where the manual's step-2 photo holds the lid. The lid and loop
    (0.218 kg, centre of mass 46 mm out from the vent tube) then twist the grip by about 0.03 N m. Held anywhere on
    its bakelite handle it would be 0.22-0.46 N m, as much as or more than a top-down grip holds (0.1-0.4 N m).
  - All the work is at TOP+0.07 to 0.16 m.
  - Holding the lid up against the rim takes 0.10 N m about the pivot (the lid's 0.113 N m less the loop's 0.013 N m):
    0.9 N at the middle of the lid handle, 0.6 N at its end.
  - The loop weighs 8 g.
  - The two hands work about 20 cm apart.
  - The lid's way in is tight: about 3 mm of room against the arm's 5-19 mm placement error. The scripted demo only
    gets through because it moves slowly, pauses at each pose, and aims against the arm's measured placement errors
    (2-6 mm, measured pose by pose in debug runs).
- **What is simplified.**
  - The rubber gasket is rigid. In a real cooker the handles are squeezed together (the manual's step 5), which
    presses the gasket onto the rim. Once the loop is in, the squeezed gasket holds the lid there, and its grip on the
    rim keeps the lid from sliding toward the far wall (the manual's opening steps push it that way).
    - In the sim nothing is squeezed. The latched lid hangs up to about 1 mm below the rim, so "within 2 mm of the
      rim's underside all round" stands for the witness ring.
    - The latched lid is held in place only by friction where its bar rests on the pivot. In a test, a steady lift
      at the lid handle's end held up to 1.7 N, and at 1.85 N it slid the lid and pulled the loop out. A real
      latched cooker holds far more. So the scorer's latch-hold lift is 1 N.
    - Taking moments about the pivot, the loop's pull should be 0.74 N from the lid's weight alone, and 1.9 N with
      the 1 N lift 157.5 mm out. The simulation's contact sensor read 0.70 N and 1.94 N.
    - The real squeeze force is not published, and we have not measured it. A robot that finds the loop will not go
      in can squeeze the handles' stacked ends (50 mm, within the 93.9 mm jaw) in one gripper, as the manual's tip 4
      advises a person to do.
  - The loop's catch under the body handle is drawn as a pocket shaped as an arc round the loop's hinge, so it
    latches without that squeeze.
  - The rice and water are a rigid layer of the right mass.
  - The handle bar is rigid on the lid. A real one has deliberate play at the vent tube (manual, page 10).
  - Our estimates, not maker figures:
    - the lid handle's length (110 mm, reaching 25 mm past the body handle's end so the hanging loop clears it);
    - the pivot post's flat 10.5 mm top;
    - the loop's 60 mm length.

## How this differs from the board

The nearest tasks on the board are:
- #1902 (accepted): roll a shelf through a face frame narrower than the shelf;
- #1819 (final): vent a pressure pan, then take its lid off;
- #1443 (final): start a rice cooker with a covered steam vent.

- **#1902** also gets a part through an opening smaller than itself by tilting it. Its shelf goes into an open
  cabinet and stays put. Here a lid goes through a closed mouth into the vessel. Then it has to be turned in line,
  hung on a pivot and levered up against the inside of the rim, and it falls back unless the locking loop goes in.
  The verb is "close" and the object is a pressure vessel.
- **#1819** opens a pressure pan: it releases the steam, then takes off an ordinary outside lid. It is a safety-order
  task. Here an inside-fitting lid is put in and latched shut.
- **#1443** presses a rice cooker's start button. No lid is fitted.

Other lid tasks on the board are a lid on top of a vessel: #32 snap-fit case lid, #42 two-latch lid, #1363 and #2047
pan lids, #2021 food-processor lid. None puts a lid in through a mouth smaller than itself and latches it from inside.

## One hand or two

Two hands are needed for a physical reason. The lid only stays up against the rim while something holds it there:
the lid's weight turns it down about the pivot (0.10 N m), so it drops back into the cooker as soon as it is let go.
One gripper cannot hold the lid up and push the loop in at the same time. A person squeezes the handles with one hand
and pushes the loop with the other, or with the thumb of the same hand.

In the one-hand check, the right hand did its part exactly as in the demo, with the lid in, on the pivot and seated
against the rim. It held the lid there for 4 s and then let go, while the left never moved. The lid swung back down
inside the cooker (tilted 28 degrees, the handles 67 mm apart). Under the revised scorer that trial scored 0.25.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. Everything here is built from shapes with
`bodykit.py`, at real size and mass, as three loose rigid bodies and one hinge.
- **The cooker body** (1.44 kg: 1.04 kg of body plus 0.40 kg of rice and water) stands loose on the counter.
  - It has a 130 mm flat base, a rounded bottom edge and a 159 mm straight wall 3.2 mm thick.
  - At the top, a 3 mm lip runs inward to the 120 x 138 mm oval mouth. The lip is 72 pieces, each square to the
    oval's edge.
  - The body handle is black bakelite, 130 x 26 x 28 mm, on a steel bracket. It points left and toward the robot at
    45 degrees.
  - A 6 x 10.5 mm steel pivot post stands on the bracket 93 mm from the cooker's axis. Its flat top is 116.5 mm
    above the counter.
  - Under the handle's end, a pocket is cut for the loop's crossbar. Its roof is an arc round the loop's hinge, 0.3 mm
    clear of the crossbar, and it ends in a stop.
- **The lid** (0.21 kg) is one body.
  - An oval aluminium plate, 128 x 146 x 2.5 mm, edged with a 5 mm black rubber gasket.
  - A vent tube in the middle, and a safety valve under the bar.
  - A steel handle bar, 10 x 8 mm, from the vent tube out over the rim.
  - A bakelite lid handle, 110 x 22 x 20 mm. Closed, its end reaches 25 mm past the body handle's end, so the
    loop hanging from it clears the body handle while the lid is turned up.
  - Seated, the gasket's top is against the lip's underside, the bar lies on the pivot, and the lid handle is 2 mm
    above the body handle. The lid's top face is then 4.5 mm below the rim's top.
  - The lid's centre of mass is 38 mm out from the vent tube toward the handle. Released, the lid therefore swings
    down inside the cooker about the pivot (0.10 N m, the loop included).
- **The locking loop** (8 g) is a steel wire loop: 3.5 mm wire, 44 mm wide, hanging 60 mm from a hinge at the lid
  handle's end.
  - Its legs straddle both handles.
  - Pushed in 36-48 degrees, its crossbar goes into the pocket under the body handle.
  - There the lid's weight pulls it up against the pocket's roof, and friction holds it.
  - On the counter it lies folded out.
- **Third-party assets:** none. Every shape is a primitive built in the task file and `bodykit.py`.

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
- **Closing steps, marks and checks.** The maker's manual for the Classic:
  [Hawkins Pressure Cooker Instruction Manual with Tested Recipes](https://www.hawkinscookers.com/Cookbooks/AllIndianCookbook.pdf)
  (for the Hawkins Classic and Contura models).
  - Page 8: the parts, with the pivot on the metal handle bracket and the locking loop at the lid handle's end.
  - Page 10: the lid handle bar's deliberate play.
  - Pages 12-13: the six closing steps quoted above, and opening ("Squeeze handles together. Release locking loop.").
  - Page 14: the tips (minimum tilt; centralise the lid "so that the gasket is in even contact with the rim"; do not
    let go of the lid handle; squeeze the handles at the end away from the body).
  - Page 15: the trial run ("Close cooker (without vent weight). Place cooker on high heat.") and its steam-tight
    check ("If steam comes out around the edge of the lid, check the centralisation of the lid").
  - [The maker's "How to close & open" page](https://www.hawkinscookers.com/1.4.1.pc_close_open.html) gives the
    same steps and tips for the Hawkins Classic, calling the gasket the "sealing ring".
  - The cooker has no printed alignment marks and no pressure-indicator pin. Its closed position, as the manual
    shows it, is the lid handle lying on the body handle in line with it, squeezed together, with the loop pushed in
    over the handles' end and the lid centralised. The scoring below checks exactly those.
- **Our estimates.** These come from the overall sizes and the maker's photos:
  - the mouth (120 x 138 mm) and the lid (128 x 146 mm);
  - the rim height (105 mm);
  - the handle bar (10 x 8 mm, 150 mm) and the lid handle (110 x 22 x 20 mm);
  - the loop (3.5 mm wire, 44 x 60 mm, 8 g);
  - the split of the weight into a 1.04 kg body and a 0.21 kg lid. As noted above, the body is probably too
    heavy.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: the full score, with the cooker, the lid and
the loop still. Otherwise it gives `score()` at the end of the episode: 0, 0.25, 0.5 or 1.0.

### What is scored, in the simulation and in a real trial

Every check below can be made on the closed cooker, whether the robot is still holding the lid or has latched it,
except the witness ring, which is read off the gasket after opening. The simulation applies the same rule to the
same place; the real instrument is in the right-hand column.

| Check | Simulation (`score()`) | Real trial |
|---|---|---|
| **Seated: in line** | The middle of the lid handle's end is within 13 mm sideways of the body handle's centre line (2.9 degrees of lid turn). | Photo from above: the same, against the body handle's centre line (13 mm is half its 26 mm width). |
| **Seated: handles together** | The lid handle's underside is within 4 mm of the body handle's top at the body handle's end (2 mm when seated). | The short arm of a 4 mm hex key cannot be pushed in between the two handles at the body handle's end. |
| **Seated: depth** | At four points 5 mm in from the mouth's edge, on its diagonals, the lid's top face is 4.5 +/- 2 mm below the rim's top (4.5 mm is the modelled seated depth). | A caliper's depth rod at the same four points, marked on the rim: each reading within 2 mm of D0, the depth read there once with the cooker closed by hand as the manual says. One photo per reading. |
| **Seated: witness ring** | At 24 points round the gasket's outer edge (18 mm apart), the gasket is at least 1 mm in under the rim and within 2 mm of its underside; and the lid touches the cooker. | Before the trial a thin film of cornflour-and-water paste is wiped round the underside of the rim's lip. After opening, the black gasket must carry an unbroken white ring, at least 1 mm wide, all round. Photo. |
| **Latched** | The locking loop is swung in 34-50 degrees from hanging, and its crossbar touches the cooker in the pocket under the body handle's end. | Photo from the side: the loop lies over the handles' end, its crossbar hooked under the body handle's end. |
| **Hands off** | No finger pad touches the lid, the loop or the cooker, and each gripper is at least 5 cm from the lid handle's end and from the loop. | Video: the robot has let go and moved away. |
| **Latch holds** | 0.5 s after the hands are off a seated, latched lid, the scorer pulls the lid handle straight up with 1 N, 10 mm in from its end, for 1 s. Every check above must hold throughout. | A spring scale hooked under the lid handle 10 mm from its end is pulled straight up until it reads 100 g (1 N) and held 1 s: the lid handle does not rise (the 4 mm hex key still cannot go in). Video. |

**Tiers.**
- 1.0: seated (all four checks), latched, hands off, and the latch held the lift.
- 0.5: seated, for example with the robot still holding the lid up.
- 0.25: the lid was at some time wholly inside the cooker, below the rim. Real trial: from the video.
- Otherwise 0.0.

**Failures: 0.0 for good, at any time.** The cooker tips more than 25 degrees, or the cooker or the lid falls off the
counter.

A lid that is not latched swings back down into the cooker as soon as it is let go, so it cannot stay seated with the
hands off. The witness ring shows that the gasket met the rim all round; the other three seated checks show that the
lid is still there at the end. A lid that was seated and then fell back fails them.

### The real trial, step by step

Kit, beyond the cooker: a 150 mm digital caliper with a depth rod (about US$20), a 4 mm hex key (about US$1), a
hanging spring or luggage scale reading to 10 g (about US$10), a teaspoon of cornflour, and masking tape.

Once per cooker:
1. Put four small pieces of masking tape on the outside of the body, just below the rim, on the mouth's diagonals
   (45 degrees either side of the body handle and of the far side). These are the depth marks.
2. Close the cooker by hand as the manual says: lid in, bar on the pivot, lid centralised, handles squeezed, loop
   pushed in. At each mark, rest the caliper's end on the rim's top and drop its depth rod onto the lid 5 mm in from
   the mouth's edge. The four readings are D0 (the model's is 4.5 mm). The real gasket is squeezed in this state,
   so a robot holding the lid up must also hold the handles together, as the manual's step 5 does.

Each session: put 95 g of uncooked white rice and 300 ml of room-temperature tap water in the cooker (0.40 kg, about
21 mm deep), as for a small batch of rice. The cooker is never heated. Throw it away at the end of the session.

Before each trial: stir a teaspoon of cornflour into a teaspoon of water and wipe a thin film round the underside of
the rim's lip with a fingertip. Lay the lid at its marks on the counter.

At the end of the trial, in this order:
1. Photos from above and from the side: in line, the handles together (the hex key), and where the loop is.
2. The four depth readings, one photo each.
3. If the robot has let go: the lift with the spring scale, on video.
4. Open the cooker as the manual says. If the robot still holds the lid, the tester first takes the lid handle and
   squeezes the handles together, and then the robot's gripper is opened. Photograph the gasket's top face for the
   witness ring.
5. Reset: wipe the gasket and the rim clean.

### Commands

`-E object_jitter=0` keeps every object exactly where the task file puts it. The cameras render at 1280 x 704 for sharp
videos; Dream-1's client scales every frame to 320 x 176 before sending it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=close_an_inside_fitting_pressure_cooker.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Close the pressure cooker, ready to go on the stove." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/cooker-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=close_an_inside_fitting_pressure_cooker.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Close the pressure cooker, ready to go on the stove." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/cooker-demo
```

Set `PC_DEBUG=1` to print what the score is built from, four times a second:
- the lid's place and turn in the cooker's frame;
- the four seated checks: the ring count (24 points), the four depths, the alignment and the handles' gap;
- the contact forces of the lid and the loop on the cooker;
- the loop's swing and the handles' gap;
- the finger pads' forces;
- how far the lid has turned in the hand;
- the latch-hold lift: whether it is on, and how many tests passed or failed.

`PC_LIFT_RAMP=0.5` (debug only) ramps the lift at 0.5 N/s instead, to find what the latch holds.

`PC_ONEHAND=1` runs the one-hand check.

## Real-trial safety and rig protection

- The trial is cold. The cooker never goes on a stove, holds no pressure, and its vent weight is left off.
- The grippers touch only the lid's steel handle bar, its bakelite handle and the steel loop, all clean and dry.
- The cornflour paste is on the underside of the rim's lip, inside the cooker, where the fingers cannot reach. It is
  food, and it washes off with water.
- The rice and water stay inside the cooker. A cooker that tips more than 25 degrees ends the trial at 0, and the
  counter can carry a tray to catch a spill.
- The robot pushes and lifts no more than the lid's own handling needs: the lid weighs 0.21 kg, and the scorer's
  lift is 1 N.
- There are no sharp edges: the mouth's edge is rolled, and the loop is round wire.

## Runtime per trial

On one RTX 5090: about 12 minutes for a Dream-1 trial of 1,800 steps (704 s on the clock, with Dream answering in 0.72 s on average), and about 9 minutes for the scripted demo (1,258 steps, 62.9 s of simulated time, 565 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-JGRP2P. Its review said:

> **Not accepted** (TK-JGRP2P).
>
> Scoring (not met): The gasket-to-rim interface is concealed inside the closed opaque cooker, so the specified all-around photo and feeler-gauge check cannot be performed as written.
>
> What to change: Please replace the all-around gasket photo and feeler-gauge check with a repeatable test that can actually be performed on the closed cooker. Specify how to verify seating both when the lid is held for partial credit and after it is latched, for example using observable alignment or witness marks that can be inspected after opening, and retain a practical check that the latch holds.

There was no review before that one: this is the task's first revision.

What changed in response:
- **Scoring (not met), the hidden gasket check.** The all-round gasket photo and the feeler gauge are gone. "Seated" is
  now four checks, each made with a simple tool, and the scorer in the task file now applies the same four rules:
  - in line: the middle of the lid handle's end within 13 mm sideways of the body handle's centre line (photo from
    above);
  - handles together: a 4 mm hex key cannot be pushed in between the handles at the body handle's end;
  - depth: a caliper's depth rod at four taped marks on the rim reads within 2 mm of D0, the depth read there with
    the cooker closed by hand (the model's seated depth is 4.5 mm);
  - witness ring: cornflour paste wiped on the rim's underside leaves an unbroken white ring, at least 1 mm wide, all
    round the black gasket, inspected after opening.
- **Seating while the lid is held (0.5).** The first three checks are made on the closed cooker with the robot still
  holding the lid. The tester then takes the lid handle, squeezing the handles together, and opens the cooker to read
  the ring. A robot holding the lid must keep the handles together, as the manual's step 5 does.
- **Seating after latching (1.0).** The same four checks, plus the maker's own closed position from its manual: the
  handles in line and together, and the loop pushed in over their end with its crossbar under the body handle's end
  (photo). The Classic has no printed alignment marks and no pressure-indicator pin, so these are its indicators.
- **A practical check that the latch holds.** Kept, and made measurable: with the hands off, a spring scale hooked
  under the lid handle 10 mm from its end is pulled up to 100 g (1 N) for 1 s, and the handle must not rise
  (the hex key still cannot go in). The scorer now does the same lift in the simulation (1 N at the same point, for
  1 s, 0.5 s after both hands are clear), and every seated and latched check must hold throughout.
- **The scorer changed to match.**
  - Seated was 8 gasket points plus a 3-degree level test. It is now the four checks above: 24 ring points, plus the
    new depth and alignment checks. The handles-together check moved from "latched" to "seated", and the level test,
    which the depth check implies, was dropped.
  - The height tolerance is now 2 mm (it was 1.5 mm), the same in the scorer and in the real depth check.
  - The scene and the scripted moves are unchanged. The demo was re-run under the new scorer, lift included, and
    scores 1.0. The one-hand check scores 0.25 (the lid falls back in).
- **The maker's manual cited** for the marks and indicators: the Classic's own instruction manual, pages 8, 10 and
  12-15, and the maker's web page. Cited: the six closing steps; "centralise the lid so that the gasket is in even
  contact with the rim"; and the trial run's steam-leak check.
- **Found while checking the arithmetic.**
  - The lid's turning moment about the pivot is 0.10 N m with the loop (0.11 N m was given). The grip twist is
    0.03 N m (0.01 N m was given, which left out the loop). Both are corrected.
  - The maker's 1.28 kg turns out to be the weight in the carton. The modelled 1.04 kg body, which was split from
    it, is probably too heavy: a 3.25 mm shell of this shape is 0.60 kg of aluminium. This is now stated under
    "Realism check". A real trial uses the real cooker.
- **Product, price and real-trial kit.**
  - Hawkins Classic CL15: ₹1,119 on Flipkart, ₹1,175-1,250 maker's list price, US$55.67 at a US store. The links are in
    "Real sizes and their sources".
  - The scoring kit costs about US$30: a caliper, a hex key, a spring scale and cornflour.
  - The rice and water are now given as a recipe, fresh each session, and there is a short rig-protection section.

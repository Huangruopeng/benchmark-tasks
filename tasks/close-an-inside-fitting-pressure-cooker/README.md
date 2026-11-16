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
6. The right lets go. The loop now holds the lid up against the rim. Both arms head back to where they started; the
   trial ends once the full score has held for 1 s with the lid, the loop and the cooker still.

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

## Realism check

- **The real product.** A Hawkins Classic CL15, the 1.5 litre size of India's best-selling inside-fitting pressure
  cooker range ("ideal for 1 or 2 persons"; Hawkins sells the same design from 1.5 to 12 litres).
  - The maker's 2022 catalogue gives a 130 mm flat base and 3.25 mm aluminium.
  - Retail listings give 31.5 x 15.9 x 15.1 cm overall and 1.28 kg.
  - Hawkins sells its 11 cm grid for "pressure cookers of mouth diameter 12 cm", its smallest mouth.
  - Modelled: a 159 mm body, the rim's top 105 mm above the counter, an oval mouth of 120 x 138 mm, and an oval lid
    of 128 x 146 mm. The lid and mouth sizes are our estimates: no maker figure was found, so we gave the lid a
    4 mm overlap all round.
- **The real place and height.** The manual's trial run says "close cooker (without vent weight); place cooker on high
  heat", so the cooker is closed before it goes on the stove. Here it is closed on the kitchen counter (the station
  table) at full size. The body handle points left and toward the robot at 45 degrees, as a cook would set it down.
  That angle also keeps the right wrist's roll joint inside its range for the lid's 90-degree turn: with the handle
  pointing straight left, the lid's turn lagged 8-14 degrees at the joint's limit.
- **Every real step.** The maker's closing instructions (Hawkins manual and website, "How to Close") have six steps:
  1. Hold the lid just above the body with the lid handle at a right angle to the body handle.
  2. Tilt the lid into the cooker and move it toward the far wall.
  3. Move the lid handle toward the body handle.
  4. Lower the rest of the lid with a slight twist of the wrist.
  5. Place the lid handle bar on the pivot, centre the lid, and squeeze the lid and body handles together.
  6. Push the locking loop to latch securely.

  The manual adds: "do not let go of the lid handle or the lid will fall into the cooker". All six steps are physical
  here, and none is done by a rule:
  - the lid passes the mouth only by tilting and turning;
  - the handle bar rests on a steel pivot post on the body handle's bracket;
  - the lid falls back inside if nothing holds it up;
  - only the pushed-in loop keeps it up once it is let go.
- **The YAM's limits.**
  - Only the lid's 10 mm handle bar is gripped. The jaw opens to 93.9 mm, and the 159 mm body is never gripped.
  - The hand holds the bar 32 mm from the vent tube, where the manual's step-2 photo holds the lid. The lid's weight
    then twists the grip by about 0.01 N m. Held by the end of its handle it would be about 0.3 N m, more than a
    top-down grip holds (0.1-0.4 N m).
  - All the work is at TOP+0.07 to 0.16 m.
  - Holding the lid up against the rim takes about 1 N (the lid's 0.11 N m about the pivot).
  - The loop weighs 8 g.
  - The two hands work about 20 cm apart.
  - The lid's way in is tight: about 3 mm of room against the arm's 5-19 mm placement error. The scripted demo only
    gets through because it moves slowly, pauses at each pose, and aims against the arm's measured placement errors
    (2-6 mm, measured pose by pose in debug runs).
- **What is simplified.**
  - The rubber gasket is rigid. A real one compresses a millimetre or two when the handles are squeezed.
  - The loop's catch under the body handle is drawn as a pocket shaped as an arc round the loop's hinge, so it
    latches without that travel.
  - The rice and water are a rigid layer of the right mass.
  - The handle bar is rigid on the lid. A real one has deliberate play at the vent tube.
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
the lid's weight turns it down about the pivot (0.11 N m), so it drops back into the cooker as soon as it is let go.
One gripper cannot hold the lid up and push the loop in at the same time. A person squeezes the handles with one hand
and pushes the loop with the other, or with the thumb of the same hand.

In the one-hand check, the right hand did its part exactly as in the demo, with the lid in, on the pivot and seated
against the rim. It held the lid there for 4 s and then let go, while the left never moved. The lid swung back down
inside the cooker (tilted 28 degrees, the handles 67 mm apart), and that trial scored 0.25.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. Everything here is built from shapes with
`bodykit.py`, at real size and mass, as three loose rigid bodies and one hinge.
- **The cooker body** (1.44 kg: 1.04 kg of aluminium plus 0.40 kg of rice and water) stands loose on the counter.
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
    above the body handle.
  - The lid's centre of mass is 38 mm out from the vent tube toward the handle. Released, the lid therefore swings
    down inside the cooker about the pivot (0.11 N m).
- **The locking loop** (8 g) is a steel wire loop: 3.5 mm wire, 44 mm wide, hanging 60 mm from a hinge at the lid
  handle's end.
  - Its legs straddle both handles.
  - Pushed in about 45 degrees, its crossbar goes into the pocket under the body handle.
  - There the lid's weight pulls it up against the pocket's roof, and friction holds it.
  - On the counter it lies folded out.


## Real sizes and their sources

- **Cooker.** Hawkins Classic CL15, 1.5 litres.
  - Hawkins Pressure Cooker Catalog 2022: a 130 mm flat base, 3.25 mm thick; inside-fitting lid; "fixed sturdy pivot"
    that is part of the metal handle; safety valve under the lid handle bar.
  - Retail listings: 31.5 x 15.9 x 15.1 cm overall, 1.28 kg with its lid.
  - The Hawkins 11 cm grid is sold "for pressure cookers of mouth diameter 12 cm".
- **Closing steps.** Hawkins' "How to Close & Open" web page (demonstrated on the Hawkins Classic) and the Hawkins
  Hevibase instruction manual (inside front cover and page 8): the six steps above, the pivot and the locking loop. The
  manual's parts diagram puts the locking loop at the lid handle's end, and its tips say to use the minimum tilt, centre
  the lid before latching, and not to let go of the lid handle or the lid will fall into the cooker.
- **Our estimates.** The mouth (120 x 138 mm), the lid (128 x 146 mm), the rim height (105 mm), the handle bar
  (10 x 8 mm, 150 mm), the lid handle (110 x 22 x 20 mm), the loop (3.5 mm wire, 44 x 60 mm, 8 g) and the split of the
  1.28 kg between the body (1.04 kg) and the lid (0.21 kg plus the loop). They come from the overall sizes and the
  maker's photos.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: the full score, with the cooker, the lid and
the loop still. Otherwise it gives `score()` at the end of the episode: 0, 0.25, 0.5 or 1.0.

**Judged at the end.**
- **Seated.**
  - Eight points round the top of the lid's gasket, at its outer edge, are each in under the rim: at least 1 mm
    outside the mouth's edge and within 1.5 mm of the rim's underside.
  - The lid is level with the cooker within 3 degrees, and it is in contact with the cooker.

  Real trial: the gasket overlaps and touches the rim all round, and a 1.5 mm feeler gauge passes nowhere; photo.
- **Latched.**
  - The locking loop is swung in 34 to 50 degrees from hanging (its crossbar under the pocket's roof).
  - Its crossbar touches the cooker in the pocket under the body handle's end.
  - The lid handle's underside is within 4 mm of the body handle's top at the handles' end.

  Real trial: by eye the loop is over the handles' end, and the lid handle cannot be lifted off the body handle by
  hand.
- **Hands off.** No finger pad touches the lid, the loop or the cooker, and each gripper is at least 5 cm from the lid
  handle's end and from the loop.

**Tiers.**
- 1.0: seated, latched and hands off.
- 0.5: seated.
- 0.25: the lid was at some time wholly inside the cooker, below the rim. Real trial: from the video.
- Otherwise 0.0.

**Failures: 0.0 for good, at any time.** The cooker tips more than 25 degrees, or the cooker or the lid falls off the
counter.

A lid that is not latched swings back down into the cooker as soon as it is let go, so it cannot stay seated with the
hands off.

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
- the eight gasket points' heights against the rim;
- the contact forces of the lid and the loop on the cooker;
- the loop's swing and the handles' gap;
- the finger pads' forces;
- how far the lid has turned in the hand.

`PC_ONEHAND=1` runs the one-hand check.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 1,800 steps (294 s on the clock, with Dream answering in 0.84 s on average), and about 6 minutes for the scripted demo (1,229 steps, 61.5 s of simulated time, 373 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

# Scoop a portion of fries into a fry carton at a fry dump station

A fast-food fry station. On a stainless work counter stands a countertop fry dump station (the Avantco
FFDS-1): a stainless base holding a full-size food pan 2 in deep, a back panel and a heat hood. A heap of fries lies
against the pan's back wall, and the station's fry scoop lies empty in the front of the pan. In front
of the station stands an empty small paper fry carton. The job is the one done at every fry station: scoop fries from
the heap, fill the carton with a portion, put the scoop back and leave the carton standing on the counter. In the real
trial the fries are clean replica fries (hard-maple sticks of a 3/8 in straight-cut fry's size and close to its weight, see below),
so no oil reaches the robot. The task is one task file,
[`fill_a_fry_carton_at_the_dump_station.py`](fill_a_fry_carton_at_the_dump_station.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

The instruction gives the goal and its constraints only ("Using the fry scoop, fill the small fry carton with a portion
of fries from the station. Put the scoop back in the pan, leave the carton standing on the counter, and finish with
both arms back where they started."). How to hold the carton, how to load the scoop, how far to tip it and how to keep
the fries off the counter are left to the policy.

What it tests: loading a scoop from a heap of loose sticks (driving it in along the pan's floor so the sticks ride up
onto its blade, then raising the lip so they slide back into its funnel), carrying the load on a side-handled tool,
tipping it so the sticks slide down its straight chute into a small carton held steady by the other hand, portioning
by amount, and putting the tool back.

Why two hands: the empty carton weighs 7.7 g and stands on a base 3.4 cm deep, while a single replica fry weighs about
4.3 g (a fried fry 5 g). A scoopful landing on its inside back wall, or the scoop's chute brushing its low front, knocks
over a carton that stands alone, and the scoop is a one-handed tool held by its side handle, so the other hand is free
to hold the carton, as workers do. With one hand (the right doing exactly its part of the demo, the left never moving)
the score was 0: as the chute's end came over the mouth it pushed the unheld carton 12 mm aside, the pour then
pushed it 3 cm further and tipped it to 27 degrees, and it fell onto its side with 14 fries in it. A run that fills
the carton by dipping it into the pan (the carton used as a scoop, one hand, the other home) scored 0: the carton-in-the-pan rule latched at 12 s.

## Real-world basis

- **Fry station:** the Avantco FFDS-1 countertop French fry warmer / dump station (WebstaurantStore item 177FFDS1,
  $146.99; 13 x 22.5 x 19.5 in, base 2 7/8 in tall, "a standard size 12" x 20" food pan up to 2" deep", two 500 W
  calrod elements and a halogen light in its overhead unit, 1,000 W). The counter is the robot's table.
- **Scoop:** the body of the Choice 407RFFSCP8 stainless right-handle French fry scoop
  (https://www.webstaurantstore.com/choice-9-x-8-stainless-steel-right-handle-french-fry-scoop/407RFFSCP8.html, $7.49;
  listed "Length: 9", Maximum Width: 8", Minimum Width: 3 3/4"", "Style: Flat Bottom", "Funneled end directs fries into
  serving containers", matte black cool-touch grip). Its product photos show the shape the model follows: a flat front
  blade 8 in wide with straight parallel sides and no walls, a funnel whose two wings close in to a straight,
  parallel-sided chute 3 3/4 in wide at the serving end. Section lengths are scaled from those photos to the listed
  9 in: blade 4.9 cm, funnel 8 cm, chute 10 cm. Walls 2 1/4 in (5.7 cm), the listed height of the same-pattern
  TableCraft FFR8. The handle, as on the Choice, is fixed on the outside of the right wing beside the chute and
  trails back toward the serving end; the model angles it 45 degrees out from the chute, further than the Choice's,
  for the robot arm's reach. Modelled at 0.20 kg.
- **Carton:** the FF1 small fry carton of Graphic Packaging International's fry packaging range (4.50 x 2.25 x 4.50
  in; 1,000 to a 17.0 lb case, so about 7.7 g each), with its low "front scoop" and tall back.
- **Fries and portion:** USDA FNDDS 2021-2023 food code 71401030 "Potato, french fries, fast food" (FoodData Central
  2709461): 1 straight-cut fry 5 g; a kids' order 70 g, a small 110 g, a medium 145 g. A small order is therefore
  about 22 fries; the task asks for 18 to 28 (90 to 140 g of real fries). Fast-food fries are a 3/8 in straight cut
  (for example Simplot "Straight Cut Fries" 3/8 in, SKU 10071179259411).
- **Jobs:** Fast Food and Counter Workers (SOC 35-3023), 3,854,050 employed in May 2025 (BLS OEWS); O*NET 35-3023.00
  task: "Select food items from serving or storage areas and place them in dishes, on serving trays, or in take-out
  bags." The fry scoop, the dump station and the paper carton are the tools those workers use for fries.

## The fries in a real trial: clean replica fries

Fresh fried fries vary from batch to batch and leave oil on everything they touch, so the real trial uses a clean,
reusable stand-in that is built to the simulated fry's size, mass and shape.

- **Source.** 3/8 in (9.5 mm) square hard-maple dowel, kiln-dried and smooth-sanded, for example Woodworking Shop
  SKU SQM380 "3/8" X 36" Maple Square Dowel" ($3.12 each; "All wood dowels have a tolerance of +/- 1/64"",
  https://woodworkingshop.com/product/SQM380). Any 3/8 in square hard-maple (or other hardwood of about 0.7 g/cm3)
  dowel will do. 159 sticks need 12.1 m of dowel: 14 dowels of 36 in, about $44.
- **Size and mass.** Twelve lengths, 55, 60, 64, 68, 71, 75, 78, 82, 85, 88, 91 and 95 mm, in equal numbers
  (13 or 14 of each), as in the simulation. Every long edge and both ends are eased with a 2 mm 45-degree bevel,
  like a fry's rounded edges. Hard maple weighs about 705 kg/m3 at 12% moisture (Wood Database; USDA Wood Handbook
  FPL-GTR-282, sugar maple specific gravity 0.63 at 12% moisture), so a 75 mm stick weighs about 4.3 g (a fast-food
  fry 5 g) and the set about 685 g (computed from the published density).
- **Flow.** Friction of wood on clean metal is 0.2 to 0.6 and of oak on oak 0.54 to 0.62 static (Engineering
  ToolBox); the simulation uses 0.5 stick on stick, 0.55 on the pan and 0.5 on the scoop. Rigid sticks of this size
  and weight are exactly what the simulation models.
- **Preparation (once).** Cut the dowels with a fine saw in a mitre box against a stop block for each length; sand
  the 2 mm bevels with 120-grit paper on a sanding block; brush off the dust. Leave them unfinished: no oil, wax,
  paint or finish. Weigh ten 75 mm sticks and record their mean (about 4.3 g expected); count the set (159).
- **Temperature and handling.** Room temperature (18 to 25 °C), dry. They are not food and are never served.
  Store them in a lidded box marked with the count. Check them before each session and replace any cracked or
  splintered stick with one of the same length.
- **Reset, each trial (about 2 minutes).** Tip the sticks from the carton, and any on the counter, the station deck
  or the floor, back into the pan; count them (159). Push them toward the back wall with the flat of the scoop and
  loosen them by hand into a heap from the back wall to about 2 cm behind the scoop's open end, 6 to 7 cm high, as in
  the setup image. Lay the empty scoop flat in the front of the pan (serving end 4 mm from the front wall, centred,
  handle end over the right rim). Stand an empty carton on its mark (replace it if it is creased, torn or damp).

## Real-trial safety and rig protection

- **No oil, no food.** The replica fries are dry, unfinished hardwood, so nothing greasy reaches the grippers, the
  scoop's handle or the carton. Neither gripper ever needs to touch a fry (a fry the robot touches is "handled" and
  wasted, see Scoring); the grippers hold only the scoop's phenolic handle and the paperboard carton.
- **Station unplugged.** The dump station is used cold and unplugged: its heat hood (two 500 W calrod elements and a
  halogen lamp) stays off, so nothing near the wrist cameras is hot.
- **Edges.** The sticks' edges are bevelled and sanded, so no splinter or sharp corner bears on the gripper pads;
  inspect the scoop's sheet edges and file any burr before the first trial.
- **Cleaning.** Wipe the pan, the scoop and the gripper pads with a dry cloth at the end of a session to remove wood
  dust; no solvent is needed.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The counter and the station are kinematic, fixed
where they are put; the scoop, the carton and the 159 fries are loose rigid bodies at their real sizes and masses.

- **Fries:** each fry is one convex hull: a 9.5 mm square stick with its four long edges and its two ends bevelled
  2 mm at 45 degrees, 55 to 95 mm long, of density 700 kg/m3 (4.26 g at 75 mm). They are spawned lying flat at random
  angles in layers (none touching) behind the scoop and settle into a heap before the trial starts; the same heap every
  trial (fixed seed). Friction 0.5 between fries, 0.55 on the pan and 0.5 on the scoop's bowl.
- **Scoop:** 1 mm sheet, open at both ends. From the serving end: a flat-bottomed chute 10 cm long and 9.5 cm wide
  between straight parallel walls 5.7 cm tall; a funnel 8 cm long whose floor widens to 20.3 cm between two wings
  (full height over their rear half, then a sloping front edge down to the floor); a flat blade 4.9 cm long and 20.3 cm
  wide with straight sides and no walls. The handle (a steel strap from the outside of the right wing, 11.5 cm from
  the serving end, then a 12 cm black grip, 22 x 15 mm, trailing back beside the chute toward the serving end, angled
  45 degrees out from the long axis and rising 35 degrees) has its own material (friction 1.1, black phenolic). It
  starts empty.
- **Carton:** paperboard modelled as rigid 3 mm walls (real about 0.45 mm) at its real 7.7 g: a base, a sloping tall
  back, a sloping low front and two side gussets whose top edges rise from the front's height to the back's.
- **Touches:** a PhysX rigid-contact view reports every contact between a fry and a link of either arm.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step. The fries' convex hulls
are added by the task file while the bodies are authored.

## Scoring

0 at any time, and it stays 0, if any part of the carton goes into the pan below its rim: the carton is not the scoop
(a fry carton can be used as a scoop, and this closes that one-hand route). Only the scoop may serve: a fry is served
only if the scoop carried it up out of the pan during the trial (its centre in the scoop's bowl, above the pan's rim)
and no gripper or arm link ever touched it (PhysX contact view; such a fry is "handled"). The scoop and the carton
both start empty, and a fry found in either when the trial starts could never count. At the end of the trial: 0 if a
handled fry is in the carton; 1.0 with 18 to 28 served fries in the carton, the carton standing upright on the counter
(tilted under 15 degrees, base within 5 mm of the counter), at most 4 fries spilled (outside both the pan and the
carton, handled, or in the carton without having ridden the scoop out of the pan), the scoop back in the pan (both its
ends over the pan, within 12 degrees of flat, its bottom within 3 cm of the pan's floor) and both arms back where they
started (each gripper's fingertip centre within 5 cm of its start); 0.5 with 18 to 28 served fries in the standing
carton; 0.25 with at least 6 served fries in the standing carton; else 0. A fry is in the carton when its centre is
inside the carton's walls, up to 2 cm above its back rim (fries stand proud of a full carton).

The portion is a small fast-food order: 110 g, which is 22 fries at USDA's 5 g a fry; 18 to 28 fries are 90 to 140 g
of real fries. It is counted rather than weighed because a replica stick weighs 4.3 g, not 5 g, so a count gives the
same portion with the stand-in as with real fries.

In a real trial: tip the carton out onto a tray and count its sticks; count the sticks on the counter, the station's
deck and the floor; and check from the video that the carton never went into the pan, that every stick in the carton
was tipped in from the scoop after the scoop had lifted it out of the pan (none pushed or dropped in, none touched by a
gripper), that the carton stands, that the scoop lies in the pan, and that the arms are back at their marks.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the carton, the scoop and every fry still) has held for
1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps
every object exactly where the task file puts it. The cameras render at 1280 x 704 for sharp videos; Dream-1's client
scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same
input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fill_a_fry_carton_at_the_dump_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Using the fry scoop, fill the small fry carton with a portion of fries from the station. Put the scoop back in the pan, leave the carton standing on the counter, and finish with both arms back where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/fries-dream
```

The scripted demo, which reaches full score: the left hand pinches the carton low at its left end, across its front and
back, its wrist leaned out to the left so it stays clear of the mouth, and holds it standing on the counter. The right
takes the scoop's handle from its end, fingers above and below it and in line with it, tips the scoop 8 degrees
lip-down onto the pan's floor and drives it along the floor into the heap with a 20 cm stroke of the hand: the heap
packs against the pan's back wall and the fries ride up over the thin lip onto the open blade (46 fries on the
scoop at the end of the push in the demo run). It levels the scoop and, as the blade has no walls, raises the lip in
place rather than drawing the load back along the floor, so the fries slide back into the funnel, then lifts the
scoopful clear of the rim (41 fries carried out of the pan), carried lip-up. It brings the chute's end over
the carton's mouth, the scoop turned 20 degrees to the left, its end just above the low front and below the tall
back, and tips the scoop up (to 65 degrees, in steps, with three small shakes for the fries that jam in the funnel), so
the fries slide down the straight chute into the carton; it lifts the chute's end straight up off the fries it rests
on, levels the scoop and lays it back in the pan with what is left in it. The left lets go of the filled carton, slides its fingers
off the carton's end and both arms go back to where they started (24 fries served in the carton and
3 spilled in the demo run: full score).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fill_a_fry_carton_at_the_dump_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Using the fry scoop, fill the small fry carton with a portion of fries from the station. Put the scoop back in the pan, leave the carton standing on the counter, and finish with both arms back where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/fries-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/fries-dream/*.json; do inspect-robots video "$log"; done
```

Set `FRIES_DEBUG=1` to print what the score is built from four times a second: how many fries are on the scoop, have
been carried out of the pan on it and are served in the carton, how many are spilled (and, every 2 s, where they lie)
or were touched by the robot, where the carton and the scoop are and how far each is tilted and turned, how far the
scoop has turned in the right hand's grip, the left hand's clearance from the scoop and from the right hand, and where
both TCPs are; at the start it prints the settled heap's extent and height profile. `FRIES_ONE_HAND=1` plays the demo
with the left arm kept home; `FRIES_CARTON_SCOOP=1` plays an adversarial run that fills the carton by dipping it into
the heap.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,600 steps (457 s on the clock, with Dream answering in 0.93 s on average), and about 9 minutes for the scripted demo (1,123 steps, 56.1 s of simulated time, 525 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed (the heap is spawned from a fixed seed), and the scripted demo
follows fixed waypoints.

## Third-party assets

None. Every body (counter, station, scoop, carton and fries) is built in the task file from boxes, cylinders and
convex hulls; the robot is the `yam_arena` embodiment's own.

## Changes in this revision

This is the second revision of TK-DBQWUG. The latest review said:

> **Not accepted** (TK-99EZB6).
> Simulated scoop should have a straight portion to replicate the real thing

What changed in response:

- **The scoop now has the real scoop's straight portions** (task file: `_sc_width()`, `_wall_h()`, `_scoop_shapes()`;
  task.json setup item 4; "Real-world basis" and "How it is modelled" above). It follows the body of the Choice
  407RFFSCP8 (listed 9 in long, 8 in maximum width, 3 3/4 in minimum width, flat bottom; link above). From the serving
  end: a straight chute 10 cm long and 9.5 cm wide between parallel walls 5.7 cm tall; a funnel 8 cm long whose wings
  flare to 20.3 cm; a flat front blade 4.9 cm long and 20.3 cm wide with straight parallel sides and no walls. The
  previous scoop tapered over its whole length with no straight part. Check: the setup screenshot and the demo video
  show the chute, the funnel and the blade; the scorer's "in the scoop's bowl" test uses the same widths.
- **The handle is placed as on the real scoop** (setup item 4; `_h_dir()`, `_h_root()`): fixed on the outside of the
  right wing where the funnel meets the chute, it trails back beside the chute toward the serving end and rises, as on
  the Choice (35 degrees in the model). It is angled 45 degrees out from the chute, further than the Choice's, for the arm's reach
  (fidelity note). Before, it stood straight out sideways near the serving end.
- **The demo works the real shape** ("Run and score a trial"): the blade has no walls, so the load is no longer drawn
  back along the pan's floor; the lip is raised in place and the fries leave through the straight chute. Check: demo
  1.0 (24 served, 3 spilled; the trial ended at step 1,123 of 1,600 once the
  full score had held 1 s); Dream-1 run 0.0.
- **Unchanged:** the scoop and the carton start empty, only fries the scoop carried out of the pan count, 18 to 28 fries,
  the carton-in-the-pan latch, the maple stand-ins and the rig-safety section. Re-run with the new scoop: one hand only
  0; the carton used as a scoop 0 (latched at 12 s).

Earlier review TK-DBQWUG (the scoop started with 141 g of fries; no source, preparation or reset for fried fries; oil
on the shared grippers): answered in the first revision and still in place: the robot loads the empty scoop itself,
and the fries are dry hard-maple stand-ins with a stated source, preparation, handling and 2-minute reset.

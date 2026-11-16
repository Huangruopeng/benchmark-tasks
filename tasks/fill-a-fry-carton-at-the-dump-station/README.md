# Scoop a portion of fries into a fry carton at a fry dump station

A fast-food fry station. On a stainless work counter stands a countertop fry dump station (sized like the Avantco
FFDS-1): a stainless base holding a full-size food pan 2 in deep, a back panel and a heat hood. The pan is heaped with
freshly fried straight-cut fries, and the station's fry scoop lies in the front of the pan where the fries were dumped
over it. In front of the station stands an empty small paper fry carton. The job is the one done at every fry station:
fill the carton with a portion using the scoop, put the scoop back, and leave the carton standing on the counter. The
task is one task file, [`fill_a_fry_carton_at_the_dump_station.py`](fill_a_fry_carton_at_the_dump_station.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

The instruction gives the goal and its constraints only ("Using the fry scoop, fill the small fry carton with a portion
of fries from the station. Put the scoop back in the pan, leave the carton standing on the counter, and finish with
both arms back where they started."). How to hold the carton, where to take the scoop, how far to tip it and how to
keep the fries off the counter are left to the policy.

What it tests: holding a light container steady by one end while the other hand works over it; taking a tool by its
handle and carrying a load on it level; tipping a scoop through its funnel end into a narrow mouth without spilling;
portioning by amount (the score is by weight); and putting the tool back.

Why two hands: the empty carton weighs 7.7 g and stands on a base 3.4 cm deep, while a single fry weighs 5 g. A
scoopful landing on its inside back wall, or the scoop's funnel end brushing its low front, knocks a carton that stands
alone over. With one hand (the right doing exactly its part of the demo, the left never moving) the score was 0: as the scoop's funnel end came down to the carton's mouth it pushed the unheld carton's rim (in the demo the left hand holds the carton against that push, which moves it about 1 cm) and the carton went over at 17 s; all 28 fries ended on the counter. Poured from 3 cm higher instead, so that the funnel end cleared the rim, the landing fries pushed the carton over as well (tilted 42 degrees 3 s into the pour), again 0. A run that fills the carton by dipping it into the pan (the carton used as a scoop, one hand, the other home) scored 0: the carton-in-the-pan rule latched at 11 s.

## Real-world basis

- **Fry station:** sized like the Avantco FFDS-1 countertop French fry warmer / dump station (13 x 22.5 x 19.5 in, base
  2 7/8 in tall, "a standard size 12" x 20" food pan up to 2" deep", two 500 W calrod elements and a halogen light in
  its overhead unit; webstaurantstore spec sheet). The counter is the robot's table.
- **Scoop:** a flat-bottomed stainless right-handle French fry scoop like the Choice 407RFFSCP8 (9 in long, 8 in wide at
  its open end, 3 3/4 in at its funnel end, "Funneled end directs fries into serving containers", cool-touch handle) and
  the TableCraft FFR8 (2 1/4 in tall walls); modelled at 0.20 kg.
- **Carton:** the FF1 small fry carton of Graphic Packaging International's fry packaging range (4.50 x 2.25 x 4.50 in;
  1,000 to a 17.0 lb case, so about 7.7 g each), with its low "front scoop" and tall back.
- **Fries and portion:** USDA FNDDS 2021-2023 food code 71401030 "Potato, french fries, fast food" (FoodData Central
  2709461): 1 straight-cut fry 5 g; a kids' order 70 g, a small 110 g, a medium 145 g. Fries are a 3/8 in straight cut
  (9 mm once fried), 55 to 95 mm long.
- **Jobs:** Fast Food and Counter Workers (SOC 35-3023), 3,854,050 employed in May 2025 (BLS OEWS); O*NET 35-3023.00 task:
  "Select food items from serving or storage areas and place them in dishes, on serving trays, or in take-out bags."

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The counter and the station are kinematic, fixed where
they are put; the scoop, the carton and the 165 fries are loose rigid bodies made of shapes from `bodykit.py`, at their
real sizes and masses.

- **Fries:** rigid sticks 9 mm square, 55 to 95 mm long, 5 g for 75 mm, spawned lying flat at random angles in layers
  (none touching) and settled into a heap before the trial starts; the same heap every trial. Friction 0.5 between
  fries, 0.6 on the pan, 0.55 on the scoop's bowl.
- **Scoop:** a flat bottom and two side walls along the funnel's taper, open at both ends; its handle has its own
  material (friction 1.1, black phenolic). 28 fries (141 g) lie in its front part at the start, as when a basket is dumped over
  a scoop left in the pan.
- **Carton:** paperboard modelled as rigid 3 mm walls (real about 0.45 mm) at its real 7.7 g: a base, a sloping tall back,
  a sloping low front and two side gussets whose top edges rise from the front's height to the back's.
- **Touches:** a PhysX rigid-contact view reports every contact between a fry and a link of either arm.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()` each
body's own pose, and lets the task file act on the simulation after every control step.

## Scoring

0 at any time, and it stays 0, if any part of the carton goes into the pan below its rim: the carton is not the scoop
(the fry carton can be used as a scoop, and this closes that one-hand route). A fry that a gripper or an arm touches is
"handled": it may not be served, and it counts as spilled wherever it lies. At the end of the trial: 0 if a handled fry
is in the carton; 1.0 with 70 to 145 g of fries in the carton (from a kids' to a medium order), the carton standing
upright on the counter, at most 4 fries spilled (about 20 g), the scoop back in the pan and both arms back where they started (each
gripper's fingertip centre within 5 cm of its start); 0.5 with 70 to 145 g in the standing carton; 0.25 with at least
30 g in the standing carton; else 0. A fry is in the carton when its centre is inside the carton's walls, up to 2 cm
above its back rim (fries stand proud of a full carton). In a real trial: weigh the carton on a kitchen scale tared
with an empty carton, count the fries on the counter and the floor, and check from the video that the carton never went
into the pan, that no fry the gripper touched went into the carton, that the carton stands, that the scoop lies in the
pan, and that the arms are back at their marks.

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
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/fries-dream
```

The scripted demo, which reaches full score: the left hand pinches the carton low at its left end, across its front and back, its wrist leaned out to the left so it stays clear of the mouth, and holds it standing on the counter. The right takes the scoop's handle from its end, fingers above and below it and in line with it, tips the scoop lip-down, pushes it 7 cm along the pan's floor into the heap, levels it, raises the lip and lifts the scoopful clear of the pan's rim, carried a little lip-up. It brings the funnel end over the carton's mouth, just above its low front and below its tall back, and tips the scoop up by turning the wrist about the handle (to 65 degrees, in steps, with two small shakes for the fries that jam across the funnel), lifts the funnel end straight up off the fries it rests on, levels the scoop and lays it back in the pan with what is left in it. The left lets go of the filled carton, slides its fingers off the carton's end and both arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fill_a_fry_carton_at_the_dump_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Using the fry scoop, fill the small fry carton with a portion of fries from the station. Put the scoop back in the pan, leave the carton standing on the counter, and finish with both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/fries-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/fries-dream/*.json; do inspect-robots video "$log"; done
```

Set `FRIES_DEBUG=1` to print what the score is built from four times a second: how many fries are on the scoop and in
the carton (and their weight), how many are spilled or were touched by the robot, where the carton and the scoop are
and how far each is tilted, how far the scoop has turned in the right hand's grip, the left hand's clearance from the
scoop and from the right hand, and where both TCPs are. `FRIES_ONE_HAND=1` plays the demo with the left arm kept home;
`FRIES_CARTON_SCOOP=1` plays an adversarial run that fills the carton by dipping it into the pan.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,200 steps (496 s on the clock, with Dream answering in 1.04 s on average), and about 5 minutes for the scripted demo (973 steps, 48.6 s of simulated time, 297 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed (the heap is spawned from a fixed seed), and the scripted demo
follows fixed waypoints.

# Uncork a wine bottle in its cradle

A full 750 ml bottle of red wine lies in a wooden pouring cradle, tipped up 10 degrees, its neck pointing back toward
the right arm and its shoulder against the cradle's neck rest. The bottle and its cork are modelled on the ones we
measured (see "The measurements" below): the bottle is 300 mm long, 70 mm across the body and 28 mm across the mouth,
with a 19 mm bore; its agglomerated cork, 45 mm long and 19 mm across in the bore, sits flush in the neck and grips it
as hard as the real one: it takes 263.3 N to start it, the peak force we measured pulling that cork straight out. A wing
corkscrew (167 mm, 250 g; design values) has been set on the mouth, wings down, its point on the middle of the cork. The
right hand takes the square knob on top and turns it clockwise, a quarter turn at a time (turn, let go, turn back, take
it again), screwing the worm about 41 mm into the cork; the rack draws in and the two wings rise. Then each hand comes
down beside a wing's paddle and pushes it down along its arc, both together; the rack draws the worm and the cork out of
the neck. The right hand lifts off and the left takes both wings the rest of the way down. The corkscrew's stroke (44.9
mm) is about the cork's length, so a few millimetres of cork are still in the neck: the right takes the corkscrew by its
rack and pulls it straight off the bottle, which draws the last of the cork out, and lays it on the table with the cork
on its worm. Both arms go home. The task is one task file,
[`uncork_a_wine_bottle_in_its_cradle.py`](uncork_a_wine_bottle_in_its_cradle.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: using a real lever tool the way people use it: turning a knob in regrip strokes while following it as it
draws in, then pressing its two levers down through a long arc against the cork's grip (263.3 N at the cork, from our
measurement; 41.0 N at the paddles together), and finishing the job with a straight pull that takes the tool and the
cork off the bottle.

One hand or two: the demo presses both wings together, as people do with a wing corkscrew, and the press takes 41.0 N
at the paddles, 20.5 N each with two hands. The score depends only on the outcome, not on the number of hands.
Computed from the arms' joint torques: while the cork holds (the wings between 44 and 90 degrees) the right arm can
push its paddle with 28-36 N at its joints' peak torques and the left arm with 35-46 N, and the simulated drives are
capped at the same peak torques (29-37 N and 33-46 N). On a real YAM the rated (continuous) torques give each arm
11-20 N on its paddle in that range, short of the 41 N one arm would need.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cradle (with its neck rest) is a fixed body; the
bottle, the cork and the corkscrew's five parts (bell, rack, worm spindle and two wings) are loose rigid bodies made of
shapes from `bodykit.py`, at real size (the cork at 10 g, below), joined by PhysX joints authored in the task file.

- The bottle. A 190 mm body 70 mm across, a 30 mm shoulder in two steps, and an 80 mm neck 28 mm across, with a 19 mm
  bore: 300 mm overall, the dimensions we measured. Its glass is near-black green with a cream label and a navy foil
  capsule, as on the measured bottle (photo 7). It weighs 1.25 kg full (design value: 0.75 kg of wine, 0.5 kg of
  glass).
- The cork's grip (the calibrated stand-in for a compressed cork's friction). The cork, a cylinder 45 mm long and 19 mm
  across (the bore), slides out along the neck on a slide joint whose drive is capped at the friction force and
  re-aimed after every control step so that it never pulls the cork back: a Coulomb friction. The force is 263.3 N
  while the whole 45 mm is in the neck and falls in proportion to the length still in it (friction on the contact
  length): 263.3 N x (45 mm - drawn) / 45 mm. 263.3 N is the peak we measured pulling this cork straight out with a
  corkscrew on a luggage scale (see "The measurements" below). Once its inner end has passed the lip the cork is free of
  the neck. The cork weighs 10 g in the simulation, more than a real cork: with a 3.5 g cork (an earlier version),
  between the neck's hold and the worm's thread PhysX's solver shared the load wrongly (the worm slid 12 mm out of the
  cork under a 240 N pull); at 10 g the cork starts at the set force and the worm gives about 3 mm. Its 0.1 N weight is
  nothing against the forces involved.
- The neck rest. Real pouring cradles hold the neck as well as the body, so that the bottle cannot slide out when the
  cradle is tipped to pour. Here a beech U-yoke under the neck, on a third post, has its face 1.5 mm in front of the
  shoulder, with 2 mm round the 28 mm neck; the shoulder bears on it when the corkscrew is pulled off.
- The worm's thread. PhysX has no screw thread. A joint keeps the cork on the worm's axis and lets the worm turn in it
  against the thread's drag; its slide (the depth of the point in the cork) is held by a stiff drive. Resting on the
  cork's face, the point cannot be pushed in unturned but can be drawn back out (the drive's target follows it out
  only). Turned 45 degrees clockwise there, the thread has caught, and from then on the target follows the turns, 10 mm
  per turn of the spindle against the rack, the way the real worm draws in. Turned back past its point, or drawn 3 mm
  out of the cork before it caught, the worm is free of the cork. (This joint is on from the start: PhysX did not carry
  the cork's full grip reliably through a joint switched on during the run.)
- The rack and the pinions. The rack slides in the bell; the worm's spindle turns in the rack (a real wing corkscrew's
  rack has ring teeth and turns with the worm; splitting it into a non-turning rack and a turning spindle is the same
  motion). Each wing turns on a pivot in the bell and its pinion (17.5 mm pitch radius) meshes with the rack. PhysX's
  rack-and-pinion joint cannot run in this GPU scene, so the mesh is built from drives: before every physics step the
  task file takes the mean of where the rack and each wing say the gear is, and aims the rack's slide drive and each
  wing's pivot drive at it (60 kN/m at the rack, the same stiffness seen through the pinion at each wing). A torque on a
  wing reaches the rack as that torque over 17.5 mm, and the cork's pull comes back to the paddles the same way. Under
  the full 263.3 N the rack and each wing differ by 6.5-6.7 mm of rack travel: the mesh's give. With that and the
  worm's 3.3 mm of give in the cork, the wings turn from 39.4 to about 77 degrees before the cork starts.
- The paddles' push is read from the gear: each wing's drive torque over the paddle's 112.5 mm lever. In a debug run of
  the demo the cork started at 263.5 N on the rack, with 21.4 N on the front paddle (the right hand) and 20.0 N on the
  back one (the left), 41.4 N together (peaks 21.8 N and 20.2 N); then the pull fell with the cork's grip as it came
  out. At that point (the wings at about 77 degrees) each arm can push its paddle about 28 N (right) and 45 N (left)
  at its joints' peak torques (see "Why the bottle lies in a cradle").
- The bell's collar. The bell's skirt (33 mm inside) fits over the 28 mm mouth with 2.5 mm to spare all round, and its
  inner ring sits on the lip. Contact between rings of boxes did not hold a 250 g corkscrew level on a near-horizontal
  bottle (it drooped 13 degrees), so the collar's fit is a joint: it keeps the bell on the neck's axis, free to slide
  off it and to turn on it against the collar's friction on the lip (0.17 N m s/rad), until the collar has slid 15 mm
  off the mouth; from then on the corkscrew is free of the bottle.
- The square knob is gripped across its flats, so the turn is passed on by the flats, not by friction alone; the flats
  come round every 90 degrees, which is how far the wrist turns in each stroke.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step; the task file also hooks
the scene's write to the simulator, which runs before every physics step, for the gear.

## The measurements

We measured the bottle and its cork ourselves. The bottle is a Bogle Family Vineyards Cabernet Sauvignon, 750 ml, with
the marking "1126259 1000" on its base, the barcode UPC 0 80887 49396 6 on its label (photo 7) and an agglomerated
(technical) cork branded "BOGLE FAMILY VINEYARDS". Every figure below is from this one bottle and its cork. The photos
are in [`measurement/`](measurement/).

| Quantity | Value | Instrument | Photo |
|---|---|---|---|
| Overall length, base to mouth | 300 mm | tape measure, 1 mm divisions | [7](measurement/7_bottle_length_300mm.jpg) |
| Body, base to shoulder | 190 mm | tape measure, 1 mm divisions | [7](measurement/7_bottle_length_300mm.jpg) |
| Shoulder, body to neck | 30 mm | tape measure, 1 mm divisions | [7](measurement/7_bottle_length_300mm.jpg) |
| Neck, shoulder to mouth | 80 mm | tape measure, 1 mm divisions | [7](measurement/7_bottle_length_300mm.jpg) |
| Body outer diameter | 70 mm | tape measure, 1 mm divisions | [6](measurement/6_bottle_body_diameter_70mm.jpg) |
| Mouth outer diameter | 28 mm | tape measure, 1 mm divisions | [9](measurement/9_mouth_outer_diameter_28mm.jpg) |
| Bore (mouth inner diameter) | 19 mm | soft tape measure across the mouth, 1 mm divisions | [8](measurement/8_mouth_bore_tape.jpg) |
| Cork length, after extraction | 45 mm | tape measure, 1 mm divisions | [4](measurement/4_cork_length.jpg) |
| Cork diameter, after extraction | 22 mm | tape measure, 1 mm divisions | [5](measurement/5_cork_diameter.jpg) |
| Peak straight-pull extraction force | 26.84 kg = 263.3 N | digital luggage scale, 0.01 kg | [1](measurement/1_straight_pull_setup.jpg), [2](measurement/2_peak_reading_26.84kg.jpg) |

The pull test. The bottle stood on the floor, held still between the tester's feet. A True "Soar" winged corkscrew
(photo [3](measurement/3_bottle_and_corkscrew.jpg)) was screwed fully into the cork, its wings raised, and used only as
a puller, without its lever action. The strap of the digital luggage scale was hooked on the corkscrew's top, and the
scale was pulled straight up by hand, along the neck, until the cork came out. We recorded the scale's peak reading,
26.84 kg (photo 2): 26.84 kg x 9.81 m/s² = 263.3 N. This is within the 120-400 N published for wine corks (the
Australian Wine Research Institute's fact sheet "Applying cylindrical closures").

How the task follows them:

- **Bottle.** The simulated bottle has the measured dimensions: body 70 mm across and 190 mm long, shoulder 30 mm, neck
  80 mm, 300 mm overall, 28 mm across the mouth and a 19 mm bore.
- **Cork.** 45 mm long and 19 mm across, the bore it is compressed in, flush with the lip. Out of the bottle the cork
  re-expanded to 22 mm.
- **Force.** The simulated cork starts at 263.3 N, and the force falls in proportion to the length still in the neck
  (263.3 N x remaining length / 45 mm). The measured pull ran straight along the neck, as the rack pulls the cork in the
  task, so the force acts at the rack as measured; the wings' 6.4:1 leverage lies between the paddles and the rack.

## Trial bottles

Trial corks come from the measured batch, and each trial bottle is checked against the measurements before use:

- **Same product, same lot.** Trial bottles are Bogle Family Vineyards Cabernet Sauvignon, 750 ml, bought from the same
  retail lot as the measured bottle, each with the same base marking "1126259 1000", the same label barcode
  UPC 0 80887 49396 6 and the same cork branded "BOGLE FAMILY VINEYARDS".
- **Bore, every bottle.** Before the trials, with the foil capsule cut away from the mouth, a soft tape measure laid
  across the mouth (as in photo 8) reads the bore of each trial bottle: 19 mm. A bottle that reads otherwise is not
  used.
- **Force, one bottle per case.** One bottle from each case of 12 is opened with the same luggage-scale pull as above.
  The case is used when the peak reading is within 10% of the measured 26.84 kg: 24.16-29.52 kg (237-290 N).

## Why the bottle lies in a cradle: where a YAM arm can push

We checked the layouts a person uses with a wing corkscrew against the YAM's reach and joint torques (URDF kinematics,
the force each arm can push along the paddle's path from the Jacobian, its joint torque limits and the arm's own weight:
DM4340 motors on joints 1-3, 27 N m peak and 9 N m rated, and DM4310 on joints 4-6, 7 N m peak and 3 N m rated). The
cork's 263.3 N needs 41.0 N on the two paddles together, 20.5 N each.

| Layout | Paddle path (height above the table) | Reach | Push per arm along the paddle's path |
|---|---|---|---|
| Upright bottle on the table, wings pressed down (the most common way) | 457 mm at the start, 370 mm halfway, 258 mm at the end; the knob at 442 mm | no IK solution for either arm at any of these points | none |
| Bottle in the cradle, corkscrew turned so the wings swing up and down | upper paddle 126-247 mm, pushed upward for its first 40 degrees; lower paddle down to 33 mm below the table top | the lower wing hits the table | none |
| Bottle in the cradle, tipped 10 degrees, wings swinging nearly level (this task) | 133 mm at the start, 100 mm at the end | both arms, 14-25 degrees of joint margin | while the cork holds (wings at 44-90 degrees): right 28-36 N, left 35-46 N at peak torque (a wrist joint, 4 or 5, is the limit); 11-20 N at rated torque |

So the cradle is the only layout the YAM reaches, and an older red is opened lying in its cradle in real life too. The
press needs the wrist motors' short-term peak torque, not their rated torque, for the about 4 s that the cork takes to
start; the simulated arm's drives are capped at the same peak torques.

The last few millimetres. The corkscrew's stroke is 17.5 mm x 147 degrees of wing travel = 44.9 mm, about the cork's 45
mm, and the worm can only be screwed about 41.4 mm in: deeper, the wings start so close to the axis that the two hands
would begin the press 97-110 mm apart (129 mm now), close enough for the wrists to collide. In the demo the left takes
the wings to about 175 degrees, and 4.7 mm of cork is still in the neck, held by 263.3 N x 4.7 / 45 = 27.5 N, falling to
nothing as it comes out. The right pulls it out along the neck by the corkscrew's rack: pulling the rack pulls the worm
and the cork directly, and the bell with it once the rack is out against its stop. (Pulled by the bell instead, the rack
slides into the bell and the wings rise: the cork stays.) With the hand straight down, the wrist's pitch joint limits
that pull to about 30 N at peak torque; with the fingers leaning 45 degrees toward the bottle it runs nearly along the
hand, and the arm can pull about 60 N at peak torque and 31 N at rated torque. The reaction goes into the cradle's neck
rest. On the slats alone the bottle would slide at about 11.6 N: its 12.26 N weight presses 12.07 N / cos 40 degrees =
15.8 N on the two slats (set 40 degrees either side of the bottom), times a friction of 0.6 is 9.5 N, plus 2.1 N of its
weight down the 10-degree slope.

## Real-trial safety and rig protection

- A rubber bar mat (about 30 x 15 cm) lies under the cradle's neck end, and a folded paper towel under the mouth, so
  any wine that runs from the mouth as the cork comes free stays off the bench.
- The grippers' pads wear nitrile finger cots for the session; they touch only the corkscrew's knob, paddles and rack.
- After each trial the corkscrew, the mat and the cradle's neck rest are wiped with a damp cloth and dried; the cots
  are changed if wine reaches them.
- The worm's point faces into the cork at the start, and nobody handles the corkscrew while the arms move.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the bottle, the cork and the corkscrew's bell still,
and both hands still and at least 10 cm clear of them) has held for 1 s, otherwise `score()` at the end of the episode
(0, 0.25, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same layout.
The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it,
so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=uncork_a_wine_bottle_in_its_cradle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the bottle of wine without taking it out of its cradle, and leave the cork off it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cork-dream
```

The scripted demo, which reaches full score: the right hand comes onto the square knob from the side, grips it across
its flats and turns it clockwise in quarter-turn strokes (turn, let go, turn back, take it again), following the knob
in as the worm draws into the cork, until the worm is about 41 mm in and the wings are up. It backs off and comes down
beside the front wing's paddle while the left comes down beside the back one, and both push their paddles down along
their arcs together, the closed fingers turning with the paddles and aimed 5-7 cm ahead of them, until the wings are at
172 degrees. The right lifts off; the left takes both wings the rest of the way down to 175 degrees, holds them there
for 1 s and goes home. The right then takes the corkscrew by its rack, between the wings' heads and the knob, its
fingers closing across the rack and leaning 45 degrees toward the bottle, and pulls it straight along the neck: the
rack pulls the worm and the cork's last few millimetres out of the neck, and the bell with it off the mouth. It lays
the corkscrew on the table to the right, with the cork on its worm, and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=uncork_a_wine_bottle_in_its_cradle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the bottle of wine without taking it out of its cradle, and leave the cork off it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cork-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cork-dream/*.json; do inspect-robots video "$log"; done
```

Set `CORK_DEBUG=1` to print what the score is built from four times a second: how far the cork has been drawn out and
its grip, the worm's depth in the cork (and how far off its axis and tilted), whether the thread has caught and the
turns since, the rack's slide and both wings' angles, the mesh's give, the push on each paddle and the pull on the rack,
how far the bottle has moved and turned, and where each TCP is against the paddles and the knob.

## Runtime per trial

On one RTX 5090: about 17 minutes for a Dream-1 trial of 2,400 steps (994 s on the clock, with Dream answering in 0.69 s on average), and about 21 minutes for the scripted demo (1,799 steps, 90.0 s of simulated time, 1245 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-ZARV6K. Its review said:

> Construct validity (not met): The measured bottle’s bore is assumed rather than measured, and using the same wine
> does not establish that trial corks are from the measured batch.
>
> What to change: Measure the bore of the selected trial bottles and establish that their corks are from the batch
> used for the extraction-force measurement, or measure the force for the verified trial batch and bore. Update the
> setup and simulation to match those measurements, and provide a matching full-score demo if the calibration changes.

The review before that (TK-BV97DW) asked for the extraction force to be measured for the selected cork batch and
bottle bore, with the simulation calibrated to it and a matching demo; we measured it ourselves (263.3 N), and that is
kept. The one before (TK-DMFHER) asked for a measured force rather than a published one; that is kept. The first review
(TK-BKSCBK) asked for a real interference fit, the cork flush in the neck and the Societal relatability category; that
is kept.

What changed in response:

- **The bore is measured: 19 mm.** On the bottle whose cork we pulled, a soft tape measure across the mouth reads
  19 mm (photo 8). We measured the rest of that bottle with the tape as well: 300 mm long (body 190 mm, shoulder 30 mm,
  neck 80 mm), 70 mm across the body and 28 mm across the mouth (photos 6, 7 and 9). "The measurements" lists each
  quantity with its value, instrument and photo, and the pull-test method.
- **The trial corks are from the measured batch.** Trial bottles are Bogle Family Vineyards Cabernet Sauvignon,
  750 ml, from the same retail lot as the measured bottle, with its base marking "1126259 1000", its label barcode
  UPC 0 80887 49396 6 and its cork branded "BOGLE FAMILY VINEYARDS".
- **The trial bottles' bore and force are checked.** Before the trials, the bore of each trial bottle is read with the
  same soft tape across the mouth (19 mm; any other reading and the bottle is not used), and one bottle per case of 12
  is pulled with the same luggage-scale method; the case is used when it reads within 10% of 26.84 kg
  (24.16-29.52 kg, 237-290 N). This is in the setup, the reset notes and "Trial bottles".
- **Setup and simulation match the measurements.** The simulated bottle is now 70 mm across the body (it was 75 mm)
  and 28 mm across the mouth (it was 31 mm), with the measured 19 mm bore, and the cork in the neck is 19 mm across;
  its 300 mm length (190 + 30 + 80 mm) and the 45 mm cork already matched. The glass is now near-black green and the
  capsule navy, as on the measured bottle. The cradle's slats follow the narrower body, and the neck rest keeps 2 mm
  round the 28 mm neck.
- **Calibration kept at the measured 263.3 N; new full-score demo.** The force did not change (41.0 N on the two
  paddles together), but the bottle did, so the demo was run again on the new bottle. In a debug run the cork started
  at 263.5 N on the rack, with 21.4 N and 20.0 N on the paddles (41.4 N together); 4.7 mm of cork was left in the
  neck after the press and came out with the straight pull; the bottle moved at most 2.7 mm and turned at most
  2.3 degrees in its cradle; and the trial scored 1.0 (1,799 steps). Two further demo runs also scored 1.0 in 1,799
  steps; the demo and Dream-1 videos are new, and Dream-1 scored 0.0 (it never touched the corkscrew).

# Uncork a wine bottle in its cradle

A full 75 cl bottle of red wine lies in a wooden pouring cradle, tipped up 10 degrees, its neck pointing back toward the
right arm and its shoulder against the cradle's neck rest. Its agglomerated cork, 45 mm long, sits flush in the neck
and grips it as hard as a real one: it takes 263.3 N to start it, the peak force we measured pulling the cork of a
bottle of the same wine straight out (see "The extraction force" below). A wing corkscrew (real size, 250 g) has been
set on the mouth, wings down, its point on the middle of the cork. The right hand takes the square knob on top and
turns it clockwise, a quarter turn at a time (turn, let go, turn back, take it again), screwing the worm about 41 mm
into the cork; the rack draws in and the two wings rise. Then each hand comes down beside a wing's paddle and pushes
it down along its arc, both together; the rack draws the worm and the cork out of the neck. The right hand lifts off
and the left takes both wings the rest of the way down. The corkscrew's stroke (44.9 mm) is about the cork's length,
so a few millimetres of cork are still in the neck: the right takes the corkscrew by its rack and pulls it straight
off the bottle, which draws the last of the cork out, and lays it on the table with the cork on its worm. Both arms go
home. The task is one task file, [`uncork_a_wine_bottle_in_its_cradle.py`](uncork_a_wine_bottle_in_its_cradle.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: using a real lever tool the way people use it: turning a knob in regrip strokes while following it as it
draws in, then pressing its two levers down through a long arc against the cork's grip (263.3 N at the cork, from our
measurement; 41.0 N at the paddles together), and finishing the job with a straight pull that takes the tool and the
cork off the bottle.

One hand or two: the demo presses both wings together, as people do with a wing corkscrew, and the press takes 41.0 N
at the paddles, 20.5 N each with two hands. We did not re-run the one-hand check at 263.3 N before this revision's
deadline, so we make no claim that the simulation needs two hands. The arms' joint torques give the expected answer.
While the cork holds (the wings between 44 and 90 degrees) the right arm can push its paddle with 28-36 N at its
joints' peak torques and the left arm with 35-46 N, and the simulated drives are capped at the same peak torques
(29-37 N and 33-46 N). So the right alone could not start the cork in the simulation, but the left alone might. On a
real YAM the rated (continuous) torques give each arm 11-20 N on its paddle in that range, well short of 41 N for one
arm. (In the previous revision, at 196 N, one simulated arm started the cork with up to 32.6 N.)

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cradle (with its neck rest) is a fixed body; the
bottle, the cork and the corkscrew's five parts (bell, rack, worm spindle and two wings) are loose rigid bodies made of
shapes from `bodykit.py`, at real size (the cork at 10 g, below), joined by PhysX joints authored in the task file.

- The cork's grip (the calibrated stand-in for a compressed cork's friction). The cork slides out along the neck on a
  slide joint whose drive is capped at the friction force and re-aimed after every control step so that it never pulls
  the cork back: a Coulomb friction. The force is 263.3 N while the whole 45 mm is in the neck and falls in proportion
  to the length still in it (friction on the contact length): 263.3 N x (45 mm - drawn) / 45 mm. 263.3 N is the peak
  we measured pulling the cork of a bottle of this wine straight out with a corkscrew on a luggage scale (see "The
  extraction force" below). Once its inner end has passed the lip the cork is free of the neck. The cork weighs 10 g
  in the simulation, not its real 5.7 g or so: with a 3.5 g cork (an earlier version), between the neck's hold and the
  worm's thread PhysX's solver shared the load wrongly (the worm slid 12 mm out of the cork at 240 N); at 10 g the
  cork starts at the set force and the worm gives about 3 mm. The extra 4.3 g is nothing against the forces involved.
- The neck rest. Real pouring cradles hold the neck as well as the body, so that the bottle cannot slide out when the
  cradle is tipped to pour. Here a beech U-yoke under the neck, on a third post, has its face 1.5 mm in front of the
  shoulder, with 2 mm round the neck; the shoulder bears on it when the corkscrew is pulled off.
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
  the full 263.3 N the rack and each wing differ by about 6.5 mm of rack travel: the mesh's give. With that and the
  worm's 3.3 mm of give in the cork, the wings turn from 39.5 to about 77 degrees before the cork starts.
- The paddles' push is read from the gear: each wing's drive torque over the paddle's 112.5 mm lever. In a debug run of
  the demo the cork started at 263.7 N on the rack, with 21.6 N on the front paddle (the right hand) and 19.8 N on the
  back one (the left), 41.4 N together (peaks 21.6 N and 20.0 N); then the pull fell with the cork's grip as it came
  out. At that point (the wings at about 77 degrees) each arm can push its paddle about 28 N (right) and 45 N (left)
  at its joints' peak torques (see "Why the bottle lies in a cradle").
- The bell's collar. The bell's skirt fits over the bead with 1 mm to spare and its inner ring sits on the lip. Contact
  between rings of boxes did not hold a 250 g corkscrew level on a near-horizontal bottle (it drooped 13 degrees), so
  the collar's fit is a joint: it keeps the bell on the neck's axis, free to slide off it and to turn on it against
  the collar's friction on the lip (0.17 N m s/rad), until the collar has slid 15 mm off the bead; from then on the
  corkscrew is free of the bottle.
- The square knob is gripped across its flats, so the turn is passed on by the flats, not by friction alone; the flats
  come round every 90 degrees, which is how far the wrist turns in each stroke.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step; the task file also hooks
the scene's write to the simulator, which runs before every physics step, for the gear.

## The extraction force: our measurement

We measured the cork's extraction force ourselves, on one bottle.

- **Bottle and cork.** Bogle Family Vineyards Cabernet Sauvignon (California), 750 ml, with an agglomerated
  (technical) cork branded "BOGLE FAMILY VINEYARDS". After extraction the cork measured about 45 mm long and about
  22 mm across with a tape measure; it had partly re-expanded from the bore. Its diameter before bottling is not known
  to us: 24 mm is the usual figure for still-wine corks, a standard value, not our measurement. We did not measure the
  bottle's bore; the task uses the standard CETIE bore of 18.5 mm.
- **Method.** The bottle stood on the floor, held still between the tester's feet. A True "Soar" winged corkscrew
  (chrome finish; retailers list it at 6.75 in, 171 mm, long) was screwed fully into the cork, its wings raised, and
  used only as a puller, without its lever action. The strap of a digital luggage scale was hooked on the corkscrew's
  top, and the scale was pulled straight up by hand, along the neck, until the cork came out. We recorded the scale's
  peak reading.
- **Result.** Peak 26.84 kg: 26.84 kg x 9.81 m/s² = 263.3 N.
- **Limits.**
  - One bottle (n = 1), so we know no spread between corks.
  - A hand pull at no set speed, not a motorised pull at a constant 300 mm/min as in ISO 9727-5.
  - The scale reads in 0.01 kg steps (about 0.1 N); its accuracy is not known to us. The reading may include the
    corkscrew's own weight, about 0.1-0.2 kg (under 1% of the peak).
  - The cork's size was measured with a tape measure after it came out, so to about a millimetre, and its diameter
    in the bore is not known.
- **Photos** (in [`measurement/`](measurement/)):
  - [the straight pull](measurement/1_straight_pull_setup.jpg): the bottle between the feet, the corkscrew screwed in
    with its wings up, the scale's strap on its top;
  - [the peak reading, 26.84 kg](measurement/2_peak_reading_26.84kg.jpg);
  - [the bottle and the corkscrew](measurement/3_bottle_and_corkscrew.jpg);
  - [the cork's length, about 45 mm](measurement/4_cork_length.jpg);
  - [the cork's diameter, about 22 mm](measurement/5_cork_diameter.jpg).
- **Context.** The result is in line with the extraction forces published for wine corks, about 120-400 N: 120-350 N
  (CETIE 2008) and 230-400 N (McMahon 1994), as given in the Australian Wine Research Institute's fact sheet "Applying
  cylindrical closures", and 196 ± 20 N (mean ± SD) for a natural cork in a 17.7 mm neck in Giunchi, Versari,
  Parpinello and Galassi (2008), *Journal of Food Engineering* 88(4), 576-580,
  https://doi.org/10.1016/j.jfoodeng.2008.03.004. These are context only; the task is calibrated to our measurement.

How the task follows it:

- **Force.** The simulated cork starts at 263.3 N, and the force falls in proportion to the length still in the neck
  (263.3 N x remaining length / 45 mm). The measured pull ran straight along the neck, as the rack pulls the cork in
  the task, so the force acts at the rack as measured; the wings' 6.4:1 leverage lies between the paddles and the rack.
- **Cork and bottle.** An agglomerated cork 45 mm long, flush with the lip, in an 18.5 mm bore (standard, not
  measured). It weighs about 5.7 g in real life (45 x 24 mm, 20.4 cm³, at 280 kg/m³, a maker's density for
  micro-agglomerated corks); the simulation uses 10 g (see "How it is modelled").
- **Trial bottles.** Unopened 750 ml bottles of the same wine, Bogle Family Vineyards Cabernet Sauvignon.

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
would begin the press 97-110 mm apart (129 mm now), close enough for the wrists to collide. In the demo the wings reach
about 173 degrees, and 4.7 mm of cork is still in the neck, held by 263.3 N x 4.7 / 45 = 27.5 N, falling to nothing as
it comes out. The right pulls it out along the neck by the corkscrew's rack: pulling the rack pulls the worm and the
cork directly, and the bell with it once the rack is out against its stop. (Pulled by the bell instead, the rack slides
into the bell and the wings rise: the cork stays.) With the hand straight down, the wrist's pitch joint limits that pull
to about 30 N at peak torque; with the fingers leaning 45 degrees toward the bottle it runs nearly along the hand, and
the arm can pull about 60 N at peak torque and 31 N at rated torque. The reaction goes into the cradle's neck rest. On
the slats alone the bottle would slide at about 11.6 N: its 12.26 N weight presses 12.07 N / cos 40 degrees = 15.8 N on
the two slats (set 40 degrees either side of the bottom), times a friction of 0.6 is 9.5 N, plus 2.1 N of its weight
down the 10-degree slope.

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
their arcs together, the closed fingers turning with the paddles and aimed 5 cm ahead of them, until the wings are at
165 degrees. The right lifts off; the left takes both wings the rest of the way down to 175 degrees, holds them there
for 1 s and goes home. The right then takes the corkscrew by its rack, between the wings' heads and the knob, its
fingers closing across the rack and leaning 45 degrees toward the bottle, and pulls it straight along the neck: the
rack pulls the worm and the cork's last few millimetres out of the neck, and the bell with it off the bead. It lays the
corkscrew on the table to the right, with the cork on its worm, and goes home:

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

On one RTX 5090: about 22 minutes for a Dream-1 trial of 2,400 steps (1294 s on the clock, with Dream answering in 0.75 s on average), and about 26 minutes for the scripted demo (1,799 steps, 90.0 s of simulated time, 1549 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-BV97DW. Its review said:

> Construct validity (not met): The 196 N simulated resistance has not been measured for the specified cork batch and
> bottle, as required by the latest feedback.
>
> What to change: Measure and report the extraction force for the selected cork batch and 17.7 mm bottle bore using the
> documented method. Calibrate the simulation to that measured result and provide a matching full-score demonstration.

The review before that (TK-DMFHER) asked for the extraction force to be measured for the selected cork and bottle
rather than taken from published figures; it is now our own measurement, below. The first review (TK-BKSCBK) asked
for a real interference fit at a calibrated extraction force, the cork flush in the neck and the Societal
relatability category; that was done in an earlier revision and is kept.

What changed in response:

- **Measured by us, and reported.** We pulled the cork of one bottle (n = 1) of Bogle Family Vineyards Cabernet
  Sauvignon, 750 ml, straight out by hand with a winged corkscrew (True "Soar", used only as a puller) hung on a
  digital luggage scale, with the bottle held between the feet. The peak reading was 26.84 kg: 26.84 x 9.81 = 263.3 N.
  The method, its limits (one bottle, a hand pull at no set speed, the scale's 0.01 kg steps) and five photos (the
  pull, the reading, the bottle and corkscrew, the cork's length and diameter) are in the README and the task's
  `measurement/` folder. The published figures are kept only as context (120-400 N for wine corks; 196 ± 20 N in
  Giunchi et al. 2008).
- **The cork and bottle we measured.** The cork is an agglomerated (technical) cork, about 45 mm long and about 22 mm
  across after extraction (partly re-expanded); the simulated cork is now 45 mm long (it was a 38 mm natural cork).
  We did not measure the bore, so the bottle now has the standard CETIE 18.5 mm bore, stated as standard, not
  measured; the 17.7 mm bore came from the earlier citation and is gone. Trial bottles are unopened bottles of the
  same wine, so the cork batch and the bottle are the ones measured; the earlier batch check, built round the
  published 196 N, is removed.
- **Calibrated to the measurement.** The simulated cork now starts at 263.3 N (34% more than 196 N) and the force
  falls in proportion to the length left in the neck, 263.3 N x remaining / 45 mm.
- **Matching full-score demonstration.** In a debug run of the demo the cork started at 263.7 N on the rack, with
  21.6 N and 19.8 N on the two paddles (41.4 N together; the arms can push about 28 N and 45 N there at peak torque).
  The wings then reached about 173 degrees, leaving 4.7 mm of cork in the neck, held by 27.5 N; the right pulled it
  out by the corkscrew's rack and laid the corkscrew down, and the trial scored 1.0 (1,799 steps). The final demo run
  scored 1.0, and the demo and Dream-1 videos are new; Dream-1 scored 0.0 (it never touched the corkscrew).
- **One hand.** Not re-run at 263.3 N before the deadline; the task makes no two-hand claim. From the arms' joint
  torques, one arm would need 41.0 N on its paddle: the right can push 28-36 N along its paddle's path at peak torque,
  the left 35-46 N, so one left arm might start the cork in the simulation.
- **Layout checked against the robot.** A reach and torque check (URDF kinematics and joint torque limits) shows the
  cradle is the only layout the arms reach: upright, the paddles would be 258-457 mm above the table, out of reach.
  The press relies on the wrist motors' short-term peak torque for about 4 s. To take the cork's last few millimetres out
  (the corkscrew's 44.9 mm stroke is about the cork's length), the cradle has a neck rest, as real pouring cradles
  do, and the demo ends with a straight pull on the corkscrew by its rack.

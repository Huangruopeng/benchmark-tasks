# Uncork a wine bottle in its cradle

A full 75 cl bottle of red wine lies in a wooden pouring cradle, tipped up 10 degrees, its neck pointing back toward the
right arm. Its natural cork sits flush in the neck and grips it as hard as a real one: it takes 196 N to start it,
the mean extraction force a published study measured for a natural cork in a bottle with this 17.7 mm bore (not a
measurement of ours; see "The extraction force" below). A
wing corkscrew (real size, 250 g) has been set on the mouth, wings down, its point on the middle of the cork. The right
hand takes the square knob on top and turns it clockwise, a quarter turn at a time (turn, let go, turn back, take it
again), screwing the worm 40 mm into the cork; the rack draws in and the two wings rise. Then each hand comes down
beside a wing's paddle and pushes it down along its arc, both together; the rack draws the worm and the cork out of the
neck. The right hand lifts off, the left takes both wings the rest of the way down, and the right lifts the corkscrew
off the bottle by its bell, with the cork on the worm, and lays it on the table. Both arms go home. The task is one task
file, [`uncork_a_wine_bottle_in_its_cradle.py`](uncork_a_wine_bottle_in_its_cradle.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: using a real lever tool the way people use it: turning a knob in regrip strokes while following it as it
draws in, then pressing its two levers down through a long arc against the cork's grip (196 N at the cork, from the
published measurement; about 30 N at the paddles), and finishing the job by taking the tool and the cork off the
bottle.

One hand or two: the demo presses both wings together, as people often do, but the task does not need two hands in
the simulation, and we do not claim that it does. The corkscrew multiplies the push at each paddle by 112.5 / 17.5 =
6.4, so the cork needs about 30 N at the paddles together (in the demo, 12.6 N and 18.2 N). A real YAM arm's working
push of about 20 N on one paddle gives the cork only about 130 N, but the simulated arm pushes harder. In the one-hand
check (the right doing its part exactly as in the demo, the left never moving), the right pushed its paddle with up to
32.5 N, which put 197 N on the rack and started the cork. It drew the cork 24 mm out before its hand fell behind the
paddle. Then it took the corkscrew off by its bell with the cork still 14 mm in the neck, which dragged the bottle up
to 2.9 cm in its cradle (it ended 2.4 cm from its place, past the 2 cm allowed), and the trial scored 0.5.

In a second one-hand check the right pressed its wing on toward the bottom of its arc instead of lifting off at
150 degrees. It again started the cork, with up to 32.6 N on its paddle, and drew it 34 mm out. Its hand then pushed
the wings the rest of the way down as it took the corkscrew by its bell, the cork came out with the corkscrew, and the
trial scored 1.0. So one arm can open this bottle in the simulation: the two-hand press in the demo is one way to do
the job, not a requirement.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cradle is a fixed body; the bottle, the cork and
the corkscrew's five parts (bell, rack, worm spindle and two wings) are loose rigid bodies made of shapes from
`bodykit.py`, at real size and mass, joined by PhysX joints authored in the task file.

- The cork's grip (the calibrated stand-in for a compressed cork's friction). The cork slides out along the neck on a
  slide joint whose drive is capped at the friction force and re-aimed after every control step so that it never pulls
  the cork back: a Coulomb friction. The force is 196 N while the whole 38 mm is in the neck and falls in proportion to
  the length still in it (friction on the contact length). 196 N is the mean extraction force that a published study
  measured for a natural cork in a bottle with this 17.7 mm bore; we did not measure it ourselves (see "The extraction
  force" below for the source, its method and the batch check). Once its inner end has passed the lip the cork is free
  of the neck. The cork weighs 10 g in the simulation, not its real 3.5 g: at 3.5 g, between the neck's hold (240 N in
  the previous calibration) and the worm's thread, PhysX's solver shared the load wrongly (the worm slid 12 mm out of
  the cork); at 10 g the cork started at the set force and the worm gave about 3 mm. The extra 6.5 g is nothing against
  the forces involved.
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
  the full 196 N the rack and each wing differ by about 5 mm of rack travel: the mesh's give.
- The paddles' push is read from the gear: each wing's drive torque over the paddle's 112.5 mm lever. In a debug run of
  the demo the cork started at 196.1 N on the rack, with 12.6 N and 18.2 N square to the two paddles (peaks 12.8 N and
  18.3 N); then the pull fell with the cork's grip as it came out.
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

## The extraction force: a published measurement, and the batch check

We did not measure the cork's extraction force ourselves. The 196 N is a published measurement:

- **Source.** Giunchi, A., Versari, A., Parpinello, G. P. and Galassi, S. (2008). Analysis of mechanical properties of
  cork stoppers and synthetic closures used for wine bottling. *Journal of Food Engineering* 88(4), 576-580.
  https://doi.org/10.1016/j.jfoodeng.2008.03.004
- **What they measured.** The peak force to pull each closure out of a 750 ml wine bottle with a 17.7 mm neck. For
  their natural cork (one piece of cork bark) it was 196 ± 20 N (mean ± SD), the lowest of the seven closures they
  tested (176-421 N).
- **How they measured it.** Following ISO 9727-5, 24 h after bottling, with a texture analyser (TA-HDi) fitted with a 4 mm corkscrew and a 500 N load cell, pulling at 5 mm/s (the standard's 300 mm/min).
- **What we could not find in the paper:** the cork's size, its surface treatment and the number of corks pulled.

How the task follows it:

- **Bottle.** The bottle has the source's 17.7 mm bore (not the more common 18.5 mm).
- **Cork.** A one-piece natural cork, 24 mm across. Its length, 38 mm, is our choice, since we could not find the
  source's: it is a standard length for 75 cl bottles, and short enough for the corkscrew's 45 mm stroke to draw it
  fully.
- **Force.** The simulated cork starts at the source's mean, 196 N, and the force falls in proportion to the length
  still in the neck.
- **Rest time.** Every trial bottle is corked at least 24 h before the trial, as in the measurement.

**Batch check (the measurement for the batch in use).** Before any real trials, the cork batch is checked in the
trial bottles, the way ISO 9727-5 pulls corks:

1. Fill five bottles of the trial type (17.7 mm bore) and cork them flush with new corks from the batch, using a
   four-jaw hand or bench corker.
2. Wait 24 h.
3. Pull each cork with a corkscrew on a motorised test stand with a digital force gauge, at 300 mm/min (5 mm/s), and
   record the peak force.
4. Accept the batch if the mean of the five lies within the source's 196 ± 20 N (176 to 216 N).

Otherwise the simulated grip is set to the batch's own mean, the demonstration is re-run, and the README and
`task.json` record the new figure.

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
in as the worm draws into the cork, until the worm is about 40 mm in and the wings are up. It backs off and comes down
beside the front wing's paddle while the left comes down beside the back one, and both push their paddles down along
their arcs together, the closed fingers turning with the paddles and aimed 4 cm ahead of them, until the wings are at
150 degrees and the cork is nearly out. The right lifts off; the left takes both wings the rest of the way down,
drawing the cork clear of the neck, and goes home. The right takes the corkscrew by its bell, draws it off the bottle
with the cork on its worm, lays it on the table to the right and goes home:

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

On one RTX 5090: about 18 minutes for a Dream-1 trial of 2,400 steps (1050 s on the clock, with Dream answering in 0.66 s on average), and about 27 minutes for the scripted demo (1,706 steps, 85.3 s of simulated time, 1615 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-DMFHER. Its review said:

> Construct validity (not met): The specified 240 N extraction force is drawn from a published range rather than
> measured for the selected cork and bottle combination.
>
> What to change: Measure the extraction force for the selected cork batch and bottle bore, and document the result and
> measurement method. Calibrate the simulated resistance to that measured force; if it differs materially from 240 N,
> update the simulation and provide a matching full-score demonstration.

The review before that (TK-BKSCBK) asked for a cork and bore that make a real interference fit, a calibrated extraction
force, the cork flush in the neck and the Societal relatability category; that was done in the previous revision and is
kept.

What changed in response:

- **The force is now a published measurement, not a range.** We could not measure corks ourselves. The force is now the
  measured mean that Giunchi, Versari, Parpinello and Galassi (Journal of Food Engineering 88(4), 576-580, 2008,
  doi:10.1016/j.jfoodeng.2008.03.004) report for a natural cork in a 750 ml bottle with a 17.7 mm neck: 196 ± 20 N
  (mean ± SD). They pulled it 24 h after bottling, as in ISO 9727-5, on a texture analyser with a 4 mm corkscrew and a
  500 N load cell, at 5 mm/s. The README and the setup state that this is their measurement, not ours.
- **The setup matches the measured combination where the source states it.**
  - The bottle's bore is now the source's 17.7 mm (it was 18.5 mm).
  - The cork is a one-piece natural cork, 24 mm across, corked flush.
  - We could not find the source's cork size, so the 38 mm length stays and is marked as our choice.
  - Trial bottles are corked at least 24 h ahead, as in the measurement (it was one hour).
- **Measurement method for the batch in use.** The setup and README now give the source's method, plus a batch check
  for real trials: five trial bottles corked with the batch, left 24 h, then each cork pulled with a corkscrew on a
  motorised test stand with a force gauge at 300 mm/min. The batch is accepted if the mean lies within 196 ± 20 N
  (176-216 N); otherwise the simulated grip is re-set to the batch's mean and the demo re-run.
- **Simulation re-calibrated and re-run.** 196 N is 18% below 240 N, so the simulated grip now starts at 196 N and falls
  with the length left in the neck, as before. In a debug run of the demo the cork started at 196.1 N on the rack, with
  12.6 N and 18.2 N at the two paddles, and the trial scored 1.0. The final demo run also scored 1.0, and the demo and
  Dream-1 videos are new. Dream-1 scored 0.0: it never touched the corkscrew.
- **One hand, reported plainly.** At 196 N one simulated arm can open the bottle, so the earlier claim that the job
  needs two hands, and the "bimanual" keyword, are removed. In the one-hand check (the right doing its part of the demo,
  the left idle) the right pushed its paddle with up to 32.5 N, more than a real YAM's working push of about 20 N. That
  put 197 N on the rack and drew the cork 24 mm out, and the trial scored 0.5. When the right instead pressed its wing
  on toward the bottom of its arc, it opened the bottle alone and scored 1.0.

# Replace a car's wiper blade: hold the sprung wiper arm off the windscreen while the other hand swaps the blade

The front of a car stands on the table: a black cowl, then the windscreen rising away from the arms between two red
pillars. The driver's wiper is parked along the bottom of the glass, its arm pivoting at the left and pressed onto the
glass by its spring; the old blade, its rubber worn grey, hangs on the hook at the arm's tip. In front of the car stand
a new blade in its cardboard tray and a long grey scrap tray. One hand lifts the wiper arm off the glass and holds it
up; the other slides the old blade along the arm toward the pivot, off its hook, and drops it in the scrap tray, then
takes the new blade, hooks it onto the arm and slides it toward the tip until it clicks in. The arm is lowered gently
back onto the glass and both arms go back to where they started. The task is one task file,
[`replace_a_wiper_blade_holding_its_arm_off_the_glass.py`](replace_a_wiper_blade_holding_its_arm_off_the_glass.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: holding a sprung part steady with one hand while the other works on it; unhooking and hooking a part by
sliding it along another; lowering a sprung part under control instead of letting it go.

Why two hands: the arm's spring presses it onto the glass, and the blade can only slide along the arm with the arm
lifted. Let go of the arm to work on the blade and it snaps down onto the glass, which scores 0. With one hand (the
right doing its part of the demo, the left never moving) the demo scored 0: the arm stayed on the glass and
the old blade stayed hooked on.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The car, the cardboard tray and the scrap tray are
kinematic, fixed where they are put; the wiper arm and the two blades are loose rigid bodies made of shapes from
`bodykit.py`.

- The arm: a hinge at its pivot (a `Hinge` from `bodykit.py`) about the forward line, with a spring pressing it toward
  the glass; it lifts up to 35 degrees. The arm and the car do not collide (they are joined); a blade on the glass
  holds the arm up, and without a blade the arm stops 6 degrees lower, its tip on the glass.
- The hook: a slide along the arm (a `Thread` from `bodykit.py`, a joint that keeps the blade on the arm's line and lets
  it slide along it). After every control step the task file follows each blade. A hooked blade is held by a spring
  whose hold follows it when it is pushed more than 3 mm past it, but only while the arm is lifted at least 8 degrees;
  slid 2 cm toward the pivot (arm lifted) the joint switches off and the blade comes free; slid to within 4 mm of the
  hook's end it clicks in. A free blade held in line with the arm (within 30 degrees), within 1.5 cm of the hook's line
  and 0.5 to 1.6 cm short of its end catches on (the joint switches on there). The hook takes one blade at a time.
- Snapping down: after every control step the task file reads the arm's angle and turning speed; an arm reaching
  within 4 degrees of resting on the glass faster than 1.5 rad/s has snapped down (an arm let go 15 degrees up reaches
  the glass at about 4 rad/s; lowered in the demo it moves at under 0.5 rad/s).

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the wiper arm and both blades still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or
1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=replace_a_wiper_blade_holding_its_arm_off_the_glass.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Replace the wiper blade: with one hand lift the wiper arm off the windscreen and hold it up; with the other slide the old blade along the arm toward its pivot, off the hook, and drop it in the grey scrap tray, then hook the new blade from the cardboard tray onto the arm and slide it toward the arm's tip until it clicks in. Lower the arm gently back onto the glass without letting it snap down, and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/wiper-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the wiper arm 7 cm from its pivot,
fingers closing across it, and lifts it 15 degrees along its swing, then holds it there. The right hand comes down on
the old blade 8 cm beyond its middle, past the arm's tip, fingers closing across it, slides it 3.6 cm along the arm
toward the pivot, off the hook, draws it back toward the arms, carries it over the scrap tray and lets it drop in. It
takes the new blade out of its tray the same way, carries it to the arm, hooks it on 1.1 cm short of the hook's end,
slides it along the arm toward the tip until it clicks in, lets go and goes back to where it started. The left lowers
the arm onto the glass, lets go and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=replace_a_wiper_blade_holding_its_arm_off_the_glass.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Replace the wiper blade: with one hand lift the wiper arm off the windscreen and hold it up; with the other slide the old blade along the arm toward its pivot, off the hook, and drop it in the grey scrap tray, then hook the new blade from the cardboard tray onto the arm and slide it toward the arm's tip until it clicks in. Lower the arm gently back onto the glass without letting it snap down, and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/wiper-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/wiper-dream/*.json; do inspect-robots video "$log"; done
```

Set `WIPER_DEBUG=1` to print what the score is built from twice a second: how far the arm is lifted and how fast it
turns, whether it has snapped down, where each blade is, whether it is hooked on and clicked in, how far it has slid
along the hook and how far it is off the hook's line, where each TCP is, and the bodies' speeds.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,000 steps (122 s on the clock, with Dream answering in 0.68 s on average), and about 1 minute for the scripted demo (648 steps, 32.4 s of simulated time, 86 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

# Hold a loose valve by its lever while pushing a hose onto its barbed tail and sliding the hose clamp over the barb

A light PVC ball valve lies loose on the table, its red lever toward the left arm and its barbed hose tail toward the right; in line with the barb, in two notched wooden rests, lies a length of green garden hose with a steel worm-drive clamp loose on it. One hand holds the valve by its lever while the other lifts the hose out of its rests, pushes it over the barb and then slides the clamp along the hose until it sits over the barb. Then the first hand lifts the valve by its lever to show the hose comes with it, sets it back down and lets go, and both arms go back to where they started. The task is one task file, [`push_a_hose_onto_a_valves_barb_and_clamp_it.py`](push_a_hose_onto_a_valves_barb_and_clamp_it.py), for the `yam_arena` embodiment (scene, scoring and the
scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: holding a loose part against the push of the other hand; pushing one part onto another in line, to a depth; a second push in order (the clamp after the hose); showing the joint holds.

Why two hands: the 80 g valve slides on the table at well under 1 N, and the barb takes about 3 N to push the hose
onto. With one hand (the right doing its part as in the demo, the left never moving) the hose caught on the barb's tip,
but pushing it shoved the valve 5 cm along the table without the hose going on, and pushing the clamp shoved the valve
and hose another 8 cm (0.0).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The rests are kinematic; the valve, the hose and the clamp are loose rigid bodies made of shapes from `bodykit.py`.

- The hose on the barb: a sliding joint (a `Thread` from `bodykit.py`, a PhysX D6 joint) between the valve and the hose, switched on when the hose's end meets the barb's tip in line (within 3.5 mm and 12 degrees). A spring holds the hose where it last stopped; after every control step, a hose pushed more than 1 mm past its hold has given, and the task file moves the hold there, so it takes about 3 N to push it further on, more than the 80 g valve's grip on the table.
- The clamp on the hose: a slider (a `Slider` from `bodykit.py`) along the hose, held the same way (about 1.5 N); its band is drawn solid, so it does not collide with the valve and slides over the barb. Once the hose is at least 2 cm onto the barb and the clamp's middle is 4 to 24 mm from the hose's end (over the barb), the hose is locked on (its hold made stiff).

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the valve, the hose and the clamp still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=push_a_hose_onto_a_valves_barb_and_clamp_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the green hose to the valve: hold the valve by its red lever with one hand, and with the other push the hose's end over the valve's barbed tail at least 2 cm, then slide the steel clamp along the hose until it sits over the barb. Then lift the valve by its lever to show the hose is on, set it back down, let go, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/hose-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the valve's lever grip, fingers closing across it, and holds the valve. The right takes the hose 6 cm from its end, lifts it out of its rests, lines it up with the barb and pushes it about 2.8 cm over the barb, lets go and rises. It comes down round the hose behind the clamp, closes its fingers round the hose (not round the wider clamp) and pushes the clamp along the hose until it sits over the barb, lets go, rises and goes back to where it started. The left lifts the valve 5 cm by its lever, the hose coming with it, holds it up, sets it back down, lets go and goes back to where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=push_a_hose_onto_a_valves_barb_and_clamp_it.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fit the green hose to the valve: hold the valve by its red lever with one hand, and with the other push the hose's end over the valve's barbed tail at least 2 cm, then slide the steel clamp along the hose until it sits over the barb. Then lift the valve by its lever to show the hose is on, set it back down, let go, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/hose-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/hose-dream/*.json; do inspect-robots video "$log"; done
```

Set `HOSE_DEBUG=1` to print what the score is built from twice a second: how far the hose's end is over the barb and off its axis, whether it is caught, how far the clamp's middle is behind the hose's end, whether it is clamped, how high the valve is lifted and whether that has been shown, whether the valve is down and both hands are clear, and where each TCP is from the valve's lever, the hose's end and the clamp.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (217 s on the clock, with Dream answering in 1.04 s on average), and about 2 minutes for the scripted demo (572 steps, 28.6 s of simulated time, 90 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

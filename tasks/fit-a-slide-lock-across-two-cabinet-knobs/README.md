# Childproof a two-door cabinet with a U-shaped slide lock across its knobs

A low two-door cabinet has a round knob on each door, near where the doors meet. The two parts of a U-shaped
cabinet slide lock lie on the floor in front of it. One arm picks up the U, turns it upright and slides its two rods
in behind both knob heads, one rod above and one below the knob stems, and then holds it there. The other arm picks
up the lock block, lines its two holes up with the two rod ends beyond the far knob, and slides it along the rods
until it is snug against that knob's stem. Both arms then let go.

The scene, scoring and scripted demo are in
[`fit_a_slide_lock_across_two_cabinet_knobs.py`](fit_a_slide_lock_across_two_cabinet_knobs.py), for the
`yam_arena` embodiment. It uses the helper [`bodykit.py`](bodykit.py), kept next to it. The helper builds the
multi-shape rigid bodies (the cabinet, the doors with their knobs, the U and the block) and the doors' spring hinges.

## Requirements

- [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), tested at commit `0059d02`.
- Isaac Lab-Arena 0.3.1 and Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU.

The repository's `setup.sh`, or the program's Brev launchable, installs everything, including the Dream-1 policy. The
task uses only boxes and cylinders, so it needs no models from Arena's object library.

## Run and score a trial

The scorer is `task_score`. A trial scores 1.0 once `success()` holds for 1 s, which needs all of these:

- the lock fitted snug across both knobs;
- both doors shut;
- both grippers clear of the lock;
- the lock still.

Otherwise the trial gets `score()` at the end of the episode: 0.75, 0.5, 0.25 or 0. `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fit_a_slide_lock_across_two_cabinet_knobs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Childproof this cabinet: fit the slide lock across the two door knobs so that the doors cannot be opened, then take your hands away." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/slide-lock-dream
```

The scripted demo reaches full score:

1. The left hand takes the U by its grip from above and lifts it.
2. The left hand turns the U upright with its rods pointing right. It brings the U beside the left knob, slides it in
   behind both knob heads until the grip is 4 mm from the left knob's stem, and holds it there.
3. The right hand takes the block from above, turns it upright and brings its holes in line with the rod ends.
4. The right hand pushes the block onto the rods and along them until it meets the right knob's stem.
5. Both hands let go and draw back.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fit_a_slide_lock_across_two_cabinet_knobs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Childproof this cabinet: fit the slide lock across the two door knobs so that the doors cannot be opened, then take your hands away." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/slide-lock-demo
```

To make videos (one MP4 per camera, next to each run's frames):

```bash
for log in ~/yam-eval/slide-lock-dream/*.json; do inspect-robots video "$log"; done
```

## Runtime per trial

On one L40S: about 3.3 minutes for a Dream-1 trial of 1,200 steps (199 s of episode time), plus about a minute
while Isaac Sim starts. The scripted demo takes 428 steps (21.4 s of simulated time) and about 104 s of episode time
when rendering at 1280 x 720. Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None are needed. With `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the loose parts shift.

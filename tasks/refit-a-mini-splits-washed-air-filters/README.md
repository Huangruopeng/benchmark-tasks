# Refit a mini-split's two washed air filters

A wall-mounted ductless mini-split's indoor unit has its front panel off and its filter bay empty. Its two washed
mesh air filters lie face up on the table in front of it. Each arm takes one filter by its pull tab and stands it up,
tilted top-in, in front of its bay. It slides the top edge in under the bay's lip, then swings the filter upright while
lifting it, so the top edge rises into the pocket and the bottom edge clears the two lugs. Finally it lets the filter
down behind the lugs onto the ledge and lets go.

The scene, scoring and scripted demo are in
[`refit_a_mini_splits_washed_air_filters.py`](refit_a_mini_splits_washed_air_filters.py), for the `yam_arena`
embodiment. It uses the helper [`bodykit.py`](bodykit.py), kept next to it, which builds the multi-shape rigid bodies:
the unit with its bay, pockets, ledges and lugs, and the two filters with their tabs.

## Requirements

- [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), tested at commit `0059d02`.
- Isaac Lab-Arena 0.3.1 and Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU.

The repository's `setup.sh`, or the program's Brev launchable, installs everything, including the Dream-1 policy. The
task uses only boxes, so it needs no models from Arena's object library.

## Run and score a trial

The scorer is `task_score`. A trial scores 1.0 once `success()` holds for 1 s. That needs both filters seated in
their bays, both grippers out of the unit, and both filters still.

Otherwise the trial gets `score()` at the end of the episode: 0.75, 0.5, 0.25 or 0. `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=refit_a_mini_splits_washed_air_filters.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the two clean air filters back into the air conditioner, one in each bay, then take your hands away." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/filters-dream
```

The scripted demo reaches full score. Both arms work at once, one per filter:

1. Take the filter by its tab from above, lift it and turn it upright, tilted 28 degrees top-in, in front of its bay.
2. Slide its top edge in under the lip.
3. Swing it upright while lifting it, so the top edge rises into the pocket and the bottom edge clears the lugs.
4. Let it down onto the ledge behind the lugs, let go, and draw back.
5. Return to the start.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=refit_a_mini_splits_washed_air_filters.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the two clean air filters back into the air conditioner, one in each bay, then take your hands away." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/filters-demo
```

To make videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/filters-dream/*.json; do inspect-robots video "$log"; done
```

## Runtime per trial

On one L40S, a Dream-1 trial of 1,200 steps takes about 2.7 minutes of episode time (164 s), plus about a minute
for Isaac Sim to start. The scripted demo takes 419 steps (21 s of simulated time), which is about 84 s of episode time
when rendering at 1280 x 720. On a new machine, Isaac Sim's first start adds a few minutes while it compiles its
shaders.

## Seeds

None are needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a
nonzero `object_jitter`, the trial's seed sets how far the loose filters shift.

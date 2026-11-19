# Seat a resident's lower denture on her lower ridge

A seated resident (a nursing-care training manikin) faces the robot with her mouth open. Her clean lower full denture
lies on a paper towel on the bedside table. The robot puts the denture back in her mouth and seats it on her lower
ridge: square and centred through the lips, back first onto the ridge either side of the tongue, then the front let
down so the channel sits on the ridge all round. Then it takes its hand away.

The scene, scoring and scripted demo are in [`seat_a_residents_lower_denture.py`](seat_a_residents_lower_denture.py),
for the `yam_arena` embodiment. It uses the helper [`bodykit.py`](bodykit.py), kept next to it, which builds:

- the head, with the inside of her mouth: ridge, tongue and upper denture;
- the lower lip, a sprung flap on a hinge;
- the denture, a horseshoe of boxes with a channel underneath;
- the bedside table and the denture cup.

## Requirements

- [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), tested at commit `0059d02`.
- Isaac Lab-Arena 0.3.1 and Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU.

The repository's `setup.sh`, or the program's Brev launchable, installs everything, including the Dream-1 policy. The
task uses only boxes, cylinders and spheres, so it needs no models from Arena's object library.

## Run and score a trial

The scorer is `task_score`. A trial scores 1.0 once `success()` holds for 1 s: the denture seated on the ridge, both
grippers out of her mouth, the denture still. Otherwise the trial gets `score()` at the end of the episode: 0.75, 0.5,
0.25 or 0. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the
same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=seat_a_residents_lower_denture.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the care manikin's clean lower denture back in its mouth, seated on its lower gum, then take your hand away." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/denture-dream
```

The scripted demo reaches full score with the right hand:

1. Take the denture by its front, with one finger above the front teeth and one under the front border where it
   overhangs the table's edge.
2. Carry it level to her mouth and slide it in, square and centred, above the ridge.
3. Pitch the hand down while lowering it, so the denture's back comes down onto the ridge first. The lower finger
   pushes the soft lower lip down.
4. Let go: the front drops, and the channel lands on the ridge all round.
5. Press the front teeth with the closed fingers, then draw the hand out and return it to its start.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=seat_a_residents_lower_denture.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the care manikin's clean lower denture back in its mouth, seated on its lower gum, then take your hand away." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/denture-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/denture-dream/*.json; do inspect-robots video "$log"; done
```

## Runtime per trial

On one L40S, a Dream-1 trial of 1,200 steps takes about 3.2 minutes of episode time (190 s), plus about a minute
for Isaac Sim to start. The scripted demo takes 533 steps (26.7 s of simulated time) and about 120 s of episode time
when rendering at 1280 x 720. Isaac Sim's first start on a new machine adds a few minutes while it compiles its
shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the loose denture shifts.

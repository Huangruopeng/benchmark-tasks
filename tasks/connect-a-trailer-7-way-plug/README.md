# Plug in a trailer's 7-way connector under its spring cover

One arm holds the trailer socket's spring cover up while the other takes the 7-way plug from its parking cup,
squares it to the socket and pushes it fully home; the cover then drops behind the plug's latch ear. The scene,
scoring and scripted demo are in [`connect_a_trailer_7_way_plug.py`](connect_a_trailer_7_way_plug.py), for the
`yam_arena` embodiment. It uses the helper [`bodykit.py`](bodykit.py), kept next to it. The helper builds the
multi-shape rigid bodies (wall and socket, cover, plug, holder), the cover's spring hinge and the cord's ball joints.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), tested at commit `0059d02`, on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The task uses only boxes, cylinders and simple meshes, so it needs
no models from Arena's object library.

## Run and score a trial

The scorer is `task_score`. A trial scores 1.0 once `success()` holds for 1 s: the plug fully home, the cover
latched behind its ear, both grippers off, all still. Otherwise the trial gets `score()` at the end of the episode:
0.75, 0.5, 0.25 or 0. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=connect_a_trailer_7_way_plug.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the black 7-way plug fully into the trailer's socket so that the socket's cover closes down behind the plug's latch ear, then let go of it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/plug-dream
```

The scripted demo reaches full score:

1. The left hand's closed fingertips lift the cover by its thumb lip to about 100 degrees and hold it there.
2. The right hand takes the plug by its body, draws it out of its cup and pushes it fully home under the cover.
3. The left slides out sideways, and the cover drops behind the plug's latch ear.
4. The right lets go, and both arms return to their start.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=connect_a_trailer_7_way_plug.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug the black 7-way plug fully into the trailer's socket so that the socket's cover closes down behind the plug's latch ear, then let go of it." \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/plug-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/plug-dream/*.json; do inspect-robots video "$log"; done
```

## Runtime per trial

On one L40S: about 6.5 minutes for a Dream-1 trial of 2,400 steps (393 s on the clock, including Isaac Sim's start),
and about 2.5 minutes for the scripted demo (549 steps, 27.5 s of simulated time; 144 s on the clock while rendering
at 1280 x 720). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects shift.

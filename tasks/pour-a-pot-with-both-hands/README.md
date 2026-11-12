# Lift a full 3 kg soup pot by both handles and pour the soup into a tureen

The skill is two arms lifting, carrying and tipping one heavy two-handled pot together and setting it back down. Both
arms lift a full pot of soup off a hob by its two flat loop handles, carry it to a tureen, tip it together to pour
the soup into the tureen without spilling, and set it back on the hob. The pot is a 3 L stainless pot (inside 18 cm
across and 12 cm deep, 1.2 kg empty) holding 1.8 kg of soup. The soup is a rigid-bead stand-in: 120 beads of 15 g, the
weight of 1.8 L of soup, that pour and roll but do not slosh or flow like liquid. Full, the pot weighs 3.0 kg. Held by
one handle it swings down about that handle, so the arms have to move as one. The size is chosen so the full pot, about
1.5 kg per arm, stays inside the YAM's ~2 kg per-arm payload, while one arm alone would have to carry all 3 kg about a
single handle. The task is one task file,
[`pour_a_pot_with_both_hands.py`](pour_a_pot_with_both_hands.py), for the `yam_arena` embodiment (scene, scoring and
the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The pot (floor, round wall, rim, two loop handles)
and each bead are loose rigid bodies built from several shapes, and the hob and the tureen are fixed bodies built the
same way, so `bodykit.py` adds such bodies: importing it hooks `yam_arena.scene` when the simulator starts, without
changing yam_arena, and gives `score()` each body's own pose. It also lets the scripted demo give a grip as a full
rotation, since pouring needs level grips that roll, not just straight-down ones.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the pot and every bead still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0 by the beads in the tureen, the beads spilled and
where the pot ends; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so
every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales
every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pour_a_pot_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Lift the full pot of soup (about 3 kg) off the hob by both handles, pour all of the soup into the white tureen without spilling, and set the pot back down on the hob." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/pot-dream
```

The scripted demo, which reaches full score: both arms turn their grips level in front of the handles, pointing at the
pot with their fingers one above and one below each handle's plane, slide back so the handle's outer bar comes between
the fingers (the fingertips inside the U) and close on it, so the lower finger carries the weight. They lift the pot
together, carry it over the tureen, roll both wrists to tip it 110 degrees about the line through the handles, tip it
back, carry it back, set it down on the hob and slide out forward. The robot's gains are the stock ones, so the loaded
arms sag 4-5 cm at the handles under the full pot; the waypoints aim 4 cm high while lifting and carrying, 2 cm while
pouring (the lip then dips just inside the tureen's rim) and 3 cm with the pot empty:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pour_a_pot_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Lift the full pot of soup (about 3 kg) off the hob by both handles, pour all of the soup into the white tureen without spilling, and set the pot back down on the hob." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/pot-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pot-dream/*.json; do inspect-robots video "$log"; done
```

Set `POT_DEBUG=1` to print what the score is built from (beads in the tureen, in the pot and spilled; the pot's tilt and
position) twice a second.

## Runtime per trial

On one RTX 5090: about 12 minutes for a Dream-1 trial of 1,200 steps (736 s on the clock, with Dream answering in 0.49 s on average), and about 10 minutes for the scripted demo (670 steps, 33.5 s of simulated time, 570 s on the clock). These were measured while other simulations shared the GPU, so a GPU of its own runs faster. Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the pot and the beads) shift.

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

## The real-trial head: a care-training manikin, or the low-cost printed rig

A real trial can use a nursing-care training manikin with removable full dentures (a Kyoto Kagaku oral-care trainer,
about $800), or the low-cost rig in [`fixture/`](fixture/), about $60 in materials. The rig's parts are written by
[`fixture/make_fixture.py`](fixture/make_fixture.py) from the task file's own boxes, so the printed mouth, ridge,
tongue and denture match the simulation exactly. Rerun it after any change to the task file's dimensions:

```bash
cd fixture && PYTHONPATH=<path to yam-isaac-lab-arena> python3 make_fixture.py
```

| File | Part | Print |
|---|---|---|
| `mouth_lower.stl` | The mouth's lower half, up to the mouth's top edge: chin, the pocket the lip sits in, the mouth's sides, floor, lower ridge, tongue, cheeks, throat. 130 x 150 x 108 mm. | PLA, 0.2 mm layers, 3 walls, 15% infill, as written (z up). |
| `mouth_upper.stl` | The upper half: upper lip, upper denture, nose, face above. 144 x 150 x 127 mm. | PLA, as written; supports under the nose. |
| `lower_denture.stl` | Her lower full denture: base, 13 mm channel, 2 mm inner and 4 mm outer flanges, teeth. 51 x 62 x 21 mm, 22.3 cm³. | Pink resin (26 g), or PLA at 100% infill (28 g); teeth up, supports under the channel. |
| `lip_mould.stl` | An open-top mould for the lower lip: a 12.4 x 64.4 x 31 mm cavity. | PLA. |
| `riser.stl` | A hollow block 115 mm tall that puts the mouth's centre 20 cm above the table. | PLA, 15% infill. |

Frame of the mouth parts: origin at the mouth's centre on the lips' front face, x back into the mouth, y to her left,
z up, in millimetres. The parts are unions of overlapping boxes, as in the simulation; slicers merge them (Cura's
"Union Overlapping Volumes" is on by default, PrusaSlicer merges them as they are).

Build:

1. Print the five parts. Optionally paint the ridge and tongue gum-pink and the upper denture white.
2. Glue the two halves together at their split face (z = 23 mm, the mouth's top edge), edges flush. Use CA glue or
   epoxy.
3. Cast the lower lip. Mix about 25 mL of Smooth-On Ecoflex 00-30 (1A:1B by volume; trial unit about $35) and pour it to
   the mould's brim. Cure it for 4 hours at room temperature, then demould: a soft block 12 x 64 x 31 mm.
4. Glue the lip's bottom 10 mm into the pocket in the chin with Smooth-On Sil-Poxy, its front flush with the lips'
   front. Its free 21 mm stands up between the mouth's sides, its top 2.6 cm below the mouth's centre.
5. Stand the head on the riser on the table, facing the arms on the centre line, its lips' front 35.5 cm forward of
   the line between the arm bases.

Check before the first trial:

- **Mouth opening:** 66 mm wide and 49 mm high, by calliper.
- **Lip:** a spring scale pressing 2 N on the lip's top edge pushes it down about 1 cm. If it is stiffer, shave its back
  face thinner. If it is softer, recast it in Ecoflex 00-50.
- **Denture fit:** set the denture on the ridge by hand. Its channel's ceiling rests on the crest, with about 2.5 mm of
  side play either way, and it does not rock when its front teeth are pressed.
- **Mass:** the denture weighs 25 to 28 g.

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

# Clamp two precast gels into an SDS-PAGE electrode assembly

A Bio-Rad Mini-PROTEAN Tetra electrode assembly stands on a black epoxy lab bench in its open position, its two green
clamping arms swung out. Two Mini-PROTEAN TGX precast gel cassettes rest on its gel supports, one on each side, short
plates facing in, each leaning 30 degrees away from the centre, which is where the maker's manual has you put them. The
job is the next step of every SDS-PAGE run: pull both cassettes flat against the green gaskets, hold them there, swing
the clamping arms over them one at a time until each locks, and let go. The mini tank, its lid and the box the gels came
in stand nearby. The task is one task file,
[`clamp_two_precast_gels_into_an_sds_page_electrode_assembly.py`](clamp_two_precast_gels_into_an_sds_page_electrode_assembly.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

The trial starts after the manual's step 1b. Precast cassettes come flat in their pouches; the person resetting the
trial takes off the comb and the green tape and sets the two cassettes on the gel supports, short plates facing the
gaskets. The robot does the rest of the maker's sequence: steps 1c and 1d, the squeeze and the lock.

What it tests: keeping two loose plates squared and flat against a seal, under a controlled squeeze, while a third part
is locked over them. The defining difficulty, beyond pressing a part until it clicks:

- both cassettes fall back to their 30-degree lean as soon as nothing holds them;
- one squeeze must hold both flat against the gaskets at once (one jaw spans both, 3.9 cm across the pair);
- a clamping arm jams 20 degrees short of shut unless both cassettes lie within 3 mm of the gaskets at its end;
- the two arms lock in turn, the first while the squeeze holds, each with a 5 N push that must not shove the
  cassettes off their gaskets.

Why two hands: a cassette let go of falls back to its 30-degree lean, and an arm closes only over two cassettes lying
flat at once, so one hand has to hold both flat while the other pushes the arm. The two resting cassettes' top corners
are 12.4 cm apart and the YAM jaw opens 9.39 cm, so one jaw takes the pair only once one cassette has been brought
flat. With one hand (that hand doing exactly its part of the demo, the other never moving) the score was 0 both ways.
With the right hand alone, it pushed the near cassette flat. When it let go to push its arm, the cassette fell back to
30.4 degrees, and the arm jammed at 19.8 degrees and swung back open. With the left hand alone, its squeeze brought the
far cassette flat, but the near one still leaned at 30.4 degrees and its arm jammed at 19.8 degrees too. In the demo the
two wrists stay at least 20.0 cm apart while the first arm locks under the squeeze, and at least 18.4 cm apart
throughout.

## The real product and the maker's procedure

The manual's assembly steps, word for word:

- Mini-PROTEAN Tetra Cell Instruction Manual, bulletin 10007296 Rev D, section 2.2, step 1:
  - "b. Place the first gel sandwich or gel cassette (with the short plate facing inward) onto the gel supports; gel
    supports are molded into the bottom of the clamping frame assembly; there are two supports in each side of the
    assembly. Note that the gel will now rest at a 30° angle, tilting away from the center of the clamping frame."
  - "c. Using one hand, gently pull both gels towards each other, making sure that they rest firmly and squarely
    against the green gaskets that are built into the clamping frame; make certain that the short plates sit just below
    the notch at the top of the green gasket."
  - "d. While gently squeezing the gel sandwiches or cassettes against the green gaskets with one hand (keeping
    constant pressure and both gels firmly held in place), slide the green arms of the clamping frame over the gels,
    locking them into place."
  - "Important Note: Do not attempt to lock the green arms of the clamping frame, without first ensuring that the gel
    cassettes are perfectly aligned and stabilized against the notches on the green gaskets of the module. To prevent
    the gels from shifting during the locking step, firmly and evenly grip them in place against the core of the module
    with one hand."
- Mini-PROTEAN Precast Gels Instruction Manual and Application Guide, bulletin 1658100, section 2.3:
  - "5. While gently squeezing the gel cassettes (or cassette and buffer dam) against the green gaskets (maintaining
    constant pressure and with both gels in place), slide the green arms of the clamping frame one at a time over the
    gels, locking them into place (D,E)."
  - "6. The wing clamps of the electrode assembly lift each gel cassette up against the notch in the green gasket,
    forming a seal. Check again that the short plates sit just below the notch at the top of the green gasket (C)."
  - Its troubleshooting table lists the cause of "Buffer leaking from inner chamber" as improper assembly: "Top edge
    of short plate should fit under notch at top of gasket" and "Top of short plate should touch green gasket". These
    two checks are the task's real-world test of a sealed assembly.

The clamping arms are Bio-Rad's compact cell clamp (US patent 6,436,262): each arm is a C-shaped clamp on a pivot pin
at the base of one end of the frame; swung up, its two walls close over the side edges of both cassettes and press them
against the gaskets, and it passes over centre to stay shut.

Products, part numbers and prices (retrieved 2026-11-17):

| Item | Part number | Price |
|---|---|---|
| Mini-PROTEAN Tetra electrode assembly, with its clamping arms and gaskets | Bio-Rad 165-8037 (manual, section 8) | used: US $350 (eBay item 157682281677) |
| Mini-PROTEAN Tetra Vertical Electrophoresis Cell for Mini Precast Gels, 2-gel (electrode assembly, tank, lid with leads, buffer dam) | Bio-Rad 1658005 | new: US $509.00 (Flinn Scientific FB2337) |
| Mini-PROTEAN TGX precast gels, 10%, 10-well (cassette 8.5 x 10 cm, 1.0 mm gel), 10 per box | Bio-Rad 4561034 | US $139.00 per box (Flinn Scientific FB2482) |

The trial uses only the electrode assembly and two cassettes. The cheapest route is a used electrode assembly
(US $350), or the new 2-gel cell (US $509), plus one box of gels; two cassettes serve every trial, since nothing is
run. No power supply is needed: the trial is the dry assembly, before any buffer goes in.

Handling, for a shared rig: the trial is dry. No running buffer, no water, no acrylamide solution is poured. The
precast gel is polymerised and sealed between the cassette's two plastic plates; the comb and the green tape are taken
off beforehand (the maker's step 1), the wells are rinsed with water and drained, and the outside of each cassette is
wiped dry. The grippers touch only the dry plastic plates and the plastic arms.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The bench, the electrode assembly's frame, the tank,
its lid and the gel box are kinematic, fixed where they are put; the two clamping arms and the two cassettes are loose
rigid bodies made of shapes from `bodykit.py`, at their real sizes and masses (cassettes 40 g, arms 30 g).

- The frame: a glass-filled PBT core 2.5 cm thick with an open inner chamber, a green U gasket 1.5 mm proud on each
  face (legs 8 mm wide, each with a 2 mm shoulder over the notch at its top, 9.05 cm above the bench), red and black
  banana plugs, and two gel supports per side. Each support is a seat sloping down 12 degrees outward from the gasket,
  with a 2 mm lip and a back rest along a resting cassette's outer face: a cassette set on it leans 30 degrees out, and
  one pulled flat against the gasket and let go falls back to that lean.
- The cassettes: a short plate (inside, 7.5 cm tall) and a long plate (outside, 8.5 cm tall) with the gel between,
  10 cm wide and 5.5 mm thick in all, colliding as one slab up to the short plate's top and as the long plate above it.
- The clamping arms: each a C-shaped clamp (a back plate and two walls that close over the last 8 mm of both
  cassettes' side edges, their leading 4 mm sloping out 3.5 mm as a lead-in), turning on a PhysX revolute joint about
  a pivot 1 cm above the bench at its end of the frame, from 35 degrees open to 0 (closed). A spring drive capped at
  0.45 N m holds it open and resists closing (5 N at its thumb tab, 9 cm from the pivot); within 6 degrees of closed it
  passes over centre and the drive presses it shut against its stop with the same cap, so a locked arm takes
  0.45 N m to pull open. The arm's walls collide with the cassettes, so a locked arm holds them flat.
- The rule for the walls meeting a leaning cassette: an arm closes past 20 degrees from shut, where its walls reach the
  cassettes' edges, only while both cassettes lie within 3 mm of the gaskets at its end; otherwise the task file holds
  the joint's limit at 20 degrees, as the walls' leading edges would stop against the leaning cassette's edge. The
  arms' pins that lift the cassettes into the notch are not modelled: a cassette sits on its seat at the sealing
  height.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Scoring

A cassette is seated when its short plate lies flat on the gasket (within 1 mm at the bottom and at the short plate's
top, at both ends), facing the right way, with its bottom on its seat so that the short plate's top sits 0-3 mm below
the gasket's notch. All judged at the end of the trial, on live states:

- 1.0: both arms locked, both cassettes seated, and both robot arms back where they started (each gripper's fingertip
  centre within 5 cm of its start);
- 0.75: both arms locked and both cassettes seated;
- 0.5: one arm locked, with both cassettes seated;
- 0.25: both cassettes lay flat on their gaskets at the same moment at some point in the trial;
- 0.0 otherwise, and 0.0 for the whole trial once a cassette has fallen onto the bench.

In a real trial: an arm is locked when it stands upright against its end of the frame, snapped over centre (photo); a
cassette is seated when, looking down at each notch, the top of its short plate touches the green gasket just under
the notch (the maker's two leak checks) and a 1 mm feeler gauge does not enter between the short plate and the gasket
at either notch; the 0.25 tier and a fall are judged from the video; the robot arms are checked against marks at their
start. The trial ends at the run limit, or once the full score has held for 1 s with both cassettes and both clamping
arms still.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with both cassettes and both clamping arms still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same layout.
The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it,
so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clamp_two_precast_gels_into_an_sds_page_electrode_assembly.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Clamp both gels into the electrode assembly, ready to run. Finish with both arms back where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sds-dream
```

The scripted demo, which reaches full score: the right hand, its fingers closed, pushes the near cassette up flat
against its gasket and holds it. The left comes down over both cassettes with its jaw fully open (its near finger just
outside the flat near cassette, its far fingertip just past the leaning far cassette's top corner) and closes while
moving in, so its far finger brings the far cassette up flat; it squeezes the pair against the gaskets. The right lets
go of the near cassette and pushes the right-hand clamping arm up over the cassettes' edges until it locks; the wrists
stay at least 20 cm apart throughout. The left lets go, rises and pushes the other arm shut the same way. Both arms go
back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clamp_two_precast_gels_into_an_sds_page_electrode_assembly.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Clamp both gels into the electrode assembly, ready to run. Finish with both arms back where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sds-demo
```

`SDS_ONE_HAND=right` (or `left`) runs the same demo with that hand only, the other never moving.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/sds-dream/*.json; do inspect-robots video "$log"; done
```

Set `SDS_DEBUG=1` to print what the score is built from four times a second: each arm's angle, state and limit, each
cassette's lean, its height against its seat, how far its short plate lies off the gasket at each end, whether it is
seated, the finger openings, where each TCP is and how far apart the wrists are.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 2,400 steps (340 s on the clock, with Dream answering in 0.71 s on average), and about 2 minutes for the scripted demo (601 steps, 30.1 s of simulated time, 100 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Sources

- Mini-PROTEAN Tetra Cell Instruction Manual, bulletin 10007296 Rev D (Bio-Rad; text copy:
  https://manualsdir.com/manuals/600180/bio-rad-mini-protean-tetra-cell-model-225-tube-gel-casting-stand-protean-ii-xi-2-d-tube-gel-cell-mini-protean-2-d-electrophoresis-cell-mini-protean-tetra-handcast-systems.html?page=14),
  pages 10-11 (assembly), 4 (specifications) and 33 (replacement parts).
- Mini-PROTEAN Precast Gels Instruction Manual and Application Guide, bulletin 1658100 (Bio-Rad; PDF copy:
  https://am.dia-m.ru/upload/iblock/826/gotovye_geli_mini_protean_bio_rad.pdf), section 2.3 and the troubleshooting
  table; cassette 8.5 x 10 cm, styrene copolymer, gel 1.0 mm (section 1.4).
- Bio-Rad 1658005 product page:
  https://www.bio-rad.com/en-uk/sku/1658005-mini-protean-tetra-vertical-electrophoresis-cell-for-mini-precast-gels-2-gel?ID=1658005
- Flinn Scientific FB2337 (2-gel cell, US $509.00): https://www.flinnsci.com/mini-protean-tetra-cell-for-2-precast-gels/fb2337
- Flinn Scientific FB2482 (10% Mini-PROTEAN TGX gels, 10-well, US $139.00):
  https://www.flinnsci.com/10-mini-protean-tgx-precast-gel-10-well/fb2482
- Used electrode assembly, US $350: https://www.ebay.de/itm/157682281677
- US patent 6,436,262, "Compact cell clamp for slab gel plate assembly" (Bio-Rad):
  https://patents.google.com/patent/US6436262

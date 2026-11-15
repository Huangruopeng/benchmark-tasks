# Water a house plant at its roots without wetting its leaves

A house plant in a terracotta pot spreads its broad leaves low over the soil: three in the crown over the back and
sides, and one big leaf in front on a springy stalk, lying over the front of the soil. A blue watering jug with water
in it stands on a stool beside the pot. One hand pushes the big leaf round on its stalk, aside, and holds it there;
the other lifts the jug, carries it over the front of the pot and pours the water onto the soil the leaf covered,
without wetting any leaf. The jug goes back on its stool, the leaf is let go back over the soil, and both arms go back
to where they started. The task is one task file,
[`water_a_plant_holding_its_big_leaf_aside.py`](water_a_plant_holding_its_big_leaf_aside.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: holding a sprung part out of the way while the other hand works where it was; pouring a light stream
into a target without spilling it on what is around it; putting everything back.

Why two hands: the big leaf springs back over the soil as soon as nothing holds it, and the soil it covers is the only
soil not under a leaf, so water poured with the leaf in place lands on the leaf. With one hand (the left pouring as in
the demo, the right never moving) the demo scored 0: all 12 beads landed on the big leaf, which lay over the
soil, and ran off it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The pot, its soil, the stem and the crown's three leaves are one fixed body. Each leaf is a thin rigid blade (seven
  boxes giving its oval outline) 15 cm long.
- The big front leaf is a loose rigid blade joined to the stem by a hinge about the vertical (`Hinge` from
  `bodykit.py`), its spring (0.0016 N m per degree) holding it over the front of the soil; pushed round 85 degrees it
  pushes back with 0.14 N m.
- The water is 12 rigid beads in the jug, which is a cup built from a floor and 16 wall panels.
- After every physics step (240 a second, not only after each 50 ms control step), the task file checks each bead:
  within 4 mm of any leaf's blade, it has wetted that leaf for good, even if it runs off straight away. A bead counts
  as watering the soil only if it ends lying on the soil inside the pot's wall (its centre within 7.25 cm of the pot's
  axis and between 3 mm below and 15 mm above the soil's surface) and never touched a leaf. The saucer is drawn only:
  a bead that misses the pot or bounces out of it lands on the table under the saucer and never counts.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the jug, the big leaf and every bead still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=water_a_plant_holding_its_big_leaf_aside.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Water the plant at its roots, onto the soil, without wetting its leaves. Then put the jug back on its stool, leave the plant's leaves as they were and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/plant-dream
```

The scripted demo, which reaches full score: the right hand comes down beside the big leaf's left edge, fingers
closed, pushes the leaf round on its stalk 85 degrees, until it points to the right, clear of the front of the soil,
and holds it there. The left comes in from the side on the jug, fingers closing across it low down, lifts it, carries
it over the front of the pot and tips it out onto the soil the leaf covered. It sets the jug back on its stool, lets
go and goes back to where it started; the right lets the leaf go back round over the soil, rises and goes back to
where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=water_a_plant_holding_its_big_leaf_aside.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Water the plant at its roots, onto the soil, without wetting its leaves. Then put the jug back on its stool, leave the plant's leaves as they were and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/plant-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/plant-dream/*.json; do inspect-robots video "$log"; done
```

Set `PLANT_DEBUG=1` to print what the score is built from twice a second: how far round the big leaf is, how many
beads lie on the soil and how many have touched a leaf, how many are still in the jug, where the jug is and how far
it is tipped, whether the jug and the leaf are back, whether both hands are clear, and where each TCP is.

## A real trial

The plant is artificial, built to the simulated sizes, so every trial has the same leaves in the same places and the
leaves themselves record the water that touches them. Its leaves are cut from green absorbent cellulose cloth (a few
grams each, holding over 20 ml), each clipped to a wire stalk: three over the back and sides of the pot and the big
lower leaf on a sprung wire stalk lying over the front of the soil, which springs back when let go. It stands in real
potting soil, just moist, in a 17 cm glazed terracotta pot on a saucer; a 150 ml watering jug holding 120 ml of
room-temperature water stands on a 6 cm stool beside it. The two quantities are measured separately (1 g of water =
1 ml):

- **Water delivered to the soil.** Plug the pot's drainage hole with a rubber bung for the trial. Before it, lift
  the potted plant out of its saucer and weigh it, and the dry saucer, on a kitchen scale (1 g steps). After it,
  unclip the cloth leaves, lift the pot out of its saucer again and weigh each. The pot's gain is the water delivered
  to the soil: with the drain plugged, and 1.3 cm of headroom above the soil (about 235 ml), none of it leaves the
  pot. The saucer's gain is spilled water and never counts as delivered.
- **Water that touched the leaves.** The leaves are absorbent cloth, so a drop that touches one soaks in where it
  lands and is recorded even if it would have run off a real leaf. Before the trial each leaf is weighed dry; after
  it, each is unclipped into its own pre-weighed sealed bag and weighed on a 0.01 g scale. Their total gain is the
  water that touched the leaves.

The thresholds are the simulation's, at 10 ml per bead:

| Score | Real trial | Simulation |
|---|---|---|
| 1.0 | pot gain at least 90 g, cloth leaves gain at most 10 g, and the end state (marks on the stool and the pot's rim, ruler and protractor; the robot's joint log for the arms) | at least 9 of the 12 beads on the soil, at most 1 ever within 4 mm of a leaf, and the end state |
| 0.5 | pot gain at least 90 g, cloth leaves gain at most 10 g | at least 9 beads on the soil, at most 1 on a leaf |
| 0.25 | pot gain at least 40 g | at least 4 beads on the soil |

Water in the saucer, on the table or left in the jug never counts. Reset by pulling the bung and letting the pot
drain back to its starting weight, emptying and drying the saucer, clipping on dry pre-weighed cloth leaves and
refilling the jug.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (514 s on the clock, with Dream answering in 0.71 s on average), and about 6 minutes for the scripted demo (607 steps, 30.4 s of simulated time, 335 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-A2TMY6. Its review said:

> Scoring (not met): Plant-and-saucer weight and final leaf blotting cannot distinguish water delivered to soil from saucer spills or reliably measure water that previously ran off leaves.
>
> What to change: The real-world scoring still needs to distinguish water delivered to the soil from water poured into the saucer, and to measure water contacting leaves even if it runs off before the final blot. Specify a practical measurement procedure for both quantities and apply the existing numeric thresholds to those measurements; retain the beads as the simulation approximation.

The review before that (TK-J457LS) asked for a reproducible real plant, a real water volume with a reset procedure,
and real-world water thresholds for every tier; that was done in the previous revision and is kept.

What changed in response:

- **Soil versus saucer.** The pot and the saucer are now weighed separately, each against its own weight before the
  trial: the pot is lifted out of its saucer, its drainage hole plugged with a rubber bung for the trial so no
  water drains through, and the 1.3 cm of headroom above the soil holds about 235 ml. Only the pot's gain counts as
  delivered; the saucer's gain is spilled water and never counts.
- **Water that touched a leaf, even if it ran off.** The final blot is gone. The real-trial plant is now an
  artificial one built to the simulated sizes, its leaves cut from green absorbent cellulose cloth (holding over
  20 ml each) on wire stalks, the big one on a sprung stalk. A drop that touches a leaf soaks in where it lands, so
  none can run off unrecorded; each leaf is weighed dry before the trial and, unclipped into a sealed bag, after it
  (0.01 g scale). An artificial plant also gives every trial the same leaf layout, which keeps the earlier review's
  request for a reproducible plant.
- **The existing thresholds apply to these measurements:** 1.0 and 0.5 need a pot gain of at least 90 g and at
  most 10 g gained by the cloth leaves (9 of the 12 beads on the soil, at most 1 bead on a leaf); 0.25 needs a pot gain of at least
  40 g (4 beads). The beads stay the simulation approximation, 10 ml each.
- **The simulation already kept the two quantities apart; it is now stated with its numbers, and the leaf check
  runs every physics step.** A bead counts as delivered only if it ends on the soil inside the pot's wall (within
  7.25 cm of its axis, at the soil's height) and never touched a leaf; the saucer is drawn only, so beads outside
  the pot never count. A bead that ever comes within 4 mm of a leaf counts as leaf water for good; that check now
  runs after every physics step (240 a second) instead of every 50 ms control step, so a bead that only grazes a
  leaf is caught too. The demo was re-run under it: 11 of 12 beads on the soil, none on a leaf, score 1.0.

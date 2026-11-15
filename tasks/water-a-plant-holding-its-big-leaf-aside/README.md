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
leaves themselves record the water that touches them.

- **The leaves.** Each of the four leaves is cut from white cellulose sponge cloth, the Swedish-dishcloth type (70%
  cellulose, 30% cotton), whose makers rate it to absorb up to 15 times its own weight. A 17.5 x 20 cm cloth weighs
  about 9 g. Each leaf is four layers cut to the 141 cm² blade outline and stitched round the edge: 4 x 141 cm² x 9 g /
  350 cm² = about 14.5 g dry, so it holds up to about 217 ml, more than the whole 120 ml. Each is clipped to a wire
  stalk: three over the back and sides of the pot, and the big lower leaf on a sprung wire stalk lying over the front
  of the soil, which springs back when let go. (The simulation draws the leaves green.)
- **The rest.** Real potting soil, just moist, in a 17 cm glazed terracotta pot on a saucer, its drainage hole plugged
  with a rubber bung for the trial. A 150 ml watering jug holding 120 ml of room-temperature water, tinted dark blue
  with food colouring (about 1 ml per litre), stands on a 6 cm stool beside it.

The two quantities are measured separately (1 g of water = 1 ml).

**Water delivered to the soil.** The pot is weighed in the same state before and after the trial: all four cloth leaves
unclipped, the soil, stem, wire stalks, clips and bung left in.

1. Before the trial, unclip the leaves, lift the pot out of its saucer and weigh it, and the dry saucer, on a kitchen
   scale (1 g steps). Clip the leaves back on.
2. After the trial, unclip the leaves again (into their bags, below), lift the pot out, let it drip into the saucer,
   and weigh the pot and the saucer.

The pot's gain is the water delivered to the soil. With the drain plugged, and 1.3 cm of headroom above the soil
(about 235 ml), none of it leaves the pot. The saucer's gain is spilled water and never counts as delivered.

**Water that touched the leaves.** It is recorded two ways:

- **By weight.** Each leaf is weighed dry before the trial. After it, each is unclipped into its own pre-weighed clear
  sealed bag and weighed on a 0.01 g scale. Their total gain is the leaf water. Each leaf holds more than the whole
  120 ml, so none can be filled past what it holds, even if all the water lands on it.
- **By stain.** The water is dyed and the cloth is white, so every drop that touches a leaf leaves a blue stain where
  it touched. That holds however fast it came and whether or not the cloth kept it. Every leaf is photographed on both
  faces through its bag.

Any stain counts as leaf contact, and it is scored with the weights. A stain that runs over a leaf's edge means water
left that leaf without being weighed, so it counts as more than 10 ml, whatever the leaves gained.

**Validating the leaves before trials.** This is a procedure for the trial's operator to run; we have not run it or
measured its result.

1. Clip a dry, weighed leaf on its stalk at its trial angle, over a pre-weighed tray.
2. Pour 10 ml onto its middle from the trial jug, held 5 cm above it and tipped over as fast as it will go (the
   fastest pour the jug gives).
3. Do the same with 120 ml onto a second dry leaf.
4. Weigh the leaves and the tray, and photograph the leaves. The leaves pass if each keeps at least 98% of what was
   poured (a gain of at least 9.8 g and 117.6 g; the tray catches the rest), each shows a stain, and the 10 ml stain
   does not reach the edge.
5. If a leaf fails, give every leaf another layer of cloth and repeat.

The thresholds are the simulation's, at 10 ml per bead:

| Score | Real trial | Simulation |
|---|---|---|
| 1.0 | pot gain at least 90 g; cloth leaves gain at most 10 g, with no stain running over an edge; and the end state (marks on the stool and the pot's rim, ruler and protractor; the robot's joint log for the arms) | at least 9 of the 12 beads on the soil, at most 1 ever within 4 mm of a leaf, and the end state |
| 0.5 | pot gain at least 90 g; cloth leaves gain at most 10 g, with no stain running over an edge | at least 9 beads on the soil, at most 1 on a leaf |
| 0.25 | pot gain at least 40 g | at least 4 beads on the soil |

Water in the saucer, on the table or left in the jug never counts. To reset, pull the bung and let the pot drain, then
plug it again; it is weighed afresh, leaves unclipped, before each trial. Empty and dry the saucer, clip on fresh, dry,
unstained, pre-weighed cloth leaves, lay the big leaf back over the soil, and refill the jug with 120 ml of the dyed
water.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (514 s on the clock, with Dream answering in 0.71 s on average), and about 6 minutes for the scripted demo (607 steps, 30.4 s of simulated time, 335 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-EYKT6P. Its review said:

> **Not accepted** (TK-EYKT6P).
>
> Scoring (not met): The pot weighings use different leaf configurations, and cloth capacity alone does not establish
> that all water contacting a leaf is captured.
>
> What to change: Weigh the pot with the same leaves removed both before and after the trial, or explicitly correct
> for their dry mass, so its gain measures water delivered to the soil. Also specify and validate a practical method
> that records water contacting leaves even during a fast pour or after a leaf's absorbent capacity is exceeded; apply
> the existing soil and leaf-water thresholds to those measurements.

The review before that (TK-A2TMY6) asked to tell water delivered to the soil from water in the saucer, and to measure
water that touched a leaf even if it ran off; that was done in the previous revision (pot and saucer weighed apart,
drain plugged, absorbent cloth leaves weighed) and is kept. The first review (TK-J457LS) asked for a reproducible real
plant, a real water volume with a reset procedure, and real-world thresholds for every tier; that is kept too.

What changed in response:

- **Scoring (not met), the pot weighings:** the pot is now weighed in the same state both times, with all four cloth
  leaves unclipped. Before the trial the leaves are unclipped, the pot (with its soil, stem, wire stalks, clips and
  bung) is weighed, and the leaves are clipped back on. After it they are unclipped again before the pot is weighed. So
  the pot's gain is the water delivered to the soil, with no leaf mass in either weighing.
- **Scoring (not met), capacity:** each leaf now holds more than the whole 120 ml. It is four layers of white
  cellulose sponge cloth, which its makers rate at up to 15 times its own weight. Cut to the 141 cm² blade, it weighs
  about 14.5 g (a 17.5 x 20 cm cloth weighs about 9 g) and holds up to about 217 ml. So no leaf can be filled past what
  it holds in a trial.
- **A method that records leaf contact even during a fast pour or past a leaf's capacity:** the water is tinted dark
  blue with food colouring and the leaves are white. Every drop that touches a leaf leaves a stain, whether or not the
  cloth keeps it, and each leaf is photographed on both faces after the trial. This record does not depend on the
  cloth's capacity or on how fast the water came.
- **Validating it:** the README and the scoring now give a check for the trial's operator to run before trials. It is
  a procedure to run, not a result we have measured. Pour 10 ml, and 120 ml onto a second leaf, from the trial jug at
  its fastest pour (held 5 cm above, tipped over as fast as it goes), onto a dry leaf clipped at its trial angle over a
  pre-weighed tray. Each leaf must keep at least 98% by weight (at least 9.8 g and 117.6 g) and show a stain, and the
  10 ml stain must not reach the edge. A failing leaf design gets another layer of cloth and is checked again.
- **The existing thresholds apply to these measurements:** the pot's gain goes against the soil thresholds (at least
  90 g for 1.0 and 0.5, at least 40 g for 0.25). The leaves' total weight gain goes against the leaf-water threshold
  (at most 10 g for 1.0 and 0.5). Any stain counts as leaf contact and is scored with the weights. A stain running over
  a leaf's edge means water left it unweighed, so it counts as more than 10 ml. In the simulation these remain at least
  9 (or 4) of the 12 beads on the soil and at most 1 bead ever within 4 mm of a leaf, at 10 ml per bead.
- **Simulation unchanged:** only the text changed (setup, scoring, reset notes, the fidelity note at 490 characters,
  and this README). The scene, scorer, demo and instruction are as before, so the demo and Dream-1 runs and their
  scores still apply.

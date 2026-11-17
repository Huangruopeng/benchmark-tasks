# Empty the sink strainer into the bin

A stainless single-bowl kitchen sink is set into the counter on the right. It is a 25 x 22 in top-mount sink with a
bowl 21 x 15-3/4 in and 20 cm deep, and a gooseneck tap stands on its back deck. In its 3-1/2 in drain sits a
fixed-post strainer basket, 79.4 mm across and 68 g, holding nine food scraps: two strips of carrot peel, a piece of
potato peel, two clumps of cooked rice, two peas, a piece of broccoli and a piece of apple core. In a real trial the
scraps are clean, dry stand-ins, 3D-printed in ABS to the scraps' sizes and masses and reused from trial to trial (see
"The scraps in a real trial"). A 1.8 l food-waste caddy with its lid off stands on the counter just left of the sink.

The robot is told: "Empty the food scraps from the sink's drain strainer into the food-waste caddy and put the strainer
back in the drain, without letting any scraps go down the drain or onto the counter."

In the scripted demo:
1. The right hand reaches into the bowl, grips the basket's knob and lifts the basket out of the strainer body and up
   out of the bowl.
2. The left hand takes the basket's cup from the front, across its rim. The right lets go and moves clear.
3. The left alone carries the basket over the caddy, turns it over (104 degrees about its own axis, a wrist roll) and
   shakes it twice. The scraps fall into the caddy.
4. The left turns it back up and brings it back toward the sink.
5. The right takes the knob again and the left lets go.
6. The right lowers the basket into the strainer body, presses it home and lets go.
7. Both arms go back to where they started.

The task is one task file, [`empty_the_sink_strainer_into_the_bin.py`](empty_the_sink_strainer_into_the_bin.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests:
- Reaching 20 cm down into a real-depth sink to take a small, loose part by its 16 mm knob.
- Carrying the basket level, and emptying it by turning it over a small bin so the scraps fall in. No door or latch
  does the emptying.
- Keeping scraps out of the open drain and off the counter while the basket is out.
- Re-seating the basket in an 82 mm bore with about 1 mm to spare, its post in the crossbar's hub.

## How this differs from the board

The nearest tasks on the board are #1262 (empty a robot vacuum's dustbin over the waste box),
#1235 (empty a vacuum's dust cup through its bottom flap into the bin and click it back in) and #735 (lift a pool
skimmer basket out of its well by its folded handle).

- **#1262 and #1235.** Both empty a dust container into a bin and put it back. There, a held button or a flap opens
  doors that do the emptying, and the container clicks into a seat it sits on.
- **#735.** That skimmer basket is lifted by a large handle out of an open well; it is not emptied or re-seated.
- **This task.**
  - A loose basket is fished out of a 3-1/2 in drain 20 cm down a full-depth kitchen sink, by a 16 mm knob.
  - It is emptied only by turning it over the caddy.
  - It must be re-seated in a bore with about 1 mm to spare, its post in a hub.
  - The drain stands open while the basket is out, and any scrap that goes down it ends the trial at 0.

Other sink tasks on the board are about different jobs:
- #1857 and #1859 reconnect a bathroom basin's pop-up stopper.
- #630 puts a mug to soak in the right sink, and #860 gets a hair dryer out of a filled sink.
- #1251 unclogs a double sink with a plunger.
- #2058 spans a colander across a basin, and #2092 plunges a sink.
- #2125 pulls a bathroom basin's suction stopper out from under a faucet spout and sets it in the soap dish. Nothing is
  emptied or put back there; here the basket full of scraps must be emptied into the caddy without losing any down the
  open drain, and then re-seated in its bore.

## One hand or two

The demo passes the basket from hand to hand, as people often do: one hand lifts it out by its knob, and the other
holds it by its cup to empty it. The task does not need two hands, and we do not claim that it does.

In the one-hand check (run it with `SINK_ONEHAND=1`), the left never moved. The right hand alone:
- lifted the basket out by its knob;
- carried it over the caddy along the same path;
- turned it over and shook it twice;
- brought it back and pressed it home.

That trial scored 1.0. The knob grip alone holds the 76 g basket and its scraps while it is turned over: the right's
pads held about 38 N each throughout.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. Everything here is built from shapes from
`bodykit.py`, at real size and mass:
- The sink, the strainer body and the tap are fixed bodies.
- The basket, the nine scraps and the caddy are loose rigid bodies.

- **The counter.** The station table is the counter. The task file replaces the room's table with four slabs that
  leave a cut-out under the sink's bowl. yam_arena itself is not edited.
- **The sink.**
  - The 20 cm deep bowl is drawn in 0.9 mm steel, with coved floor edges and corners. Its walls and floor collide as
    10 mm slabs outside the bowl.
  - The rim stands 3 mm proud of the counter.
  - The drain is in the middle of the floor.
- **The strainer body.**
  - A flange 114 mm across slopes 5.5 mm down into an 82 mm bore. The slope is a lead-in for the basket.
  - A crossbar 49 mm below the floor carries a hub: a socket 11 mm across and 5 mm deep that the basket's post stands
    in.
  - Below the bore it narrows to a 1-1/2 in tailpiece.
- **The basket.**
  - A tapered cup (79.4 mm across its rolled rim, 60 mm at the floor, 28 mm deep) with drawn perforations. It collides
    as a solid cup with 1.5 mm walls.
  - A post through its middle with a knurled knob, 16 mm across and 9 mm tall.
  - A rubber seal 52 mm across under its floor, and a 6 mm stem 10 mm long below that.
  - Its centre of mass is 8 mm above the cup's underside.
- **The scraps.** Each is a rigid shape at the size and mass of its printed stand-in (the table in "The scraps in a
  real trial"), which copies a real scrap's size and its mass estimated from volume:
  - the peels are 1.5 mm slabs;
  - the peas are spheres;
  - each rice clump is two spheres;
  - the broccoli is a head on a stalk;
  - the apple core is a block.

  They are not soft, wet or sticky, and neither are the stand-ins. Their friction against the basket's steel is 0.40
  (0.425 for the rice and the broccoli), so they slip on a slope steeper than 22 to 23 degrees.
- **The caddy.** ABS with tapered 2.5 mm walls. Each wall is backed outside by an unseen 8 mm slab, so a fast pea
  cannot pass through the thin plastic in one physics step.
- **The tap.** A single-lever gooseneck, a static obstacle on the back deck, with its spout swung over the bowl.
- **The task file's own quaternion conversion.** `bodykit.py`'s conversion from a rotation matrix to a quaternion
  picks wrong signs near 180 degrees: a wall facing -x came out tilted the wrong way and left a slot at the caddy's
  floor. The task file gives `bodykit.plate()` a Shepperd conversion instead.

After every control step the task file reads:
- the basket's, the caddy's and every scrap's pose;
- PhysX contact forces between the basket and each finger pad, and between the basket and the strainer body;
- each gripper's position.

From these it keeps the failure latches and the end state the score is built from.

## Real sizes and their sources

- **Sink.** A 25 x 22 in top-mount single bowl, 20 to 22 gauge, with a bowl 21 x 15-3/4 in and 8-1/16 in deep: the
  Elkay Dayton DSE12522x family. Retailer listings give "25 x 22 x 8-1/16 in", with a bowl 21 x 15-3/4 in. Modelled:
  bowl 533 x 400 mm and 200 mm deep, a 38 mm front rim and a 121 mm back deck.
- **Strainer body.** The Kohler K-8801 spec sheet gives a flange 114 mm across, a body 86 mm across and 22 + 57 mm
  deep, for a 3-1/2 in (89 mm) sink outlet. Modelled: an 82 mm bore.
- **Basket.**
  - A fixed-post replacement basket, 3.15 in (80 mm) across and 2.24 in (57 mm) tall (PlumbPak PP820-50).
  - Other listings agree: one replacement basket is 2-1/4 in deep (Elkay), and another is 2.3 in tall and weighs
    0.15 lb, or 68 g (Do it Best).
  - How that height splits into cup, knob, seal and stem is our choice, from the Regency 600DBS312 product photo.
- **Caddy.** A Brabantia SinkSide food-waste caddy, 1.8 l. The maker's listing gives 22.1 x 12.7 x 10.7 cm with the
  lid on, and retailers give 0.40 to 0.43 kg. It is ABS with a stainless steel handle. Here it stands with its lid
  off, 9.5 cm tall.
- **Scraps.** Nine pieces, 7.9 g in all. Their sizes are typical of what collects in a sink strainer. Their masses
  are volume times density, our estimate: 1.01 to 1.07 g/cm3 for the peels, peas and rice, 0.81 for the apple and
  0.61 for the broccoli floret, which holds air between its buds. Nothing here was weighed by us.
- **Stand-in material.** ABS filament at 1.05 g/cm3, for example Bambu Lab ABS (the MatterHackers listing of
  "Bambu Lab Orange ABS Filament - 1.75mm (1kg)",
  <https://www.matterhackers.com/store/l/bambu-lab-abs-filament-175mm-1kg/sk/M5WW9W3C>, gives "1.05 g/cm³").
- **Tap.** A generic gooseneck: 41 cm tall, a 21 cm reach, and its spout 25 cm above the deck.

## The scraps in a real trial

Real scraps spoil, vary from batch to batch and leave residue on the grippers, so a real trial uses nine stand-ins
instead. They are printed once, kept and reused for every trial. Each has the size and shape of the simulated scrap
and its mass, so the simulation models the stand-ins' sizes and masses exactly.

| Stand-in | Shape and size | Mass | Colour | Printed |
|---|---|---|---|---|
| Carrot peel 1 | strip, 34 x 11 x 1.5 mm | 0.58 g | orange | solid |
| Carrot peel 2 | strip, 30 x 10 x 1.5 mm | 0.46 g | orange | solid |
| Potato peel | piece, 26 x 18 x 1.5 mm | 0.75 g | light brown | solid |
| Pea (two) | ball, 8 mm | 0.27 g each | green | solid |
| Rice clump 1 | two 9 mm balls joined, centres 8.55 mm apart (17.6 mm long) | 0.80 g | white | solid |
| Rice clump 2 | two 8 mm balls joined, centres 7.6 mm apart (15.6 mm long) | 0.55 g | white | solid |
| Broccoli | a 15 mm ball (the head) on a stalk 6.4 mm across, 21 mm long overall | 1.2 g | dark green, stalk light green | closed walls, part-filled |
| Apple core | block, 22 x 14 x 12 mm, two seeds marked on top | 3.0 g | cream | closed walls, part-filled |

**How they are made.**
1. Model the nine shapes from the table and print them in ABS at 1.05 g/cm3 (see "Stand-in material" above), with
   0.1 to 0.15 mm layers. Print the balls on supports and sand the support marks off, so they roll.
2. Print the peels, the peas and the rice clumps solid (100 % infill). By volume x 1.05 g/cm3, solid ABS makes them
   0.59, 0.47, 0.74, 0.28, 0.80 and 0.56 g, within 5 % of the table.
3. The broccoli and the apple core are lighter than solid plastic, because real florets and apple flesh hold air
   (0.61 and 0.81 g/cm3 in our estimate). Solid ABS would make them 2.07 and 3.88 g. Print them with closed walls and
   partial infill, and raise or lower the infill until they weigh 1.2 and 3.0 g: about 58 % and 77 % of their solid
   mass. The air stays sealed inside; the weigh-in at each session would show any water that got in.
4. Mark the apple's two seeds with a brown paint marker.

**Checks before use, and at the start of every session.**
- **Mass.** Weigh each stand-in on a 0.01 g scale. Each must be within 10 % of its mass in the table. Mass hardly
  matters for how a rigid piece slides or falls (gravity and friction both scale with it), but the check keeps the set
  the same from session to session.
- **Slip.** Each must slide or roll down a dry stainless steel tray tilted to 30 degrees. The simulated scraps slip on
  the basket's steel at 22 to 23 degrees (friction 0.40 to 0.425). If one sticks, sand its flat faces with 400-grit
  paper and check again.
- **Count.** Nine pieces, kept together in a labelled box between sessions. Replace any that cracks.

**Handling.** The stand-ins are used dry and at room temperature, and nothing about them depends on temperature. Real
scraps would be wet, and wet peel and starchy rice cling to steel. Neither the stand-ins nor the simulation reproduce
that cling, so a trial with real scraps would be somewhat harder. Handle the stand-ins with clean, dry hands.

**Reset between trials.**
1. Count the nine stand-ins, including any in the strainer body or the catch cup under its tailpiece.
2. If any is dusty or wet, wipe it with a damp cloth and dry it.
3. Seat the basket in the strainer body with its post in the hub.
4. Lay the stand-ins in the basket as in the reference photo (the first frame of the demo video). Directions are
   as seen from the robot: "near" is toward it, "far" away from it, and right is toward the right arm. "Lengthwise"
   means pointing away from the robot.
   - On the floor: carrot peel 1 lengthwise right of the post, carrot peel 2 lengthwise left of it, and one pea on
     the near side of the post and one on the far side.
   - On carrot peel 1: the potato peel, lengthwise, and on that, toward the far side, rice clump 2.
   - On carrot peel 2: rice clump 1, lengthwise.
   - On top: the broccoli at the near right and the apple core at the far left.
5. Empty the caddy and set it back on its marks on the counter. Send both arms to their start pose.

## Real-trial safety and rig protection

- **Food residue (the review's note for the team).** A real trial uses no food. The nine scraps are the clean, dry
  printed stand-ins above, so the finger pads never touch food, juice, starch or grease.
- **Clean parts.** The sink, the strainer body and basket, and the caddy are kept for these trials. Before the first
  trial, wash them in warm water with dish soap and dry them, because new stainless steel parts can carry machining
  oil. The tap is not connected to water, so the bowl stays dry.
- **Cleaning the pads.** At the end of every session, and whenever a pad looks dirty:
  1. wipe each finger pad with a lint-free cloth wrung out in warm water with a drop of dish soap;
  2. wipe it again with a cloth wrung out in clean water;
  3. dry it with a third cloth.

  If a stand-in, the basket or the sink ever gets food or grease on it, stop and wash and dry it before the next
  trial.
- **No pad covers.** Covers would change how the pads grip the 16 mm knob and the cup, and with no food in the trial
  there is nothing for them to keep off.
- **Catch cup.** Under the counter, the strainer body's tailpiece ends over a clear plastic cup instead of a drain
  pipe. Any stand-in that goes down the drain is caught there, counted and put back in the set.
- **Hazards.** Nothing in the trial is sharp, hot, heavy or breakable. The basket and the sink have rolled rims, and
  the stand-ins are blunt plastic weighing at most 3 g.
- **Force.** The job needs little force: the full basket weighs 76 g (0.75 N). The simulated grips close harder than
  that needs: about 37 N per pad on the knob, and about 49 N per pad on the cup, the simulated gripper's own limit.
  A real YAM's roughly 40 N per finger is still far more than the load needs.

## Assets and licences

No third-party assets. Every shape is built in the task file from boxes, cylinders and spheres, with `bodykit.py`
(written for this task family). The named products above are sources for real sizes only; no model, mesh, image
or scan of them is used.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: the full score, with the basket, the
caddy and every scrap still. Otherwise it gives `score()` at the end of the episode: 0, 0.25, 0.5 or 1.0.

**Judged at the end.**
- **In the caddy.** A scrap whose middle is inside the caddy's live inner volume: between its tapered walls, above its
  floor and below its rim, in the caddy's own frame, so it counts even if the caddy has been nudged. Real trial: count
  the scraps in the caddy.
- **Seated.** The basket is back in the strainer body:
  - its axis within 4 mm of the drain's;
  - at its seated height within 4 mm (its post down in the hub);
  - tilted no more than 8 degrees;
  - touching the strainer body.

  Real trial: the top of its knob is 8 mm above the sink floor, within 4 mm, by ruler; and by eye and by touch,
  the basket sits level and does not rock.
- **Hands off.** Neither gripper touches the basket, and each is at least 5 cm from it. Real trial: from the end of
  the video and a photo with a ruler.

**Tiers.**
- 1.0: all nine scraps in the caddy, the basket seated and both hands off it.
- 0.5: all nine scraps in the caddy.
- 0.25: at least five scraps in the caddy.
- Otherwise 0.0.

**Failures: 0.0 for good, at any time.**
- **Down the drain.** A scrap's middle gets inside the strainer body's bore or its tailpiece, more than 6 mm below the
  sink floor, while the basket is not seated, and it is not inside the basket's cup.
  - Inside the cup means, in the basket's own frame: above the cup's underside, no more than 10 mm above its rim, and
    inside its tapered wall (57 mm across at its foot, 73.4 mm just under the rim, plus 1 mm all round).
  - A scrap inside the cup is being carried in the basket. It never counts, wherever the basket is, so the scraps
    riding in the basket while it is lifted out of the drain or put back in do not trip this rule.
  - A scrap beside the basket, between its wall and the bore, is not in the cup and does count.
  - Real trial: watch the video for a scrap dropping into the open drain. At the end, lift the basket out and look
    into the strainer body and into the catch cup under the tailpiece. Any scrap there, outside the basket, counts.
  - Checks of this rule are below, under "Checks of the drain rule".
- **On the counter.** A scrap's middle within 15 mm of the counter, outside the bowl and the caddy. This includes the
  sink's rim and the counter right beside the caddy. Real trial: look over the counter.
- **On the floor.** A scrap or the basket on the floor.
- **Dropped.** The basket falling faster than 0.6 m/s with no finger pad on it. Real trial: judged from the video.

### Checks of the drain rule

- **The basket carried with its scraps does not trip it.** In the scripted demo the right lifts the full basket out
  of the strainer body. All nine scraps were in the bore inside the basket at once (`carried_in_bore` 9), the drain
  rule never fired, and the demo scored 1.0.
- **Put back with the scraps still in it.** With `SINK_TEST=reseat` the right lifts the full basket out of the
  strainer body and up out of the bowl, lowers it back in, presses it home, holds it seated and lifts it out again.
  Then the normal demo follows. In that run all nine scraps were in the bore inside the basket while it was lifted
  out, again while it was lowered back in, and again while it was lifted out the second time. The basket sat
  seated for about 2 s in between. The drain rule never fired, and the trial scored 1.0.
- **Scraps that are not in the basket do trip it.** Running the task file with Python, no simulator needed
  (`python empty_the_sink_strainer_into_the_bin.py`), checks the rule's geometry on poses set by hand:
  - the full basket lifted 0 to 60 mm out of the strainer body, upright or tilted up to 15 degrees: up to all nine
    scraps are in the bore inside it, and none counts;
  - a pea beside a basket lifted 10 mm, between its wall and the bore (36 mm from its axis, 5 mm above its underside;
    or 37 mm and 2 mm): counts;
  - a pea on the crossbar under a basket lifted 40 mm: counts;
  - a pea 30 mm down the bore while the basket is over the caddy: counts.

  Before this revision, "in the basket" was a cylinder as wide as the rim (79.4 mm), so both of those peas beside
  the basket would have passed as carried in it and not counted.

`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same layout.
The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending it,
so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=empty_the_sink_strainer_into_the_bin.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Empty the food scraps from the sink's drain strainer into the food-waste caddy and put the strainer back in the drain, without letting any scraps go down the drain or onto the counter." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/strainer-dream
```

The scripted demo reaches full score. It runs the steps listed at the top, with these details:
- The right grips the knob and lifts the basket out of the strainer body, up out of the bowl and toward the caddy.
- The left comes in from the front with its fingers wide open, slides them round the cup and closes on its rim.
- The right opens, rises 3 cm off the knob and waits over the sink, clear of the left's path.
- The left lifts the basket and carries it over the caddy, turning it as it goes until it is turned 104 degrees about
  the left hand's own axis, its mouth facing the sink end of the caddy and down.
- The left holds it there and shakes it twice, 2.5 cm toward the mouth and back, then turns it back up and brings it
  back to where it took it.
- The right comes back down round the knob and grips it. The left lets go and leaves.
- The right carries the basket over the drain, lowers it into the strainer body and presses it 12 mm past its seated
  height, then lets go.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=empty_the_sink_strainer_into_the_bin.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Empty the food scraps from the sink's drain strainer into the food-waste caddy and put the strainer back in the drain, without letting any scraps go down the drain or onto the counter." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/strainer-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/strainer-dream/*.json; do inspect-robots video "$log"; done
```

Set `SINK_DEBUG=1` to print what the score is built from, four times a second:
- the basket's offset from its seat, its tilt, whether it is seated, and its contact with the strainer body;
- each finger pad's force on the basket and where on the basket it presses;
- where the caddy is and how far it is tilted;
- how many scraps are in the caddy, and the failure latches;
- `carried_in_bore`: how many scraps are in the bore inside the basket's cup while the basket is not seated (the
  drain rule passes over these), now and the most at once so far.

Once a second it also prints every scrap's position in the caddy's frame.

## Runtime per trial

On one RTX 5090: about 12 minutes for a Dream-1 trial of 1,600 steps (719 s on the clock, with Dream answering in 0.73 s on average), and about 12 minutes for the scripted demo (1,256 steps, 62.8 s of simulated time, 710 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.
These times were measured while the GPU was shared with other simulations; with the GPU to itself, the
previous version's runs took about 5 minutes each.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-3UTY4M. Its review said:

> **Not accepted** (TK-3UTY4M).
>
> Flags: Note for the team: Consider washable pad covers or a cleaning protocol for contact with food residue.
>
> Scoring (not met): The stated drain-failure rule counts scraps still in the basket during removal, although the README exempts them.
>
> Clarity and reproducibility (not met): The perishable scraps and cooked rice clumps lack a repeatable preparation, temperature and handling procedure for successive trials.
>
> What to change: Please specify how to prepare or source consistent scraps for each trial, including the cooked rice clumps, temperature and handling, or use repeatable stand-ins that preserve the relevant handling properties. Also amend the drain-failure rule to exclude scraps still carried inside the basket during removal and reseating, and ensure the scorer and documentation use the same rule.

This is the task's first revision; there was no earlier review.

What changed in response:
- **Scoring (not met): the stated drain rule now exempts scraps carried in the basket, as the scorer and README do.**
  - task.json's scoring, the scorer's code and docstrings, and this README now state one rule. A scrap has gone down
    the drain if its middle gets inside the strainer body's bore or tailpiece, more than 6 mm below the sink floor,
    while the basket is not seated, and it is not inside the basket's cup.
  - A scrap inside the cup is being carried in the basket and never counts, including while the basket is lifted out
    of the drain or put back in.
  - "Inside the cup" is now the cup's real inside, in the basket's own frame: above its underside, inside its tapered
    wall (57 to 73.4 mm across, plus 1 mm all round) and no more than 10 mm above its rim. It used to be a cylinder
    as wide as the rim, so a scrap that slipped down beside the basket into the bore passed as carried. It now
    counts.
  - The real-trial check matches: watch the video for a scrap dropping into the open drain, then lift the basket out
    and look into the strainer body and the catch cup under the tailpiece. Any scrap there, outside the basket,
    counts.
- **What to change: a check that a basket carried with scraps inside does not trip the rule.**
  - New check `SINK_TEST=reseat`: the right lifts the full basket out of the drain, lowers it back in, presses it
    home, holds it seated and lifts it out again, and then the normal demo follows. All nine scraps rode in the
    basket inside the bore during each of the three passes, the drain rule never fired, and the trial scored 1.0.
  - In the normal demo, too, all nine were in the bore inside the basket as it came out, and it scored 1.0.
  - A geometric self-test in the task file (`python empty_the_sink_strainer_into_the_bin.py`, no simulator) gives 0
    scraps counted for the full basket anywhere from seated to 60 mm up, upright or tilted 15 degrees. It counts a pea
    beside the basket in the bore, a pea under the lifted basket and a pea in the bore while the basket is away.
- **Clarity and reproducibility (not met), and what to change: repeatable scraps.**
  - A real trial now uses nine clean, dry stand-ins, 3D-printed in ABS (1.05 g/cm3), made once and reused. Each has
    its scrap's size and shape and its mass (7.9 g in all), and the simulated scraps have exactly those sizes and
    masses. The new section "The scraps in a real trial" gives the table of sizes, masses and colours.
  - The cooked rice clumps are two printed balls joined: 9 mm balls at 0.80 g, and 8 mm balls at 0.55 g.
  - Seven pieces are printed solid. Solid ABS gives them masses within 5 % of the table. The broccoli and the apple
    core hold air in real life (0.61 and 0.81 g/cm3), so they are printed with closed walls and their infill is set
    until they weigh 1.2 and 3.0 g.
  - At the start of each session, each piece is weighed on a 0.01 g scale (within 10 %). Each must also slide or
    roll down a dry stainless tray tilted to 30 degrees; the simulated scraps slip at 22 to 23 degrees.
  - Temperature and handling: used dry and at room temperature, with clean, dry hands. Nothing about them depends on
    temperature.
  - The reset between trials is spelled out: count, wipe, lay them out as in the reference photo, seat the basket,
    empty and re-place the caddy. task.json's setup, reset notes and fidelity say the same.
  - Disclosed: neither the stand-ins nor the simulation have the cling of wet peel and starchy rice, so real scraps
    would make the job somewhat harder.
- **Flag (note for the team): pad covers or a cleaning protocol.** A new section, "Real-trial safety and rig
  protection":
  - no food is used, so the pads never touch food residue;
  - the sink, basket and caddy are washed with dish soap once before use and kept for the trials, and the tap is
    not connected;
  - a three-cloth pad wipe (soapy, clean, dry) ends every session;
  - a catch cup under the tailpiece keeps any stand-in out of plumbing.
- **Also.** The seated and hands-off checks now name their real-trial tools (a ruler for the knob's height, the
  video and a photo for the hands). A short "Assets and licences" section states that no third-party assets are
  used. "Real-trial safety and rig protection" also notes the forces involved.
- **Re-run.** The scorer's cup test changed, so everything was run again. The scripted demo scores 1.0 (1,256
  steps of the 1,600-step limit). The re-seat check scores 1.0. The one-hand check still scores 1.0, so we still
  don't claim two hands. Dream-1 scores 0.0: it hovers over the drain and never lifts the basket.

# Empty the sink strainer into the bin

A stainless single-bowl kitchen sink is set into the counter on the right. It is a 25 x 22 in top-mount sink with a
bowl 21 x 15-3/4 in and 20 cm deep, and a gooseneck tap stands on its back deck. In its 3-1/2 in drain sits a
fixed-post strainer basket, 79.4 mm across and 68 g, holding nine food scraps: two strips of carrot peel, a piece of
potato peel, two clumps of cooked rice, two peas, a piece of broccoli and a piece of apple core. Like real wet scraps,
they cling to the basket: turned over, they stay in it, and it takes a firm shake to free them. In a real trial the
scraps are stand-ins 3D-printed in ABS to the scraps' sizes and masses, dipped before each trial in a cooked cornstarch
paste that makes them cling (see "The scraps in a real trial"). A 1.8 l food-waste caddy with its lid off stands on
the counter just left of the sink.

The robot is told: "Empty the food scraps from the sink's drain strainer into the food-waste caddy and put the strainer
back in the drain, without letting any scraps go down the drain or onto the counter."

In the scripted demo:
1. The right hand reaches into the bowl, grips the basket's knob and lifts the basket out of the strainer body, up out
   of the bowl and toward the caddy.
2. The left comes in from the front with its fingers wide open and closes on the cup across its rim. The right lets
   go, rises 3 cm off the knob and waits over the sink.
3. The left carries the basket over the caddy, turning it as it goes to 104 degrees about its own axis (a wrist roll),
   its mouth facing the sink end of the caddy and down. The scraps stay stuck. The left shakes it twice, each time a
   5 cm jab toward the mouth (level) in 0.1 s and straight back in 0.1 s, then a 0.5 s hold. The scraps come loose
   and fall into the caddy.
4. The left turns it back up and brings it back to where it took it.
5. The right takes the knob again; the left lets go and leaves.
6. The right carries the basket over the drain, lowers it to 4.5 cm above its seat, holds it there 0.6 s so it stops
   swinging, lowers it in, presses it 12 mm past its seated height and lets go.
7. Both arms go back to where they started.

The task is one task file, [`empty_the_sink_strainer_into_the_bin.py`](empty_the_sink_strainer_into_the_bin.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests:
- Reaching 20 cm down into a real-depth sink to take a small, loose part by its 16 mm knob.
- Carrying the basket level, and emptying it over a small bin. No door or latch does the emptying.
- Shaking stuck scraps loose. They cling to the basket, so turning it over leaves them in it; a firm, short shake frees
  them, and they must still land in the caddy, not on the counter.
- Keeping scraps out of the open drain and off the counter while the basket is out.
- Re-seating the basket in an 82 mm bore with about 1 mm to spare, its post in the crossbar's hub.

## How this differs from the board

The nearest tasks on the board are #1262 and #1235 (empty a vacuum's dust container into a bin and put it back) and
#735 (lift a pool skimmer basket out of its well by its handle).

- **#1262 and #1235.** Both empty a dust container into a bin and put it back. There, a held button or a flap opens
  doors that do the emptying, dry dust falls out, and the container clicks into a seat it sits on.
- **#735.** That skimmer basket is lifted by a large handle out of an open well; it is not emptied or re-seated.
- **This task.**
  - A loose basket is fished out of a 3-1/2 in drain 20 cm down a full-depth kitchen sink, by a 16 mm knob.
  - It is emptied by turning it over the caddy and shaking loose the scraps that cling to it.
  - It must be re-seated in a bore with about 1 mm to spare, its post in a hub.
  - The drain stands open while the basket is out, and any scrap that goes down it ends the trial at 0.

Other sink tasks on the board do other jobs: #1857 and #1859 reconnect a pop-up stopper, #630 and #860 put in or take
out a mug or a hair dryer, #1251 and #2092 plunge a sink, #2058 spans a colander across a basin, and #2125 pulls a
basin's suction stopper and sets it in the soap dish. None empties a basket of clinging scraps or re-seats it.

## One hand or two

The demo passes the basket from hand to hand, as people often do: one hand lifts it out by its knob, and the other
holds it by its cup to empty it. The task does not need two hands.

In the one-hand check (`SINK_ONEHAND=1`) the left never moved: the right alone lifted the basket out by its knob,
carried it over the caddy, turned it over, shook it twice with the same jabs, brought it back and pressed it home. It
scored 1.0 in 1,031 steps; the knob grip held the 76 g basket throughout, its pads pressing at most 39 N each.

## How it is modelled

Everything is built at real size and mass from boxes, cylinders and spheres with `bodykit.py`. The sink, the
strainer body and the tap are fixed bodies; the basket, the nine scraps and the caddy are loose rigid bodies.
- **The counter.** The station table, replaced by four slabs that leave a cut-out under the sink's bowl (yam_arena
  itself is not edited).
- **The sink.** A 20 cm deep bowl drawn in 0.9 mm steel with coved floor edges and corners; its walls and floor collide
  as 10 mm slabs outside the bowl. The rim stands 3 mm proud of the counter; the drain is in the middle of the floor.
- **The strainer body.** A flange 114 mm across slopes 5.5 mm down into an 82 mm bore (a lead-in). A crossbar 49 mm
  below the floor carries a hub, a socket 11 mm across and 5 mm deep for the basket's post. Below, a 1-1/2 in
  tailpiece.
- **The basket.** A tapered cup (79.4 mm across its rolled rim, 60 mm at the floor, 28 mm deep) with drawn
  perforations, colliding as a solid cup with 1.5 mm walls; a post with a knurled knob 16 mm across and 9 mm tall; a
  rubber seal 52 mm across under its floor and a 6 mm stem 10 mm long. Its centre of mass is 8 mm above the cup's
  underside.
- **The scraps.** Each is a rigid shape at the size and mass of its printed stand-in (the table in "The scraps in a
  real trial"): the peels 1.5 mm slabs, the peas spheres, each rice clump two spheres, the broccoli a head on a
  stalk, the apple core a block. Their friction against the basket's steel is 0.40 (0.425 for the rice and the
  broccoli).
- **The cling.** Each scrap is held to the basket where it lies by a bond, a PhysX fixed joint. After every physics
  step (1/240 s) the task file works out the force the basket must put on each bonded scrap, its mass times (a - g),
  from the scrap's acceleration averaged over 3 physics steps (12.5 ms), in the basket's own frame:
  - a push into the cup's floor is carried by contact, and sideways shear up to 0.40 times that push by friction;
  - the rest, a pull away from the floor and any shear beyond friction, is the cling's load.

  Once that load tops 2.0 times the scrap's weight, the bond is switched off for good and the scrap lies loose in the
  cup (the two still collide). Held still, upright, the load is 0; on its side or upside down it is 1.0 times the
  weight, so tipping alone leaves every scrap stuck. The loads met in each part of the job are measured under "Checks
  of the cling". The paste itself is not modelled.
- **The caddy.** ABS with tapered 2.5 mm walls. Each wall is backed outside by an unseen 8 mm slab, so a fast pea
  cannot pass through the thin plastic in one physics step.
- **The tap.** A single-lever gooseneck, a static obstacle on the back deck, with its spout swung over the bowl.

After every control step the task file reads the basket's, the caddy's and every scrap's pose, PhysX contact forces
between the basket and each finger pad and the strainer body, and each gripper's position; from these it keeps the
failure latches and the end state the score is built from. After every physics step it reads each bonded scrap's
velocity, for the cling.

## Real sizes and their sources

- **Sink.** The Elkay Dayton DSE12522x family: retailer listings give "25 x 22 x 8-1/16 in", 20 to 22 gauge, with a
  bowl 21 x 15-3/4 in. Modelled: bowl 533 x 400 mm and 200 mm deep, a 38 mm front rim and a 121 mm back deck.
- **Strainer body.** The Kohler K-8801 spec sheet gives a flange 114 mm across, a body 86 mm across and 22 + 57 mm
  deep, for a 3-1/2 in (89 mm) sink outlet. Modelled: an 82 mm bore.
- **Basket.** A fixed-post replacement basket, 3.15 in (80 mm) across and 2.24 in (57 mm) tall (PlumbPak
  PP820-50); others are 2-1/4 in deep (Elkay) and 2.3 in tall, 0.15 lb or 68 g (Do it Best). The split of that height
  into cup, knob, seal and stem is ours, from the Regency 600DBS312 product photo.
- **Caddy.** A Brabantia SinkSide food-waste caddy, 1.8 l, ABS with a steel handle: 22.1 x 12.7 x 10.7 cm with the
  lid on (the maker's listing), 0.40 to 0.43 kg (retailers). Here its lid is off and it is 9.5 cm tall.
- **Scraps.** Nine pieces, 7.9 g in all. Their sizes are typical of what collects in a sink strainer. Their masses
  are computed as volume times density: 1.01 to 1.07 g/cm3 for the peels, peas and rice, 0.81 for the apple and
  0.61 for the broccoli floret, which holds air between its buds.
- **Stand-in material.** ABS filament at 1.05 g/cm3, for example Bambu Lab ABS (the MatterHackers listing of
  "Bambu Lab Orange ABS Filament - 1.75mm (1kg)",
  <https://www.matterhackers.com/store/l/bambu-lab-abs-filament-175mm-1kg/sk/M5WW9W3C>, gives "1.05 g/cm³").
- **Cling coat.** Argo 100% Pure Corn Starch, 16 oz (the Target listing,
  <https://www.target.com/p/-/A-14771426>: "Argo 100% Pure Corn Starch - 16oz", ingredients "corn starch", serving
  size 8.0 g, $2.79).
- **Pad covers.** AMMEX Gloveworks HD nitrile gloves, part GWBN46100 (the Global Industrial listing,
  <https://www.globalindustrial.com/p/gloveworks-hd-diamond-textured-nitrile-industrial-gloves-l-black-100-per-box-10-box-per-cs>:
  6.0 mil, black, diamond texture, size L, 100 per box).
- **Tap.** A generic gooseneck: 41 cm tall, a 21 cm reach, and its spout 25 cm above the deck.

## The scraps in a real trial

Real scraps spoil, vary from batch to batch and fall apart, so a real trial uses nine stand-ins. They are printed once
in ABS, kept and reused, and each has the size, shape and mass of its simulated scrap. Before each trial they are
dipped in a cooked cornstarch paste, which makes them cling to the basket's steel and to each other as wet peel and
starchy rice do: starch is what makes cooked rice stick. A per-session cling check holds the paste to the behaviour
the simulation's cling models.

| Stand-in | Shape and size | Mass (dry) | Colour | Printed |
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
   0.1 to 0.15 mm layers. Print the balls on supports and sand the support marks off.
2. Print the peels, the peas and the rice clumps solid (100 % infill). By volume x 1.05 g/cm3, solid ABS makes them
   0.59, 0.47, 0.74, 0.28, 0.80 and 0.56 g, within 5 % of the table.
3. The broccoli and the apple core hold air in real life (0.61 and 0.81 g/cm3, design values); solid ABS would make
   them 2.07 and 3.88 g. Print them with closed walls and set the infill until they weigh 1.2 and 3.0 g.
4. Mark the apple's two seeds with a brown paint marker.

**The cling paste, made fresh each session.**
1. Weigh 6.0 g of Argo 100% Pure Corn Starch and stir it into 100 g of cold tap water in a microwave-safe bowl until
   smooth.
2. Microwave on high in 20 s bursts, stirring after each, until it boils and turns from milky to translucent and
   thick, about a minute in all.
3. Lay plastic wrap on its surface and let it cool to room temperature, 20 to 25 degrees C by a kitchen thermometer
   (about 30 minutes).
4. Keep it covered at room temperature. It is food and goes off: throw it away at the end of the session, or 4 hours
   after it was made, whichever comes first.
5. For each trial, take each stand-in with tweezers, dip it all over in the paste, hold it over the bowl for 10 s to
   drip, and lay it in the basket at its place.

**The cling check, at the start of each session and after every 10 trials.** With the nine pieces coated and laid in
the clean, dry basket as for a trial, held by its knob over a tray:
1. Turn the basket on its side (90 degrees) over about 3 s and hold it 10 s: no piece moves.
2. Turn it on over, upside down, and hold it 5 s: no piece falls.
3. Give it one firm shake, a quick jab of about 5 cm toward its mouth and straight back, in about 0.2 s: every piece
   falls.

If a piece moves in step 1 or falls in step 2, dip it again, let it drip only 5 s and repeat; if it fails again, make
a new batch with 7 g of starch. If a piece is still in the basket after step 3, stir 10 g of water into the paste, dip
the pieces again and repeat. In the simulation the cling lets go at 2.0 times a scrap's weight: held still on its side
or upside down, its load is 1.0 times the weight, and the demo's jab took every scrap past 2.0 (see "Checks of the
cling").

**Every session.** Weigh each dry stand-in on a 0.01 g scale before coating: each is within 10 % of the table. Count
them: nine, kept together in a labelled box between sessions. Replace any that cracks.

**Handling.** The paste and the stand-ins are used at room temperature, 20 to 25 degrees C. Handle the coated pieces
with tweezers. At the end of each session, wash the stand-ins in warm water with a drop of dish soap, rinse them, dry
them on a towel and put them back in their box.

**Reset between trials.**
1. Count the nine stand-ins, including any in the strainer body or the catch cup under its tailpiece.
2. Rinse them in a bowl of warm water to take the old coat off, and pat them dry on a paper towel.
3. Rinse the basket under a tap away from the rig and dry it inside and out with a lint-free cloth. Wipe the caddy's
   inside and the sink's floor round the drain with a damp cloth, and dry them.
4. Seat the basket in the strainer body with its post in the hub.
5. Coat the stand-ins (dip, 10 s drip) and lay them in the basket as in the reference photo (the first frame of the
   demo video). Directions are as seen from the robot: "near" is toward it, "far" away from it, and right is toward the
   right arm. "Lengthwise" means pointing away from the robot.
   - On the floor: carrot peel 1 lengthwise right of the post, carrot peel 2 lengthwise left of it, and one pea on
     the near side of the post and one on the far side.
   - On carrot peel 1: the potato peel, lengthwise, and on that, toward the far side, rice clump 2.
   - On carrot peel 2: rice clump 1, lengthwise.
   - On top: the broccoli at the near right and the apple core at the far left.
6. Set the caddy back on its marks on the counter. Send both arms to their start pose. Start the trial within
   2 minutes of coating.

## Real-trial safety and rig protection

- **Pad covers.** Before a session, each of the four finger pads gets a disposable cover: a fingertip cut from an
  AMMEX GWBN46100 glove (6 mil black nitrile, diamond texture), pulled over the pad and taped at the finger's root with
  vinyl electrical tape. Check that a covered gripper still lifts the full basket by its knob without slipping. Change
  the covers whenever paste shows on them, and at the end of the session.
- **What the fingers touch.** Only the basket's knob and the outside of its cup. The paste is on the stand-ins inside
  the cup, and the basket is dried inside and out before each trial.
- **The paste.** Cornstarch and water: no oil, grease or acid, and it rinses off with water. Wipe any drop on the sink,
  the counter or the caddy with a damp cloth at the reset.
- **Clean parts.** The sink, the strainer body and basket, and the caddy are kept for these trials. Before the first
  trial, wash them in warm water with dish soap and dry them. The tap is not connected to water.
- **Cleaning the pads.** At the end of every session, after the covers come off, wipe each pad with a lint-free cloth
  wrung out in warm water with a drop of dish soap, then with one wrung out in clean water, and dry it with a third.
- **Catch cup.** Under the counter, the strainer body's tailpiece ends over a clear plastic cup instead of a drain
  pipe. Any stand-in that goes down the drain is caught there, counted and put back in the set.
- **Hazards.** Nothing in the trial is sharp, hot, heavy or breakable. The basket and the sink have rolled rims, and
  the stand-ins are blunt plastic weighing at most 3 g.
- **Force.** The job needs little force: the full basket weighs 76 g (0.75 N). The simulated grips close harder than
  that needs: at most 39 N per pad on the knob, and at most 51 N per pad on the cup, the simulated gripper's own limit.
  A real YAM's roughly 40 N per finger is still far more than the load needs.

## Assets and licences

No third-party assets. Every shape is built in the task file from boxes, cylinders and spheres, with `bodykit.py`
(written for this task family). The named products above are sources for real sizes, or supplies to buy; no model,
mesh, image or scan of them is used.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: the full score, with the basket, the
caddy and every scrap still. Otherwise it gives `score()` at the end of the episode: 0, 0.25, 0.5 or 1.0.

**Judged at the end.**
- **In the caddy.** A scrap whose middle is inside the caddy's live inner volume: between its tapered walls, above its
  floor and below its rim, in the caddy's own frame, so it counts even if the caddy has been nudged, and not inside
  the basket's cup. A scrap still in the basket never counts, even with the basket set down in the caddy. Real trial:
  count the scraps in the caddy, out of the basket.
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
  - A scrap inside the cup is being carried in the basket and never counts, wherever the basket is (lifted out of
    the drain or put back in). A scrap beside the basket, between its wall and the bore, does count.
  - Real trial: watch the video for a scrap dropping into the open drain. At the end, lift the basket out and look
    into the strainer body and into the catch cup under the tailpiece. Any scrap there, outside the basket, counts.
- **On the counter.** A scrap's middle within 15 mm of the counter, outside the bowl and the caddy. This includes the
  sink's rim and the counter right beside the caddy. Real trial: look over the counter.
- **On the floor.** A scrap or the basket on the floor.
- **Dropped.** The basket falling faster than 0.6 m/s with no finger pad on it. Real trial: judged from the video.

### Checks of the drain rule

- **The basket carried with its scraps does not trip it.** In this revision's demo all nine scraps were in the bore
  inside the basket at once as it was lifted out (`carried_in_bore` 9); the rule never fired, and the demo scored 1.0.
- **Put back with the scraps still in it** (`SINK_TEST=reseat`, run for the previous revision): the full basket put
  back, held seated 2 s and lifted out again before the normal demo; the rule never fired, and it scored 1.0.
- **Scraps that are not in the basket do trip it.** Run with Python, the task file checks the rule on poses set by
  hand: the full basket 0 to 60 mm up, upright or tilted up to 15 degrees, counts none; a pea beside a basket lifted
  10 mm (36 mm from its axis, 5 mm up; or 37 mm, 2 mm up), a pea on the crossbar under a basket lifted 40 mm, and a pea
  30 mm down the bore with the basket over the caddy each count.
- **The caddy count leaves out scraps in the basket.** The same self-test stands the full basket in the caddy: all
  nine scraps are inside the caddy's volume, and none is counted in the caddy; a pea on the caddy's floor beside the
  basket is counted.

### Checks of the cling

**Calibration** (`SINK_TEST=shakes`: the demo with every bond kept on, and a series of shakes over the caddy). The
largest cling load on any of the nine scraps in each part of the job, in multiples of its weight:

| Part of the job | Largest load |
|---|---|
| Lifting the basket out, handing it over and carrying it, upright | 0.47 |
| Turning it over (104 degrees) and holding it there | 1.04 |
| The previous revision's shake: twice 2.5 cm toward the mouth and back, 0.22 s each way | 1.50 |
| Raised 6 cm, brought down in 0.1 s and stopped short | 1.74 |
| A 3, 4 or 5 cm thrust toward the mouth in 0.1 s, back slowly | 1.67, 1.93, 2.22 |
| A 4 cm thrust in 0.15 s, back slowly | 1.72 |
| A 4 cm jab toward the mouth in 0.1 s and straight back in 0.1 s | 2.98 (each scrap 2.44 or more) |
| Turning it back up, handing it back to the right and re-seating it | 1.21 |

The threshold, 2.0 times the weight, is above careful handling (0.47 to 1.37, the hand-backs included), turning the
basket over (1.04) and the previous revision's gentle shake (1.50). Of the shakes tried, only the jab, which turns the basket back hard at the
end of its stroke, frees every scrap; a single 5 cm thrust frees one of the nine. The demo's shake and the cling
check's firm shake are that jab, at 5 cm.

Run with Python (no simulator), the task file prints the rule's load for set cases: held still at 0, 45, 90, 104,
135 and 180 degrees, 0, 0.42, 1.00, 1.00, 1.00 and 1.00; upright and lifted at 2 g, 0; upside down and jerked up at
1 g, 2.00.

**The demo.** At the first jab all nine scraps came loose, at loads of 2.04 to 2.93 times their weight, and fell into
the caddy; the demo scored 1.0 in 1,291 steps.

**Without a firm shake** (adversarial runs; the first two are the demo with only the shake changed):
- `SINK_TEST=noshake`: turned over over the caddy and held as long as the demo's shakes take, with no shake. The
  largest cling load was 1.04 turned over, and 1.37 over the whole run (as the left let go of the cup after handing
  it back); all nine scraps stayed stuck, rode back in the basket and were re-seated with it. None reached the caddy:
  0.0.
- `SINK_TEST=gentle`: the previous revision's shake, two 2.5 cm strokes toward the mouth and back, 0.22 s each way.
  The largest load was 1.50, at the strokes; all nine stayed stuck and were re-seated with the basket: 0.0.
- `SINK_TEST=binbasket`: the right lifts the basket out, carries it over the caddy, sets it down in the caddy with
  every scrap still stuck in it, lets go and goes home. All nine scraps ended inside the caddy's volume, none counted
  (all in the basket's cup), the hands were off and nothing was dropped: 0.0 (the caddy count before this revision
  would have given 0.5).

`-E object_jitter=0` keeps the layout fixed. The cameras render at 1280 x 704 for sharp videos; Dream-1's client
scales every frame to 320 x 176 before sending it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`); for the scripted demo, use `--policy yam_scripted`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=empty_the_sink_strainer_into_the_bin.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Empty the food scraps from the sink's drain strainer into the food-waste caddy and put the strainer back in the drain, without letting any scraps go down the drain or onto the counter." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/strainer-dream
```

The scripted demo reaches full score with the steps listed at the top.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/strainer-dream/*.json; do inspect-robots video "$log"; done
```

`SINK_DEBUG=1` prints, four times a second, what the score is built from (the basket's seat, pad forces, the caddy
count, `carried_in_bore`, the latches) and the cling (bonded scraps, loads, and when each came loose).

## Runtime per trial

On one RTX 5090: about 22 minutes for a Dream-1 trial of 1,600 steps (1328 s on the clock, with Dream answering in 0.84 s on average), and about 15 minutes for the scripted demo (1,291 steps, 64.5 s of simulated time, 887 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.
These times were measured while the GPU was shared with other simulations.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-2H8NJG. Its review said:

> **Not accepted** (TK-2H8NJG).
>
> Construct validity (not met): Dry, free-sliding ABS scraps omit the adhesion of wet food to the basket, making the emptying action easier than the household task.
>
> What to change: The drain-rule clarification, repeatable reset instructions and cleaning protocol address the earlier feedback. One point remains open: the requested stand-ins must preserve the relevant handling properties, but the dry ABS pieces slide freely and do not reproduce wet scraps clinging to the strainer. Please specify a repeatable set of stand-ins and preparation that captures this behavior, represent the corresponding handling difficulty in simulation, and provide an updated full-score demo.

The review before that (TK-3UTY4M) asked for a drain rule that exempts scraps carried in the basket, repeatable
scraps with their preparation and handling, and pad covers or a cleaning protocol; that was done in the previous
revision and is kept.

What changed in response:
- **Construct validity (not met), and stand-ins that cling (real trial).** The printed stand-ins keep their sizes and
  masses and are now dipped before each trial in a cooked cornstarch paste (6.0 g Argo 100% Pure Corn Starch in 100 g
  water, microwaved until translucent, cooled to 20-25 degrees C, made fresh each session and thrown away after 4 h)
  and drained 10 s. Starch is what makes cooked rice stick, and the coat makes the pieces cling to the steel and to
  each other. Where: task.json setup (4) and reset_notes; README "The scraps in a real trial". Check: a cling check at
  the start of each session and every 10 trials: on its side for 10 s and upside down for 5 s, no piece moves; at one
  firm shake (a 5 cm jab toward the mouth and back in about 0.2 s), every piece falls; with the fix for each failure.
  The old dry-slide check is gone.
- **Rig protection for the paste.** Nitrile pad covers (fingertips of AMMEX GWBN46100 gloves), the basket rinsed and
  dried at every reset, the stand-ins washed at the end of each session. Where: README "Real-trial safety and rig
  protection"; reset_notes.
- **Represent the handling difficulty in simulation.** Each scrap is bonded to the basket where it lies (a fixed
  joint). The bond is switched off for good once its cling load tops 2.0 times the scrap's weight: the pull away from
  the cup's floor and the shear beyond friction, from the scrap's acceleration averaged over 12.5 ms. Turned over and
  held still, the load is 1.0. Where: the task file (`cling_load`, `_cling_step`, `_author_cling`); fidelity; scoring;
  README "How it is modelled". Check: the calibration run (`SINK_TEST=shakes`) measured at most 0.47 for lifting,
  handing over and carrying, 1.04 for turning over, 1.50 for the previous revision's gentle shake, and 2.44 or more on
  every scrap for a 4 cm jab; the no-simulator self-test prints the rule's loads for set cases.
- **Adversarial check, tipping without shaking** (`SINK_TEST=noshake`, the demo with the shakes replaced by an equal
  hold). Turned over at 104 degrees the cling load peaked at 1.04 (1.37 over the whole run); all nine scraps stayed in
  the basket, went back into the drain with it, and none reached the caddy: 0.0. Where: README "Checks of the cling".
- **Neighbouring bypass, a gentle shake** (`SINK_TEST=gentle`, the previous revision's two slow 2.5 cm strokes). The
  load peaked at 1.50 at the strokes; all nine stayed in: 0.0. So the scraps come out only with a firm shake. Real trial: the same
  limits are the cling check's (still on its side and upside down: stays; one firm jab: falls).
- **Neighbouring bypass, the basket set in the caddy with the scraps still stuck in it.** A scrap now counts in the
  caddy only out of the basket's cup. Where: the task file (`_follow_inner`, `score`), task.json scoring, README "Run
  and score a trial". Check: `SINK_TEST=binbasket` left all nine inside the caddy's volume and in the basket, hands
  off: 0.0 (0.5 under the old count); the no-simulator self-test counts 0 of 9 for that pose, and counts a pea on the
  caddy's floor beside the basket.
- **An updated full-score demo.** The left now shakes the turned-over basket with two 5 cm jabs toward the caddy, each
  out in 0.1 s and straight back in 0.1 s. All nine scraps came loose at the first jab (loads 2.04 to 2.93) and fell
  in; the demo scored 1.0 in 1,291 of the 1,600 steps. Dream-1 scored 0.0: the basket never left the drain.

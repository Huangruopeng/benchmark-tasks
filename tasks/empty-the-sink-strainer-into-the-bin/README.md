# Empty the sink strainer into the bin

A stainless single-bowl kitchen sink is set into the counter on the right. It is a 25 x 22 in top-mount sink with a
bowl 21 x 15-3/4 in and 20 cm deep, and a gooseneck tap stands on its back deck. In its 3-1/2 in drain sits a
fixed-post strainer basket, 79.4 mm across and 68 g, holding nine food scraps: two strips of carrot peel, a piece of
potato peel, two clumps of cooked rice, two peas, a piece of broccoli and a piece of apple core. A 1.8 l food-waste
caddy with its lid off stands on the counter just left of the sink.

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
- **The scraps.** Each is a rigid shape at a real size, with a mass estimated from its volume:
  - the peels are 1.5 mm slabs;
  - the peas are spheres;
  - each rice clump is two spheres;
  - the broccoli is a head on a stalk;
  - the apple core is a block.

  Wet food on steel is given a friction of 0.40.
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
  are volume times density (our estimate).
- **Tap.** A generic gooseneck: 41 cm tall, a 21 cm reach, and its spout 25 cm above the deck.

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

  Real trial: by eye and by touch, the basket sits level and does not rock.
- **Hands off.** Neither gripper touches the basket, and each is at least 5 cm from it.

**Tiers.**
- 1.0: all nine scraps in the caddy, the basket seated and both hands off it.
- 0.5: all nine scraps in the caddy.
- 0.25: at least five scraps in the caddy.
- Otherwise 0.0.

**Failures: 0.0 for good, at any time.**
- **Down the drain.** A scrap's middle inside the strainer body's bore, below the sink floor, while the basket is not
  seated. A scrap carried in the basket does not count. Real trial: lift the basket out at the end and look into the
  strainer body and tailpiece.
- **On the counter.** A scrap's middle within 15 mm of the counter, outside the bowl and the caddy. This includes the
  sink's rim and the counter right beside the caddy. Real trial: look over the counter.
- **On the floor.** A scrap or the basket on the floor.
- **Dropped.** The basket falling faster than 0.6 m/s with no finger pad on it. Real trial: judged from the video.

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
- how many scraps are in the caddy, and the failure latches.

Once a second it also prints every scrap's position in the caddy's frame.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 1,600 steps (306 s on the clock, with Dream answering in 0.72 s on average), and about 5 minutes for the scripted demo (1,256 steps, 62.8 s of simulated time, 277 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

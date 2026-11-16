# Count ten tablets out of a stock bottle on a pill counting tray

A pharmacy dispensing counter. At the back, behind the counter, stands white steel pharmacy shelving full of stock
bottles. On the counter:
- a 100-count stock bottle with 16 tablets left in it, closed by its plain (not child-resistant) screw cap;
- a right-handed pill counting tray, with its stainless counting spatula standing in the slot on the tray's left side.

The robot is told: "Count out exactly 10 tablets from the stock bottle into the counting tray's trough, using the spatula, and close the trough's lid. Leave every other tablet in the stock bottle or on the tray's counting floor, and screw the bottle's cap back on. Never touch a tablet with a gripper or let one fall onto the counter."

This is how pharmacy technicians count out a prescription by hand, many times a day:
1. Open the stock bottle: hold it and unscrew its cap.
2. Pour some tablets onto the tray's counting floor.
3. With the spatula, slide them a few at a time over the rib into the trough down the tray's right side, until
   exactly ten are in.
4. Close the trough's lid over the ten, and screw the cap back on the stock bottle.

The count must be exact, and no tablet is touched by a finger at any point. (Afterwards the ten go out of the trough's
spout into the prescription vial and the rest back into the stock bottle. That second half is not part of this task.)

What it tests: a real container to open (a 38-400 screw cap that takes 1.0 N m to break loose, unscrewed in regrip
strokes), a pour that has to land on the counting floor without dropping a tablet, an exact count made with a tool on
small, slick, identical tablets, and the cap screwed back down.

Why two hands: the bottle (17 g) has to be held while its cap is unscrewed and screwed back on. Turned on its own, it
turns with the cap on the counter long before the cap's 1.0 N m breaks loose. While the right hand counts with the
spatula, the left also holds the tray still by its left wall, as pharmacy staff do.

One-hand check (the left arm left at rest, the right running the demo's moves): it scored 0.0.
- The right hand's five strokes turned the whole bottle on the counter; the cap never turned on the neck.
- Lifting the cap lifted the closed bottle with it, and the bottle fell over on its side.
- The bottle was never opened.

In the scripted demo, which reaches full score with fixed waypoints:
1. The left hand takes the stock bottle round its body. The right unscrews the cap in 5 strokes of 100 degrees,
   letting go and turning back between strokes, lifts it off the neck and sets it down beside the bottle.
2. The left lets go. The right takes the open bottle round its body and carries it over the counting tray. It tips the
   bottle 135 degrees about its own wrist in 12 steps, then shakes it 16 times along its axis. 15 tablets land on the
   counting floor and 1 stays in the bottle. It stands the bottle back left of the tray.
3. The left hand pinches the tray's left wall to hold the tray still. The right lifts the spatula out of its slot by
   its handle and makes 3 strokes. Each time, the blade comes down beside a group with its tip tilted down 5-6.5
   degrees, so only the part near the tip pushes. It sweeps the group across the floor, up the rib and into the
   trough: 5, then 1, then 4 tablets, 10 in all. Then it lays the spatula down right of the tray.
4. The right pushes the open lid over, and it falls shut. The left lets go of the tray.
5. The left takes the bottle round its body. The right picks up the cap, sets it on the neck and screws it on
   clockwise in 5 strokes until it is seated. Both hands go back.

The task is one task file, [`pour_ten_counted_tablets_into_a_held_vial.py`](pour_ten_counted_tablets_into_a_held_vial.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

## Scoring

Every tablet is followed throughout the trial. In the bottle: its centre inside the bottle. In the trough: inside the
tray's trough or its spout. On the counting floor: over the tray's floor or its return spout. Down: outside the bottle
and the tray, its centre at most 1.2 cm over the counter. Touched: a finger pad pushing a tablet with more than
0.05 N.

- 0.0 for the whole trial (latched, never reset) if a tablet is ever down or ever touched.
- Otherwise, at the end, if the bottle was opened (its cap unscrewed off its thread), exactly 10 tablets are in the
  trough and every other tablet is in the bottle or on the counting floor:
  - 1.0 with the trough's lid shut (within 8 degrees of closed) and the bottle's cap screwed back down (turned at least
    300 degrees after its thread caught again, and within 0.6 mm of its seat);
  - 0.5 with the lid shut;
  - 0.25 with the lid open.
- Else 0.0: 9 or 11 in the trough earns nothing, and so does a tablet left anywhere else.

`success()` is the full score with every moving part still. The trial ends at the run limit, or once success has held
for 1 s.

**In a real trial** the tablets look alike, and none needs telling apart: the score depends only on counts. After the
run, open the trough's lid and count the tablets in it (exactly 10), then those in the bottle and on the counting floor
(the other 6). The photo and the three videos must show no tablet on the counter or the floor at any moment, and no
finger pushing a tablet. The lid is shut when it lies on the trough. The cap is screwed down when it stops turning,
with its skirt at the neck's ledge.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The shelving, tray, lid, spatula, bottle and cap
are rigid bodies built from such shapes with `bodykit.py`; each tablet is a rigid disc, 10 mm across and 4 mm thick.

- **Shelving:** fixed, past the counter's back edge, drawn only (out of reach).
- **Bottle:** a ring of thin wall panels, a shoulder and a 38-400 neck. Hidden collision-only linings line the inside
  3 mm thick, up the body and on through the shoulder to the neck's bore, and the floor collides 3 mm thicker. Without
  them, a fast tablet could pass through the 1 mm wall. So the bore collides 45 mm wide; a real one is about 51 mm.
- **Cap and thread:** the thread is a joint that keeps the cap on the neck's axis. The task file makes it a screw:
  - each turn moves the cap 4.2 mm along the neck;
  - a friction spring holds the cap until 1.0 N m breaks it loose, then 0.3 N m (stick-slip, re-anchored every step);
  - after one full turn off, the joint lets the cap go;
  - set back on the neck within 6 mm of its axis and 12 degrees of upright, and turned 20 degrees clockwise, it
    catches its thread again. Each clockwise turn then draws it 4.2 mm down, against 0.3 N m, to its seat.
- **Tray (100 g):** the floor, walls, rib, the trough's half-tube, both spouts and the spatula slot. A hidden 10 mm
  slab under the counting floor keeps falling tablets from passing through it.
- **Lid:** a half-tube on a hinge along the trough's outer rim, free from 0 (shut) to 110 degrees (open).
- **Spatula:** a 0.8 mm steel blade with a round tip on a wooden handle, 30 g, its centre of mass 8 mm from the
  junction toward the blade.
- **Tablets:** film-coated tablets are slick. In every contact they take the lower of the two frictions, 0.25.

After every control step the task file:
- notes where each tablet is (in the bottle, the trough, on the counting floor, down, or in the air);
- latches a tablet that is down, or pushed by a finger pad (read from a contact view between the tablets and the four
  finger pads);
- follows the cap's turns on the neck, and reads the lid's angle on the tray.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with everything still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos. Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320
-E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pour_ten_counted_tablets_into_a_held_vial.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Count out exactly 10 tablets from the stock bottle into the counting tray's trough, using the spatula, and close the trough's lid. Leave every other tablet in the stock bottle or on the tray's counting floor, and screw the bottle's cap back on. Never touch a tablet with a gripper or let one fall onto the counter." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/tablets-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pour_ten_counted_tablets_into_a_held_vial.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Count out exactly 10 tablets from the stock bottle into the counting tray's trough, using the spatula, and close the trough's lid. Leave every other tablet in the stock bottle or on the tray's counting floor, and screw the bottle's cap back on. Never touch a tablet with a gripper or let one fall onto the counter." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/tablets-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tablets-dream/*.json; do inspect-robots video "$log"; done
```

Set `TAB_DEBUG=1` to print what the score is built from twice a second: the cap's turns and slide on the neck, the
lid's angle, the bottle, the cap, the tray and both hands, how many tablets are in the bottle, on the counting floor,
in the trough, in the air or down, the touch force and the latches. Add `TAB_POS=1` to print every tablet's place
every 2 s.

## Runtime per trial

On one RTX 5090: about 26 minutes for a Dream-1 trial of 3,000 steps (1568 s on the clock, with Dream answering in 0.70 s on average), and about 24 minutes for the scripted demo (2,791 steps, 139.6 s of simulated time, 1413 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Realism check

- **The real products, at their real sizes.**
  - Stock bottle: a 150 cc white HDPE round packer, 53.3 mm across and 93.0 mm tall, 17 g, with a 38-400 neck: the
    usual bottle for 100 tablets. Plain (not child-resistant) 100-count bottles of tablets are sold, for example NDC
    47335-859-08 in the FDA's listing.
  - Cap: a ribbed PP 38-400 screw cap. 38 mm caps are applied at about 17-26 in-lb (2-3 N m) and come off at about
    half that, so 1.0 N m to break it loose.
  - Counting tray: the common right-handed design (US patent 2,530,009: a trough with a cover, a rib along the floor's
    edge, and a spout at each end). Sizes from Apothecary Products 23087 (8.35 x 5.87 x 1.57 in) and PharmaSystems
    D-50 (21 x 16 x 3.5 cm): modelled 21.2 x 14.9 x 3.8 cm. The 6 mm front lip and the 1 cm slot are our estimates.
  - Spatula: Apothecary Products 23317, a 5 in (127 mm) stainless blade on a wooden handle, sold "for sorting and
    counting pills". The handle's 19 x 11 mm section is our estimate.
  - Tablets: 10 mm round, 4 mm thick, 0.35 g.
  - Shelving: steel pharmacy shelving, 36 in sections, 12 in deep, 84 in tall.
- **The real place.** A dispensing counter in front of the stock shelves.
- **Every real step of the count.** Open the bottle, pour onto the tray, slide the tablets into the trough with the
  spatula until there are exactly ten, close the lid, recap the bottle. All of it is physical; nothing is done by a
  rule. The second half of filling a prescription (the ten into the vial, the rest back into the stock bottle) is
  left out. Done with one hand, it means lifting and tipping the 100 g tray, and one YAM gripper cannot hold a
  real-size tray's tilt: in our trials it turned 15-60 degrees out of the commanded pose.
- **Hidden helpers, all listed in fidelity:** the cap's thread joint, and the collision-only linings in the bottle and
  under the tray's floor. None changes a size you can see.

## Changes in this revision

This is a revision of TK-VLDP6K. Its review said:

> The task should involve the opening of a container and counting the pills

The review before that (TK-J3KV8V) said:

> Construct validity (not met): Pouring the six uncounted tablets directly into the vial earns credit without transferring any tablets from the specified trough.
>
> Scoring (not met): The partial scores reward tablets from the wrong side of the tray and permit all remaining tablets to be spilled.
>
> What to change: Revise the partial scores so transferring uncounted tablets or spilling the remaining tablets cannot earn credit. Specify how counted tablets and spills are verified in a real-world trial, given that the tablets look identical.

Its requests were met in the previous revision and are kept here, applied to the new task (see the third bullet).

What changed in response:

- **Opening a container:** the task now starts with a closed 100-count stock bottle: 150 cc HDPE, 16 tablets left,
  a plain 38-400 screw cap.
  - One hand holds the bottle while the other unscrews the cap. It takes 1.0 N m to break loose and 0.3 N m to keep
    turning, and comes off after one full turn of thread.
  - At the end the cap is screwed back down: at least 300 degrees after its thread catches, and within 0.6 mm of its
    seat.
- **Counting the pills:** the tablets are poured out onto a real-size pill counting tray (21.2 x 14.9 cm, a trough
  down its side under a hinged lid). They are counted into the trough with a stainless counting spatula, a few at a
  time, until exactly 10 are in, and then the trough's lid is closed.
  - Full credit needs exactly 10 tablets in the trough with the lid shut, every other tablet in the bottle or on the
    counting floor, and the cap screwed back down.
- **The earlier review's loopholes stay closed:**
  - A tablet on the counter or the floor at any moment, or one pushed by a finger pad, scores 0.0 for the whole
    trial.
  - 9 or 11 tablets in the trough earn nothing, and so does a tablet left anywhere but the trough, the bottle or the
    counting floor.
  - Tested from set layouts, with the cap off and the lid then pushed shut by the demo's moves:
    - 10 in the trough scored 0.5 (the control: the cap was not on);
    - 11 in the trough scored 0.0;
    - 10 in the trough with one tablet on the counter scored 0.0.
- **Real-world check with identical tablets:** only counts are needed. After the run, open the lid and count exactly
  10 in the trough, then the other 6 in the bottle and on the counting floor. The photo and the three videos must show
  no tablet on the counter or the floor at any moment, and no finger pushing one.
- **Scene and texts:** a pharmacy dispensing counter, with stock shelving behind it. The earlier pre-counted tray
  and the vial are gone. New title, instruction, setup, scoring and fidelity; a new demo (1.0) and a new Dream-1 run.

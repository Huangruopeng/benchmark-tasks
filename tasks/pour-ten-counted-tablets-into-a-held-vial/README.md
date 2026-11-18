# Count ten tablets out of a stock bottle on a pill counting tray

A pharmacy dispensing counter. At the back, behind the counter, stands white steel pharmacy shelving full of stock
bottles. On the counter:
- a 100-count stock bottle with 16 tablets left in it, closed by its plain (not child-resistant) screw cap;
- a right-handed pill counting tray, with its stainless counting spatula standing in the slot on the tray's left side.

The robot is told: "Count out exactly 10 tablets from the stock bottle: pour some onto the counting tray's floor and slide them into the tray's trough with the spatula until exactly 10 are in, then close the trough's lid. Leave every other tablet in the stock bottle or on the counting floor, and screw the bottle's cap back on. Never pour a tablet straight into the trough, touch one with a gripper, or let one fall onto the counter."

This is how pharmacy technicians count out a prescription by hand, many times a day:
1. Open the stock bottle: hold it and unscrew its cap.
2. Pour some tablets onto the tray's counting floor.
3. With the spatula, slide them a few at a time over the rib into the trough down the tray's right side, until
   exactly ten are in.
4. Close the trough's lid over the ten, and screw the cap back on the stock bottle.

The count must be exact, every tablet goes into the trough on the spatula's push, and no tablet is touched by a finger
at any point. (Afterwards the ten go out of the trough's spout into the prescription vial and the rest back into the
stock bottle. That second half is not part of this task.)

What it tests: a real container to open (a 38-400 screw cap that takes 1.0 N m to break loose, unscrewed in regrip
strokes), a pour that has to land on the counting floor without dropping a tablet, an exact count made with a tool on
small, light, identical tablets (only tablets the spatula pushes from the counting floor into the trough count), and
the cap screwed back down.

Why two hands: the bottle has to be held while its cap is unscrewed and screwed back on. Standing free, it turns with
the cap on the counter long before the cap's 1.0 N m breaks loose: the bottle and its 16 tablets weigh 24.8 g, so even
at a friction of 1.0 the counter resists its turning with under 0.005 N m (computed: 2/3 x friction x weight x its
26.65 mm radius). While the right hand counts with the spatula, the left also holds the tray still by its left wall,
as pharmacy staff do.

One-hand check (the left arm left at rest, the right running the demo's moves, on this revision): it scored 0.0.
- With nothing holding the bottle, the right hand's five strokes never turned the cap on the neck (it stayed within 2
  degrees of where it started).
- Lifting the cap lifted the closed bottle 3 cm with it, and the bottle fell over on its side.
- The bottle was never opened.

In the scripted demo, which reaches full score with fixed waypoints:
1. The left hand takes the stock bottle round its body. The right unscrews the cap in 5 strokes of 100 degrees,
   letting go and turning back between strokes, lifts it off the neck and sets it down beside the bottle.
2. The left lets go. The right takes the open bottle round its body and carries it over the counting tray. It tips the
   bottle 135 degrees about its own wrist in 12 steps, then shakes it 16 times along its axis. 14 tablets land on the
   counting floor and 2 stay in the bottle. It stands the bottle back left of the tray.
3. The left hand pinches the tray's left wall to hold the tray still. The right lifts the spatula out of its slot by
   its handle and makes 4 strokes. Each time, the blade comes down beside a group with its tip tilted down 5-6.5
   degrees, so only the part near the tip pushes. It sweeps the group across the floor, up the rib and into the
   trough: 4, then 2, then 3, then 1, 10 in all. Then it lays the spatula down right of the tray. Every one of the ten
   entered the trough straight from the counting floor in the spatula's pushed group, 0 to 0.40 s (0 to 8 control
   steps) after the blade last touched it or the tablets behind it: the ones pushed onto the rib's crest topple in just
   after the blade stops (from the demo run with `TAB_DEBUG=1`).
4. The right pushes the open lid over, and it falls shut. The left lets go of the tray.
5. The left takes the bottle round its body. The right picks up the cap, sets it on the neck and screws it on
   clockwise in 5 strokes until it is seated. Both hands go back.

The task is one task file, [`pour_ten_counted_tablets_into_a_held_vial.py`](pour_ten_counted_tablets_into_a_held_vial.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

## Scoring

Every tablet is followed after every control step (0.05 s). In the bottle: its centre inside the bottle. In the
trough: inside the tray's trough or its spout. On the counting floor: over the tray's floor or its return spout.
Down: outside the bottle and the tray, its centre at most 1.2 cm over the counter.

**Counted in by the spatula.** A tablet's entry into the trough counts only when all of these hold:
- it comes straight from the counting floor;
- it has lain on the counting floor at least 1 s (so a tablet poured or bounced onto the floor can't carry on in);
- the spatula pushed it within the last 1 s: the blade touched it, directly or through a chain of touching tablets
  (the blade pushes a group of five, and the front tablets are pushed by the ones behind). Contact is read from two
  PhysX contact views: each tablet against the spatula, and each tablet against each other tablet;
- the tray is within 5 degrees of level (sitting on the counter, not tipped).

Every entry is checked, not just the first. A tablet that leaves the trough loses its count, and coming back in is a
new entry, checked the same way, with a new push of the spatula after it left.

- 0.0 for the whole trial (latched, never reset) if:
  - a tablet is ever down;
  - any part of either arm (gripper, fingers or any link) ever touches a tablet: any contact force over 0.0001 N, a
    48th of a tablet's weight;
  - a tablet ever enters the trough any other way: poured or dropped from the bottle, bounced in, tipped in, or pushed
    by anything but the spatula.
- Otherwise, at the end, if the bottle was opened (its cap unscrewed off its thread), exactly 10 tablets are in the
  trough, each counted in by the spatula and lying inside the trough's tube (its centre no higher than 16 mm above
  the tube's axis, so none rests on top of the lid), and every other tablet is in the bottle or on the counting floor:
  - 1.0 with the trough's lid shut (within 3 degrees of closed) and the bottle's cap screwed back down (turned at
    least 300 degrees after its thread caught again, and within 0.6 mm of its seat);
  - 0.5 with the lid shut;
  - 0.25 with the lid open.
- Else 0.0: 9 or 11 in the trough earns nothing, and so does a tablet left anywhere else.

The lid reads 0.0 degrees when shut. A tablet caught under its free edge (37.5 mm from the hinge) would hold it 6.1
degrees open (computed), so it would not count as shut.

`success()` is the full score with every moving part still. The trial ends at the run limit, or once success has held
for 1 s.

**In a real trial** the tablets look alike, and none needs telling apart.
1. **Count.** After the run, open the trough's lid and count the tablets in it (exactly 10), then those in the bottle
   and on the counting floor (the other 6).
2. **The spatula count, from the overhead camera's video, stepped frame by frame through the count** (from the first
   stroke to the lid closing). At each frame where a tablet crosses the trough's inner rim, note the frame and check:
   - the tablet had been lying still on the counting floor before the stroke that moved it;
   - the blade was pushing it, touching it or the tablets in front of it, within the 1 s before it crossed.

   Tally the crossings: in minus out must equal 10, matching the count. A crossing that fails either check scores 0:
   a tablet poured or dropped from the bottle, bounced in, tipped in by lifting the tray, or pushed by a finger or the
   bottle.
3. **Whole-trial checks.** The photo and the three videos must show no tablet on the counter or the floor, no gripper
   touching a tablet, and the tray flat on the counter throughout the count.
4. **End state.** The lid lies flat on the trough with nothing under its edge. The cap is screwed down when it stops
   turning, with its skirt at the neck's ledge.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The shelving, tray, lid, spatula, bottle and cap
are rigid bodies built from such shapes with `bodykit.py`; each tablet is a rigid disc, 10 mm across and 4 mm thick,
0.49 g.

- **Shelving:** fixed, past the counter's back edge, drawn only (out of reach).
- **Bottle:** a ring of 1 mm wall panels (53.3 mm outside, 51.3 mm inside), a shoulder and a 38-400 neck (37.6 mm
  over the thread, a 31.0 mm bore). Hidden collision-only linings line the body's inside 3 mm thick and run on
  through the shoulder to the neck, ending flush with the 31.0 mm bore; the floor collides 3 mm thicker. Without them,
  a fast tablet could pass through the 1 mm wall. So inside the body a tablet meets the lining 45.1 mm across, while
  the neck's 31.0 mm bore, where the tablets pour out, is exactly as seen.
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
- **Tablets:** in every contact they take the lower of the two frictions, 0.25.

After every control step the task file:
- notes where each tablet is (in the bottle, the trough, on the counting floor, down, or in the air);
- reads two PhysX contact views: each tablet against every link of both arms and the spatula, and each tablet against
  each of the other 15;
- finds the spatula's pushed group (the tablets touching the blade, and those touching them, and so on), and checks
  every entry into the trough as above;
- latches a tablet that is down, touched by an arm, or entered the trough any other way;
- follows the cap's turns on the neck, and reads the lid's angle on the tray and the tray's tilt.

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
  --instruction "Count out exactly 10 tablets from the stock bottle: pour some onto the counting tray's floor and slide them into the tray's trough with the spatula until exactly 10 are in, then close the trough's lid. Leave every other tablet in the stock bottle or on the counting floor, and screw the bottle's cap back on. Never pour a tablet straight into the trough, touch one with a gripper, or let one fall onto the counter." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/tablets-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pour_ten_counted_tablets_into_a_held_vial.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Count out exactly 10 tablets from the stock bottle: pour some onto the counting tray's floor and slide them into the tray's trough with the spatula until exactly 10 are in, then close the trough's lid. Leave every other tablet in the stock bottle or on the counting floor, and screw the bottle's cap back on. Never pour a tablet straight into the trough, touch one with a gripper, or let one fall onto the counter." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/tablets-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tablets-dream/*.json; do inspect-robots video "$log"; done
```

Set `TAB_DEBUG=1` to print what the score is built from twice a second (the cap's turns and slide on the neck, the
lid's angle, the tray's tilt, both hands, how many tablets are in the bottle, on the counting floor, in the trough, in
the air or down, the touch force, the latches and the counted tablets), and one line for every entry into or exit from
the trough, saying whether it counted and why. Add `TAB_POS=1` to print every tablet's place every 2 s.

## Runtime per trial

On one RTX 5090: about 22 minutes for a Dream-1 trial of 3,000 steps (1338 s on the clock, with Dream answering in 0.60 s on average), and about 30 minutes for the scripted demo (2,909 steps, 145.4 s of simulated time, 1812 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

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
    D-50 (21 x 16 x 3.5 cm): modelled 21.2 x 14.9 x 3.8 cm, with a 6 mm front lip and a 1 cm spatula slot (design
    values).
  - Spatula: Apothecary Products 23317, a 5 in (127 mm) stainless blade on a wooden handle, sold "for sorting and
    counting pills". Handle section 19 x 11 mm (design value).
  - Tablets: white Smarties (see below), about 10 mm across and 4 mm thick, 0.49 g.
  - Shelving: steel pharmacy shelving, 36 in sections, 12 in deep, 84 in tall.
- **The real place.** A dispensing counter in front of the stock shelves.
- **Every real step of the count.** Open the bottle, pour onto the tray, slide the tablets into the trough with the
  spatula until there are exactly ten, close the lid, recap the bottle. All of it is physical; nothing is done by a
  rule, and the scorer follows each tablet's way into the trough. The second half of filling a prescription (the ten
  into the vial, the rest back into the stock bottle) is left out. Done with one hand, it means lifting and tipping the
  100 g tray, and one YAM gripper cannot hold a real-size tray's tilt: in our trials it turned 15-60 degrees out of the
  commanded pose.
- **Hidden helpers, all listed in fidelity:** the cap's thread joint, and the collision-only linings in the bottle and
  under the tray's floor. None changes a size you can see.

## Real-trial materials, safety and rig protection

- **Tablets: an inert, repeatable stand-in.** 16 white Smarties Original candy tablets (Smarties Candy Company, Union,
  NJ), the same in every trial:
  - compressed dextrose discs, about 10 mm across and 4 mm thick (encyclopedia figure), 15 to a roll;
  - 0.49 g each, computed from the maker's label: a 14 oz (397 g) bag of Original rolls holds 54 rolls, 7.35 g a roll;
  - ingredients dextrose, citric acid, calcium stearate, flavour and colour lakes; free from peanuts, tree nuts,
    milk, eggs, soy and wheat (maker's FAQ, <https://smarties.com/faqs>);
  - sold in grocery stores in 14 oz bags of rolls (for example
    <https://foodlion.com/groceries/candy-chocolate/hard-candy-lollipops/smarties-original-candy-rolls-14-oz-bag.html>).
    Each roll mixes six colours; pick the white ones.
- **Handling.** Put on clean dry gloves to set them out. Keep the 16 in the closed stock bottle between trials, and
  keep them dry (no liquid on the rig). Before each session check them: replace any chipped or broken tablet from a new
  roll; replace the whole set once a day, or sooner if they turn sticky. They are never eaten. At the end of a session
  wipe the tray, the spatula and the bottle with a dry cloth.
- **Reset.** Tip the trough's tablets out of its spout into the stock bottle and slide the rest into it, so all 16 are
  back in the bottle; screw the cap down; open the lid fully; stand the spatula in its slot.
- **Rig protection.** The grippers never touch a tablet (that scores 0). The parts they hold are dry plastic and
  wood: the bottle, the cap, the spatula's handle and the tray's wall. Nothing is sharp; the blade's tip is round.
  Opening the cap takes 1.0 N m, a squeeze of a few newtons on the 40.5 mm cap.
- **Cost.** Ordinary pharmacy and grocery stock: the right-handed counting tray with its spatula (Apothecary Products
  23087) is $20.45 at WholesalePoint (<https://www.wholesalepoint.com/products/apothecary-23087.html>); the 150 cc
  packer bottle with its 38-400 cap and a bag of Smarties are everyday purchases.

## Third-party assets

None. Every body is built from boxes and cylinders in the task file.

## Changes in this revision

This is a revision of TK-WYS8RE. Its review said:

> Flags: Note for the team: Reconcile the 31 mm setup bore with the 45 mm simulated and approximately 51 mm real bore claims in the fidelity note. · Note for the team: Use consistent inert tablet stand-ins for repeatable, safe real trials.
>
> Construct validity (not met): A robot can pour exactly ten tablets directly from the bottle into the trough and earn full credit without using the spatula.
>
> Scoring (not met): The scorer checks the final count but does not verify the instructed spatula transfer, so it rewards a bypass of the task's core manipulation.
>
> What to change: The earlier feedback is addressed: the task now opens a bottle and counts tablets, and spills cannot earn credit. A new issue not raised previously is that the scorer permits pouring ten tablets directly into the trough without using the spatula. Require each credited tablet to move from the counting floor into the trough using the spatula, and specify how the simulation scorer and real-trial video verify this.

The review before that (TK-VLDP6K) asked for the opening of a container and counting the pills; the one before that
(TK-J3KV8V) asked that uncounted or spilled tablets earn nothing and that the real trial verify the counts with
identical tablets. Both were done in earlier revisions and are kept.

What changed in response:

- **Each credited tablet must move from the counting floor into the trough by the spatula (simulation).** Task file:
  `_count`, run after every control step, and `_points`; task.json "scoring"; README "Scoring". The scorer reads two
  PhysX contact views every step: each tablet against the spatula and every link of both arms, and each tablet
  against each other tablet. From them it builds the spatula's pushed group: the tablets touching the blade, the
  tablets touching those, and so on. Every entry into the trough counts only if:
  - it comes straight from the counting floor;
  - the tablet has lain there at least 1 s;
  - it was in the pushed group within the last 1 s;
  - the tray is within 5 degrees of level.

  Any other entry scores 0 for the whole trial, and full credit needs all 10 tablets in the trough counted in this
  way. Check: in the demo run with `TAB_DEBUG=1`, all 10 entries were counted, each 0 to 0.40 s after the blade's
  last contact, and the run scored 1.0.
- **Pouring ten straight from the bottle into the trough scores 0.** Check, an adversarial run from a set layout:
  the bottle opened with 10 tablets in it and 6 on the counting floor. The right hand rolled the bottle 160 degrees
  over the trough and shook it, and all 10 went straight from the bottle into the trough, with none dropped. Then the
  lid was pushed shut and the cap screwed down. The previous revision's scorer gives this run 1.0. The new one scored
  it 0.0, latched at the first tablet's entry ("came from the bottle"). Twelve more pours with other settings (3 to 11
  tablets in the trough, or one dropped) also scored 0.0.
- **How the real-trial video verifies it** (task.json "scoring" (b); README "Scoring", "In a real trial"). Step the
  overhead video frame by frame from the first stroke to the lid closing. At each frame where a tablet crosses the
  trough's inner rim, check two things: the tablet had lain still on the counting floor before that stroke, and the
  blade was pushing it (touching it or the tablets in front of it) within the 1 s before it crossed. Then tally the
  crossings: in minus out must equal 10, the count found in the trough after opening the lid. Any other crossing
  scores 0.
- **The neighbouring bypasses are closed too, each with a check:**
  - **Tipping the tray** so tablets slide into the trough: an entry needs the spatula's push and the tray within 5
    degrees of level. Check: with 10 tablets lined up beside the rib, the left hand tipped the tray 16 degrees by its
    left wall and laid it back down. One tablet then entered the trough with no push of the spatula. The entry
    latched, and the run scored 0.0.
  - **Pushing tablets with a finger or any other part of the arm:** a contact over 0.0001 N (a 48th of a tablet's
    weight) between a tablet and any of the 9 links of either arm scores 0. Before, the limit was 0.05 N on the finger
    pads only. Check: the right hand's closed fingers pushed tablets across the floor and touched them at up to
    0.017 N, below the old limit. The touch latched, and the run scored 0.0.
  - **Bouncing, flicking or tipping a tablet in, or one that leaves the trough and comes back.** The 1 s rest on the
    floor and the check on every entry stop all of these. Checked offline on the task file's own `_count` with 12
    synthetic step sequences:
    - spoiled the trial: a tablet that lands and bounces on in; one that comes in through the air; one that slides
      in with the tray tipped; one whose last push was 1.05 s before it entered; one that leaves and comes back with
      no new push;
    - counted: one pushed in by the blade, or through a touching tablet, or pushed back in after leaving.
  - **Closing the lid on a tablet:** the lid now counts as shut only within 3 degrees of closed (before: 8). It reads
    0.0 shut, and a tablet caught under its edge holds it 6.1 degrees open (computed).
  - **A tablet left on top of the shut lid:** at the end, each of the ten must lie inside the trough's tube, its
    centre no higher than 16 mm above the tube's axis. A tablet resting on the lid sits about 22 mm up (computed), so
    it does not count. The real trial counts the ten after opening the lid.
- **The bottle's numbers are reconciled** (task.json "setup" (1) and "fidelity"; README "How it is modelled"), all
  taken from the code. The body is 53.3 mm across with a 1 mm wall, 51.3 mm inside, and its hidden lining leaves
  45.1 mm. The 38-400 neck is 37.6 mm over the thread with a 31.0 mm bore. The bore is unlined, so the opening the
  tablets pour through is exactly as seen.
- **Consistent inert tablet stand-ins** (task.json "setup" (3) and "reset_notes"; README "Real-trial materials,
  safety and rig protection"): 16 white Smarties Original candy tablets, the same set in every trial. They are
  compressed dextrose discs about 10 mm across and 4 mm thick, 0.49 g each (computed from the maker's label). The
  README gives a purchase link, handling (dry gloves; kept dry in the closed bottle; chipped ones replaced; a fresh
  set daily) and reset. The simulated tablets now weigh 0.49 g to match.
- **New instruction, demo and Dream-1 run.** The instruction now says to pour onto the counting floor and slide the
  tablets in with the spatula, and never to pour one straight into the trough. Demo 1.0 (2,909 steps), Dream-1 0.0,
  one-hand check 0.0.

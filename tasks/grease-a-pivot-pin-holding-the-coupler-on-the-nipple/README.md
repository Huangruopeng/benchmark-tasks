# Grease a loader's pivot pin with a lever grease gun without leaking grease at the nipple

On the table stands the pivot of a loader arm: a yellow steel lug with a steel pin through it and, screwed into its
top, a straight grease nipple (DIN 71412 form A: M10x1 taper thread, 11 mm hex, 6.5 mm ball head). To the left a lever
grease gun lies in a cradle with its handle on top; its whip hose runs over the table to a lock-on grease coupler
standing in a steel holder cup. The job is to grease the pin through its nipple without leaking any grease at the
coupler or the nipple, and to finish with the coupler back in its holder and both arms back where they started. The
task is one task file,
[`grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py`](grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

Who does it: greasing machine pivots through their nipples is routine work for mobile heavy equipment mechanics
(O*NET task "Clean, lubricate, and perform other routine maintenance work on equipment and vehicles"; 180,300 US
workers, https://www.onetonline.org/link/summary/49-3042.00) and industrial machinery mechanics ("Clean, lubricate, or
adjust parts, equipment, or machinery"; 446,900, https://www.onetonline.org/link/summary/49-9041.00), together about
627,000 (O*NET's 2025 figures). The lever grease gun with a coupler on a whip hose is their standard tool for it.

What it tests: pushing a coupler squarely over a small ball-head fitting until its jaws lock; working a lever grease
gun's handle through repeated strokes (lift it, push it back down) only while the coupler is sealed on the nipple
(locked, or held), so that no grease leaks; pressing the coupler's thumb lever to take it off; putting it back in its
holder.

Two arms: in the demo the right arm puts the coupler on, later takes it off and puts it back, and the left arm works
the gun's handle. A locked coupler needs no hand on it, so the arms take turns and neither has to hold anything for the
other. The task does not force two arms: by the arm's inverse kinematics, the left arm reaches the holder, the gun's
handle (lifted up to 25 degrees) and the nipple (with its grip leaned 4 degrees or more, inside the coupler's 8-degree
seal tolerance), so a policy may do the whole job with the left arm. The right arm cannot reach the gun's handle (no
solution with its grip leaned up to 60 degrees).

- **One-arm check.** The right arm does its part as in the demo; the left never moves. The trial scored 0.25: the
  coupler was locked on and put back in its holder, but the gun was never worked. This shows that the pumping is
  needed, not that two arms are.

## The real rig

Everything the robot touches is a bought part or is built from stock with the drawing below. Figures marked
*published* are the maker's; *derived* ones are computed from published figures (the arithmetic is given); *estimated*
ones are ours and must be checked on the bought parts (see "Still to measure").

### The grease-fed pivot (built: one drilled and tapped steel block)

| Part | Specification | Source |
|---|---|---|
| Lug | steel flat bar 50 x 40 mm (S235JR or 1018), cut 56 mm long, stood on end, painted yellow; machined as drawn | any steel stockist |
| Base plate | steel plate 120 x 100 x 10 mm, painted black; two 9 mm countersunk holes 35 mm apart | any steel stockist |
| Pin | clevis pin without head, with split-pin holes, ISO 2340 B 20 x 100, steel | standard fastener |
| Washers, split pins | two plain washers for 20 mm clevis pins (ISO 8738, 20); two 5 mm split pins (ISO 1234) | standard fasteners |
| Side plates (the arm) | two 8 mm steel plates 74 x 34 mm, a 20.5 mm hole 21 mm from one end, painted yellow | cut from 8 mm flat bar |
| Nipple | KIPP K1132.1110100: DIN 71412 style A, straight, M10X1, hardened zinc-plated steel, *published* thread length 6 mm, length at most 16 mm, 11 mm hex ([datasheet](https://www.aftfasteners.com/content/K1132-A-Datasheet-18495-Grease-nipples-conical-head-DIN-71412-style-A-straight-enUS.pdf)). Würth 098642 is the same form with a 5.5 mm thread and 15 mm length ([page](https://eshop.wuerth.ee/Cone-grease-nipple-shape-A-straight-DIN-71412-zinc-plated-steel-shape-A-NPL-DIN71412-A-H1-A3F-WS11-HEX-M10X1/098642.sku/en/US/EUR)) | KIPP or Würth |
| Screws | two M8 x 20 countersunk screws (ISO 10642), from below through the base plate | standard fasteners |

Lug drawing (mm). It is the only machined part: a drill press, a 20.5 mm drill, a 9.0 mm drill with an M10x1 tap, a
5 mm drill and an M8 tap make it, or any machine shop.

```
FRONT VIEW, looking along the pin (mm)              TOP VIEW (mm)

      |<--------------- 50 --------------->|           |<--------------- 50 --------------->|
   -- +----------------+--+----------------+        -- +-----------------------------------+
   ^  |                |##|  M10x1, 9 deep |        ^  |                                   |
   |  |                |##|  (drill 9.0)   |        40 |                (O)  M10x1         |
   |  |                |  |  5 mm passage, |        v  |                                   |
   |  |                |  |  16.75 long    |        -- +-----------------------------------+
   |  |            .---+--+---.            |
  56  |          /              \          |           The 20.5 mm bore runs through the
   |  |         |        +       |  20.5   |           40 mm width, so the pin lies across
   |  |          \              /   through|           the 50 mm front-to-back side. The
   |  |            '----------'            |  --       nipple hole and the passage are on
   |  |                 |                  |   ^       the top face's centre line.
   |  |   [M8]          | 20          [M8] |  20
   v  +-----------------+------------------+  --       Two M8 holes, 12 deep, in the foot,
           |<- 17.5 ->|<- 17.5 ->|                     17.5 mm each side of the bore's axis.
```

Assembly: drill the 20.5 mm bore with its axis 20 mm above the foot; drill and tap the M10x1 hole 9 mm deep in the
top face's centre, and continue it with the 5 mm passage into the bore (16.75 mm of steel between the hole's bottom and
the bore); tap the two M8 holes in the foot (they clear the bore). Screw the lug onto the base plate from below. Put a
washer and a side plate on each side, push the pin through, fit the split pins, screw in the nipple (11 mm spanner,
hand tight plus a quarter turn) and clamp the base plate to the table. Grease volumes (*derived* from the drawing): the
bore's clearance round the 20 mm pin, pi/4 x (20.5² - 20²) x 40 = 0.64 cm³; the passage, pi/4 x 5² x 16.75 = 0.33 cm³;
the M10x1 hole below the nipple's thread, pi/4 x 9² x 3.5 = 0.22 cm³; together about 1.2 cm³. Before the first trial,
work the gun by hand until fresh grease comes out at both ends of the bore, so that every later stroke pushes its
grease straight out there.

### The pump: a Lincoln 1145 lever grease gun (bought)

- **The gun:** Lincoln 1145 heavy-duty dual-pressure lever grease gun, set to its high-pressure position (load pin in
  front). *Published* (Lincoln hand-held lubrication catalog,
  [PDF](https://files.valinonline.com/userfiles/documents/lincoln-hand-held-lubrication-accessories-2009.pdf)):
  "With handle in FRONT(hi) shift position, grease is delivered at high pressure (maximum 7000 psi) with strokes from
  1/4" to 9". Output 1 ounce per 33 full strokes", using No. 1 cup grease at 70 °F; 21 oz bulk or
  14.5 oz cartridge; barrel diameter 2-1/4 in.; length 25-1/4 in. with its 6 in. extension. Price: C$142.50 at one
  distributor ([ITM](https://www.itm.com/product/lincoln-industrial-1145-grease-gun), which lists the same outputs).
- **Its stroke (derived):** 1 oz is 28.35 g, so a full stroke gives 28.35 / 33 = 0.86 g, about 0.95 cm³ of grease at
  ~0.9 g/cm³ (the density is a typical value for lithium grease, not the maker's). The 9 in. (229 mm) full stroke is
  measured at the handle's grip; with the grip's middle 24.5 cm from the pivot (*estimated*), that is a 53.5-degree
  swing, so the plunger gives about 0.018 cm³ per degree of handle swing. A stroke as the task counts it (the handle
  lifted past 20 degrees and pushed back down within 3 degrees of its stop) gives at least 17 x 0.018 = 0.30 cm³; the
  demo's 25-degree strokes give 0.45 cm³ each.
- **Its force (derived):** pushing the handle down drives grease against the line's pressure p; the work balances, so
  the torque on the handle is p x 0.95 cm³ / 53.5 degrees (0.93 rad), that is p x 1.02 cm³ per radian. At the demo's
  ~17 degrees a second the simulation's damper takes 1.28 N m, which is 4.9 N at the robot's grip point (26 cm from
  the pivot) and 12.5 bar in the line. A real rig is checked before trials: with the coupler locked on and the pin
  turning freely, pushing the handle down at about one stroke in 3 s must take at most 15 N on a spring scale hooked to
  the grip's middle (3.7 N m, at most 36 bar in the line). The handle's own 0.30 kg (*estimated*) adds 1.9 N at the
  grip, downward.
- **Whip hose:** Lincoln 71518, 18 in. (457 mm), 1/8 in. NPT (male) thread, 3,000 psi (*published*, same catalog).
  It replaces the gun's 6 in. rigid extension.
- **Coupler:** a four-jaw lock-on coupler with a thumb lever, the kind sold as a locking or hands-free coupler, for
  example the Lubretec G-Lock (ME-0141087032, 32 mm long, or ME-0141087081, 81 mm; M10x1 swivel thread; "press the
  lever with the thumb and insert"; "uncoupling over 600 bar is possible",
  [datasheet](https://media-new.imu.nl/storage/lubretec.com/33142/lpi-g-lock-en.pdf)), with an adapter to the hose's
  1/8 NPT thread. The modelled coupler (4.8 cm tall, side outlet on a brass swivel) is our specification of that type.
- **Grease:** a 14.5 oz cartridge of NLGI 2 lithium grease, white, so that any smear shows on the black pad covers and
  on a white wipe.
- **Cradle:** a 6 mm plywood board 400 x 80 mm clamped to the table, two wooden saddles with a 57 mm half-round cut
  (24 and 46 cm left of the middle), and two hose clamps (to 60 mm) round barrel and saddles. The gun lies on it with
  its handle on top, its head 10.5 to 18 cm left of the middle and its outlet facing right. In the shop a mechanic
  holds the barrel in the other hand while working the handle; the cradle does that holding here, and the coupler,
  the hose and the handle are worked as in the shop.
- **Stroke card:** a card standing behind the handle with a line at 20 degrees above the handle's rest, so that strokes
  can be counted from the video.

### The coupler and the nipple

- **Coupler (modelled):** 82 g with its thumb lever, 4.8 cm tall: a zinc-plated hardened steel nose 18 mm across and
  12 mm tall, whose closed jaws form a 7.2 mm mouth with a 45-degree lead-in (10.4 mm across at the face), running 5 mm
  up to a nitrile seal ring 2 mm thick with a 4.6 mm hole. Above the nose sit a black-oxide steel body (13 mm hex,
  2.6 cm tall) with the steel thumb lever on its left-facing flat (6 mm wide, 2 cm long, sprung 2.5 mm out: 3 N starts
  it in, 10.5 N presses it fully in), and a brass swivel with the hose's side outlet. You press the thumb lever and
  push the coupler onto the nipple; let go of the lever and the jaws lock behind the ball head; it then stays on with
  no hand on it while the gun is worked, and comes off when you press the lever again and pull. Locked on, the ball head
  is fully inside the mouth and the seal's edge sits on the ball about 1 mm below its top; the nose's face is then 1 mm
  above the hex.
- **Nipple (modelled):** DIN 71412 form A, M10x1 taper thread 5.5 mm long, 11 mm hex 3.5 mm tall, 4 mm neck, 6.5 mm
  ball head, 16 mm long, its top 10.5 mm above the lug (within the bought nipples above: 15-16 mm long, 11 mm hex).
- **Why the seat tolerances:** the closed jaws leave 0.35 mm round the ball, so a coupler with the ball in its mouth is
  at most about 0.4 mm off the nipple's axis (the scorer allows 1.5 mm for the simulation's contact give). The seal ring
  sits on the ball like a socket, so it keeps sealing as the coupler rocks, until the nose's face meets the hex's
  corners at about 10 degrees and the seal lifts off the far side of the ball; the scorer allows 8 degrees.

### Forces, all within a YAM arm

- Pushing the coupler on until its jaws lock: the demo's arm leans about 12 N on it.
- Pressing the thumb lever in: 10.5 N at most, well within a YAM finger's roughly 40 N.
- Lifting the gun's handle: about 7 N at the grip (4.9 N of damper plus its 1.9 N weight), carried by the friction of
  the two fingers' squeeze on its rubber grip.
- Working the gun's handle: about 4.9 N at the grip point at the demo's speed, plus or minus the handle's 1.9 N weight;
  a real rig is held to at most 15 N (above).
- Holding the coupler on while pumping: none. The line's pressure pushes on the coupler's 4.6 mm seal (16.6 mm²): at
  12.5 bar that is 21 N, which the locked jaws take (the G-Lock datasheet above: uncoupling over 600 bar is possible).
  The simulation pushes the coupler off with that 21 N during every downstroke.
- The YAM: joints 1-3 are DM4340 motors (9 N m rated, 27 N m peak), joints 4-6 DM4310 motors (3 N m rated, 7 N m
  peak); the gripper is limited to 50 N by its SDK; the rated payload is 2 kg. By the arm's kinematics, 15 N straight
  down at the nipple loads the right arm's joints 2 and 3 with 5.3 and 3.7 N m, and 15 N pushing the gun's handle down
  at its grip loads the left arm's joints 2 and 3 with 5.0 and 3.8 N m (with the handle down; 4.3 and 3.4 N m, and
  2.1 N m on joint 1, with it lifted 25 degrees), all within their ratings; the wrist takes almost none of it.
- Sources: YAM motor types and the 50 N gripper limit from the arm's SDK documentation
  (https://internaldoc.pages.dev/products/yam) and the arm's gain and limit files (27 and 7 N m, 50 N); motor ratings
  from the DM-J4340 and DM-J4310 data; 2 kg nominal payload from a retail listing
  (https://www.roboticscenter.ai/store/product/i2rt-yam-6dof). Lock-on couplers: the G-Lock datasheet above and a
  locking coupler rated to 10,000 psi whose thumb lever releases it, used "with both hands free"
  (https://vehicleservicepros.com/distribution/tool-distributor/article/20851517/in-focus-locknlube-grease-coupler).

### Still to measure on the bought parts

The gun's maker does not publish its handle's geometry. The model's handle (30 cm from pivot to end, the rubber grip on
its last 11 cm, 26 x 20 mm, 0.30 kg, resting level 1.7 cm above the barrel, opening 60 degrees at most) and the head's
size are our estimates, as is the grip's 24.5 cm from the pivot that turns the published 9 in. stroke into degrees.
They should be measured on a bought gun before a real trial; if they differ, the stroke thresholds (in degrees) and the
handle's position in the model are updated to match. The spring-scale check above settles the handle force on the real
rig.

## Real-trial safety and rig protection

Only three places carry grease: the nipple, the coupler's mouth and nose, and the ends of the pin's bore. What the
robot holds (the coupler's hex body and thumb lever, the gun handle's rubber grip) is kept clean.

- **Pad covers.** Before a session, each finger pad of both grippers gets a disposable cover: a fingertip cut from a
  black textured 6-mil nitrile glove, pulled over the pad and taped at the finger's root with vinyl electrical tape.
  Check that a covered gripper still lifts the coupler by its hex body without slipping. Change the covers after any
  leak, whenever grease shows on them, and at the end of the session.
- **Drip trays.** A shallow tray lined with a shop towel under the pivot's base plate, and another under the holder cup
  and the gun's outlet; the hose loop lies on a towel.
- **Before each trial.** Wipe the coupler (nose, mouth face, hex body and thumb lever), the nipple and both ends of the
  pin's bore with a shop towel, then with isopropyl alcohol (70-99 %) on a lint-free wipe, and let it dry. Check the
  covers with a white wipe.
- **During a trial.** The operator keeps a hand on the stop button. Grease leaking at the coupler or the nipple ends the
  trial (it is already scored as a leak).
- **If the coupler leaks.** Stop the robot and send both arms home. Drape a shop towel over the coupler and the nipple.
  By hand (nitrile gloves), press the thumb lever, take the coupler off, wipe it and stand it in the holder; wipe the
  nipple and the lug. Peel off the robot's pad covers into a bag; wipe anything under them with isopropyl alcohol and
  check with a white wipe; fit new covers. Clean the rig parts and the trays with a citrus degreaser or isopropyl
  alcohol. Never spray solvent near the robot, and keep grease off the wrist cameras.
- **No stand-in grease.** The grease is what the trial scores: its back-pressure sets the handle force and its leak or
  purge is the outcome, so real grease is used and the covers, trays and wipes keep it off the robot.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The pivot, the gun with its cradle and the holder cup
are kinematic, fixed where they are put; the gun's handle, the six hose segments, the coupler and its thumb lever are
loose rigid bodies made of shapes from `bodykit.py`.

- **Pivot and nipple:** the lug, the pin and the nipple as built above; the nipple is a hex (three boxes), a neck and a
  6.5 mm sphere. The pivot is greasy steel: slick, with friction 0.15.
- **Coupler's mouth:** real collision geometry made of 12-sided rings of boxes. They form the closed jaws' bore wall,
  the 45-degree lead-in and the seal ring, with a plug above the seal. Its outside is drawn as a round nose. The
  lead-in and the ball centre the coupler as it is pressed down, and only a coupler whose mouth has gone fully over the
  ball has its seal on the ball.
- **Thumb lever:** a 12 g steel pad on a sprung slide (a `Slider` from `bodykit.py`) on the hex body's left-facing
  flat: 2.5 mm of travel, 3 N to start it in and 10.5 N fully in. Fingers closing across the body press it in.
- **Jaws' lock:** a joint between the pivot and the coupler (a `Thread` from `bodykit.py`, used as a lock), switched
  off at the start. After every control step the task file checks the coupler: once it is sealed on the nipple (see
  below) with its thumb lever out (pressed in less than 0.8 mm), the lock switches on and holds the coupler on the
  nipple's axis with its seal 0 to 0.2 mm off the ball's top; it can turn on the ball only slowly, against drag (the
  brass swivel takes the hose's twist). While the thumb lever is pressed in more than 1.5 mm, the lock is off.
- **The gun's handle:** it turns on a hinge (a `Hinge` from `bodykit.py`) about a front-to-back axis on the gun's head,
  from 0 (resting level on its stop) up to 60 degrees. It has no spring, as on the real gun; a damper (0.075 N m per
  degree per second) stands in for the grease's resistance, both ways. Released while lifted, it sinks back under its
  own weight at about 7 degrees a second.
- **Hose:** six stiff 12 mm segments joined by ball joints (`Ball`), 0.45 m along them, from the gun's outlet to the
  coupler's side outlet.
- **Holder cup:** smooth steel (slick, friction 0.15), 4.6 cm across and 1.6 cm tall. A 45-degree funnel, 4 cm across
  at the rim, leads down to a 22 mm seat 3 mm deep, all made of 12-sided rings. A coupler set down within about 1 cm of
  its middle slides into the seat and stands upright. It counts as in its holder with its mouth within 6 mm of the
  middle and at most 6 mm above the floor, tilted less than 35 degrees.
- **Grease pressure:** lifting the handle draws grease into the gun. Once the handle has come down 3 degrees from the
  highest point it was lifted to, and until it is down within 1.5 degrees of its stop (or is lifted 3 degrees again),
  the plunger is pushing grease out. If the coupler's seal is then on the ball (its axis within 2 mm of the ball's
  centre, the seal within 1 mm of the ball's top, tilted at most 20 degrees), the task file pushes the coupler back off
  along its axis with 21 N. A locked coupler stays put (the lock takes it); one that is not locked comes off unless it
  is held on firmly. Check: with the coupler pushed on but held with its thumb lever pressed in (not locked) and only a
  ~2 N lean, the first downstroke pushed it 6-13 mm off the ball and latched a leak; the trial scored 0.25.
- **Sealed:** the ball's centre within 1.5 mm of the coupler's axis, the seal from 1 mm into the ball's top to 0.5 mm
  off it, and the coupler tilted at most 8 degrees.
- **Shots and leaks:** a stroke is the handle lifted past 20 degrees and pushed back down within 3 degrees of its stop.
  A stroke pushed down with the coupler sealed is a shot, and for each of the first four a drawn dab of grease (kept
  under the table until then) is moved to the gap between the lug and the arm's side plates. A leak is the plunger
  pushing grease at any moment while the coupler is not sealed. It is latched, and a fifth dab shows it: on the
  nipple's lug if the coupler is within 6 cm of the nipple, otherwise under the coupler's mouth.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Assets and licences

No third-party assets: every object in the scene is built in the task file from boxes, cylinders and spheres
(`bodykit.py`, part of this task). The robot, the table and the cameras are the `yam_arena` embodiment's own.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: full score, which includes both arms back
where they started, with the coupler, its thumb lever, the gun's handle and the hose still. Otherwise it gives `score()`
at the end of the episode: 0, 0.25, 0.5 or 1.0 (see `task.json`). A leak removes the greasing credit (0.5 and 1.0) for
the rest of the trial. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

In a real trial the same rules are judged from the video and two wipes: strokes and shots are counted as the handle's
grip passes the card's 20-degree line and comes back down onto its stop with the coupler on the nipple; sealed means
the coupler stays on by itself when let go, or its nose is seen down at the nipple's hex; a leak is grease seen
escaping at the coupler or the nipple, or grease on a white cloth wiped round the nipple's hex and the outside of the
coupler's nose afterwards (both wiped clean before the trial). After four shots, a white cloth wiped over both ends of
the bore shows fresh grease; if it does not, the rig is at fault and the trial is run again.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grease the loader's pivot pin through its grease nipple with the lever grease gun, without leaking any grease at the coupler or the nipple. Leave the coupler back in its holder cup and both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grease-dream
```

The scripted demo reaches full score:

1. The right hand comes down round the coupler's hex body in its holder, its fingers across the thumb lever (which
   presses it in and opens the jaws), takes it, lifts it out and carries it over the nipple.
2. It lowers the coupler over the ball head and pushes it fully on, aiming 3 cm below the seat so that the soft arm
   leans about 12 N on it. It eases off to a light lean and lets go: the thumb lever springs out and the jaws lock
   behind the ball. The hand rises 6 cm clear and waits; the coupler stays on by itself.
3. The left hand comes down round the gun handle's rubber grip and closes on it. It works the handle five strokes:
   it lifts it about 25 degrees, turning its wrist with it, and pushes it back down onto its stop. Then it lets go, rises and
   goes back to where it started.
4. The right hand comes back down round the coupler and closes across the thumb lever, which opens the jaws. It pulls
   the coupler off the nipple, carries it back, steadies it just above the holder cup, lowers it in and presses it into
   the seat (aiming 6 mm below the floor, so the nose slides down the funnel even if the hose lies under it). Then it
   eases off, lets go and goes back to where it started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grease the loader's pivot pin through its grease nipple with the lever grease gun, without leaking any grease at the coupler or the nipple. Leave the coupler back in its holder cup and both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grease-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/grease-dream/*.json; do inspect-robots video "$log"; done
```

Set `GREASE_DEBUG=1` to print what the score is built from every quarter second (every step while the gun pushes
grease with the coupler near the ball):

- how far the gun's handle is lifted, and whether the plunger is pushing grease;
- the strokes and shots so far, and whether a leak has happened;
- where the coupler is: its offset from the ball's centre, its seal's lift off the ball, and its tilt;
- whether it is sealed, on the ball or in its holder;
- how far its thumb lever is pressed in, whether its jaws are locked, and how far it has turned about its axis;
- where each TCP is.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (520 s on the clock, with Dream answering in 0.67 s on average), and about 7 minutes for the scripted demo (877 steps, 43.9 s of simulated time, 446 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-6FHXSD. Its review said:

> Clarity and reproducibility (not met): The custom grease-fed pivot and pump lack rebuildable drawings or identified purchasable parts for the specified grease path and pump performance.
>
> Feasibility (not met): Actual grease can contaminate the shared grippers during coupler handling or leakage, and no pad protection or safe alternative is specified.
>
> What to change: The earlier force and seal concerns are addressed by the locking coupler and revised checks. Two new real-trial issues were not raised before: provide dimensioned drawings or a purchasable parts list for the grease-fed pivot and a source or build specification for a pump meeting the stated stroke and force; also specify gripper protection and grease handling that prevent contamination of the shared rig, including if the coupler leaks.

The review before that (TK-88UN9H) asked for a real coupler holding force within the robot's capability, the
corresponding push-off force in the simulation, and a full-score demo; that was done in the previous revision (the
lock-on coupler) and is kept.

The review before that (TK-RJSVE3) asked for the real coupler and nipple sealing geometry, greasing credit that needs
grease to reach the pin with no leak at the coupler, and a simulated seat tightened to match the seal; that was done
two revisions ago and is kept.

What changed in response:

- **Clarity and reproducibility (not met), "dimensioned drawings or a purchasable parts list for the grease-fed
  pivot".** The pivot is now one machined block plus bought and standard parts, with a dimensioned drawing and assembly
  notes ("The real rig" above). The lug is a 50 x 40 mm steel flat bar cut 56 mm long, with a 20.5 mm bore for the
  pin 20 mm above its foot, an M10x1 hole 9 mm deep in its top for the nipple and a 5 mm passage down to the bore; the
  pin is an ISO 2340 B 20 x 100 clevis pin with ISO 8738 washers and 5 mm split pins; the nipple is a KIPP
  K1132.1110100 (DIN 71412 style A, M10X1, 11 mm hex, at most 16 mm long; datasheet linked). The grease path holds
  about 1.2 cm³ (0.64 + 0.33 + 0.22 cm³, worked out from the drawing). The simulation was changed to match: the lug is
  5.6 cm tall with the nipple screwed straight into its top (the separate 6 mm boss is gone; the nipple stands where it
  did), and the pin turns in the drilled bore instead of a bushing.
- **"A source or build specification for a pump meeting the stated stroke and force".** Our invented pail pump is
  replaced by a bought pump: a Lincoln 1145 lever grease gun in its high-pressure setting, with a Lincoln 71518 18 in.
  whip hose, lying in a plywood cradle. Lincoln's published figures: maximum 7,000 psi, strokes up to 9 in. at the
  grip, 1 oz of grease per 33 full strokes. Derived: about 0.95 cm³ per full stroke, 0.018 cm³ per degree of handle
  swing; a counted stroke (the handle lifted past 20 degrees and pushed back within 3 degrees of its stop) gives at
  least 0.30 cm³, the demo's 25-degree strokes 0.45 cm³. The force follows from the work: the torque on the handle is
  the line pressure times 1.02 cm³ per radian. The simulation's damper takes 4.9 N at the robot's grip point at
  17 degrees a second, which is 12.5 bar, so the coupler is now pushed off with 21 N (it was 8 N) on every downstroke.
  Re-tested: a coupler pushed on but held with its thumb lever pressed (not locked) and a ~2 N lean was pushed 6-13 mm
  off on the first downstroke and latched a leak (score 0.25).
  A real rig is checked with a spring scale: at most 15 N at the grip (at most 36 bar). The simulation was changed to
  match the gun: the pail is gone; the gun lies in its cradle; its handle, which has no return spring on the real gun,
  rests on its stop and the left hand grips it, lifts it and pushes it back down; the hose is 18 in. long, in six
  segments; the stroke and leak rules now follow the handle's lift and push. The gun's handle geometry is not
  published, so the model's handle (30 cm long, grip on its last 11 cm) is our estimate and is listed under "Still to
  measure".
- **Feasibility (not met), "gripper protection and grease handling that prevent contamination of the shared rig,
  including if the coupler leaks".** New section "Real-trial safety and rig protection": disposable nitrile covers on
  every finger pad of both grippers, changed after any leak; drip trays under the pivot, the holder and the gun's
  outlet; everything the robot holds wiped with isopropyl alcohol before each trial; a leak procedure (stop, arms home,
  cover the coupler and nipple with a towel, take the coupler off by hand, bag the covers, clean with isopropyl alcohol
  or a citrus degreaser, check with a white wipe, new covers); white grease so that smears show. Real grease is kept,
  with the reason: its back-pressure and its leak or purge are what the trial measures.
- **Rig protection is part of the task itself:** the setup now states the pad covers and drip trays for real trials,
  and the reset notes the wipe-down.
- **Real-trial scoring stays observable:** strokes and shots are counted in the video against a 20-degree line on a
  card behind the handle; sealed, leak and greased are judged by the coupler staying on, grease seen escaping, and white
  wipes of the nipple, the coupler's nose and the bore's ends. The simulation counts the same strokes in degrees.
- **The instruction** now names the lever grease gun instead of the pail pump; nothing else in it changed. The context
  now counts the workers who grease machine pivots regularly (O*NET: mobile heavy equipment and industrial machinery
  mechanics, about 627,000).
- **A full-score demo and the model run on the revised task:** the new demo scores 1.0 (877 steps of the 1,200
  allowed, ending on success). The left hand lifts the handle to about 24 degrees and pushes it back onto its stop five
  times; the coupler stays locked on with no hand on it under the 21 N push (in the debug run its seal stayed 0.15-0.20
  mm off the ball, tilt 0 degrees): five shots, no leak. The Dream-1 run on the revised task scores 0.0 (1,200 steps).
- **Kept from the previous revisions:** the lock-on coupler and its thumb lever, the DIN 71412 nipple, the coupler's
  mouth and seal geometry, the sealed-seat tolerances (1.5 mm, 0.5 mm, 8 degrees), the four sealed strokes needed for
  greasing credit and the latched leak rule.

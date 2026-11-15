# Grease a loader's pivot pin from a pail pump without leaking grease at the nipple

On the table stands the pivot of a loader arm: a yellow steel lug with a steel pin through it and, on its top, a
straight grease nipple (DIN 71412 form A: M10x1 taper thread, 11 mm hex, 6.5 mm ball head). To the left stands a
grease pail with a lever pump on its lid; its black hose runs over the table to a lock-on grease coupler standing in a
steel holder cup. The job is to grease the pin through its nipple without leaking any grease at the coupler or the
nipple, and to finish with the coupler back in its holder and both arms back where they started. The task is one task
file, [`grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py`](grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: pushing a coupler squarely over a small ball-head fitting until its jaws lock; working a pump's lever
through repeated full strokes only while the coupler is sealed on the nipple (locked, or held), so that no grease
leaks; pressing the coupler's thumb lever to take it off; putting it back in its holder.

Two arms: in the demo the right arm puts the coupler on, later takes it off and puts it back, and the left arm works
the pump's lever, which stands on the pail 29 cm from the nipple. A locked coupler needs no hand on it, so the arms
take turns and neither has to hold anything for the other. The task does not force two arms: one arm can reach both
the nipple and the lever (by the arm's inverse kinematics, the right reaches the lever's grip pad with its grip leaned
20-40 degrees), so a policy may also do the whole job with one arm. The earlier version's reason for two hands
(holding a jawless coupler on against the grease pressure while the other hand pumps) does not apply to a lock-on
coupler and has been dropped.

- **One-arm check.** The right arm does its part as in the demo; the left never moves. The trial scored 0.25: the
  coupler was locked on and taken off again, but nothing was pumped. This shows that the pumping is needed, not that
  two arms are.

## The real parts

- **Nipple:** DIN 71412 form A (straight), yellow-zinc-plated steel, M10x1 taper thread (5.5 mm long, in the boss),
  11 mm hex 3.5 mm tall, 4 mm neck, 6.5 mm ball head; 16 mm long, its top 10.5 mm above the boss.
- **Coupler:** a lock-on grease coupler for ball-head nipples, the kind sold as a locking or hands-free coupler: four
  hardened steel jaws, held closed by a sliding sleeve, and a thumb lever that pulls the sleeve back. You press the
  thumb lever and push the coupler onto the nipple; let go of the lever and the jaws lock behind the ball head; it
  then stays on with no hand on it while you pump, and comes off when you press the lever again and pull. Commercial
  ones hold at 600 bar and more (one is rated to 10,000 psi, about 690 bar; see the sources below), far above what
  this pump makes. Ours (82 g with its thumb lever, 4.8 cm tall): a zinc-plated hardened steel nose 18 mm across and
  12 mm tall, whose closed jaws form a 7.2 mm mouth with a 45-degree lead-in (10.4 mm across at the face), running 5
  mm up to a nitrile seal ring 2 mm thick with a 4.6 mm hole. Above the nose sit a black-oxide steel body (13 mm hex,
  2.6 cm tall) with the steel thumb lever on its left-facing flat (6 mm wide, 2 cm long, sprung 2.5 mm out: 3 N starts
  it in, 10.5 N presses it fully in), and a brass swivel with the hose's side outlet. Locked on, the ball head is
  fully inside the mouth and the seal's edge sits on the ball about 1 mm below its top; the nose's face is then 1 mm
  above the hex.
- **Why the seat tolerances:** the closed jaws leave 0.35 mm round the ball, so a coupler with the ball in its mouth is
  at most about 0.4 mm off the nipple's axis (the scorer allows 1.5 mm for the simulation's contact give). The seal ring
  sits on the ball like a socket, so it keeps sealing as the coupler rocks, until the nose's face meets the hex's
  corners at about 10 degrees and the seal lifts off the far side of the ball; the scorer allows 8 degrees.
- **Forces, all within a YAM arm:**
  - Pushing the coupler on until its jaws lock: the demo's arm leans about 12 N on it.
  - Pressing the thumb lever in: 10.5 N at most; the gripper closes across it with up to 50 N.
  - Working the pump's lever: pressing its grip pad down takes about 8 N at 18 degrees and 11 N at 26 degrees (its
    return spring and the grease's resistance together). A real rig uses a pin that turns freely and is checked with a
    spring scale hooked on the grip pad: a full stroke into the nipple, at about one stroke a second, takes at most
    15 N.
  - Holding the coupler on while pumping: none. The grease pressure pushes on the coupler's 4.6 mm seal (16.6 mm²),
    and the locked jaws take that push. The pump's lever caps the pressure: at most 15 N on the pad, less the spring's
    ~4 N, over 4.4 cm of pad travel per 1 cm³ of grease is at most ~4.8 bar, which pushes on the seal with ~8 N. The
    simulation uses that 8 N.
  - The YAM: joints 1-3 are DM4340 motors (9 N m rated, 27 N m peak), joints 4-6 DM4310 motors (3 N m rated, 7 N m
    peak); the gripper is limited to 50 N by its SDK; the rated payload is 2 kg. Pressing 15 N straight down at the
    nipple loads joints 2 and 3 with 5.3 and 3.8 N m (from the arm's kinematics), within their rating; the wrist takes
    almost none of it.
- **How much grease:** the pin's drilled passage and its bushing's groove hold about 1.5 cm³, and the lever pump gives
  about 1 cm³ per full stroke. Four strokes (about 4 cm³) fill them and purge fresh grease at both ends of the bushing.
- **Sources:** lock-on couplers: a four-jaw hydraulic safety coupler's data sheet ("press the lever with the thumb and
  insert", "the hardened 4-jaws maintain the coupler firmly inserted", "free hands", "uncoupling over 600 bar is
  possible": https://media-new.imu.nl/storage/lubretec.com/33142/lpi-g-lock-en.pdf); a locking coupler rated to
  10,000 psi whose thumb lever releases it, used "with both hands free"
  (https://vehicleservicepros.com/distribution/tool-distributor/article/20851517/in-focus-locknlube-grease-coupler).
  YAM: motor types and the 50 N gripper limit from the arm's SDK documentation
  (https://internaldoc.pages.dev/products/yam) and the arm's gain and limit files (27 and 7 N m, 50 N); motor ratings
  from the DM-J4340 and DM-J4310 data; 2 kg nominal payload from a retail listing
  (https://www.roboticscenter.ai/store/product/i2rt-yam-6dof).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The pivot, the pail with its pump and the holder
cup are kinematic, fixed where they are put; the lever, the five hose segments, the coupler and its thumb lever are
loose rigid bodies made of shapes from `bodykit.py`.

- **Nipple:** a hex (three boxes), a neck and a 6.5 mm sphere, kinematic. The pivot is greasy steel: slick, with
  friction 0.15.
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
- **Lever:** it turns on a sprung hinge (a `Hinge` from `bodykit.py`) about a side-to-side axis on the pump head. It
  presses down 26 degrees at most. Its spring (0.055 N m per degree) holds it 15 degrees up and stands in for the
  return spring and the grease's resistance together: about 8 N at the grip pad 18 degrees down, 11 N at 26 degrees.
- **Hose:** five stiff 12 mm segments joined by ball joints (`Ball`), from the pail's outlet to the coupler's side
  outlet.
- **Holder cup:** smooth steel (slick, friction 0.15), 4.6 cm across and 1.6 cm tall. A 45-degree funnel, 4 cm across
  at the rim, leads down to a 22 mm seat 3 mm deep, all made of 12-sided rings. A coupler set down within about 1 cm of
  its middle slides into the seat and stands upright. It counts as in its holder with its mouth within 6 mm of the
  middle and at most 6 mm above the floor, tilted less than 35 degrees.
- **Grease pressure:** while the lever is pressed more than 6 degrees from rest, the pump is pushing grease. If the
  coupler's seal is then on the ball (its axis within 2 mm of the ball's centre, the seal within 1 mm of the ball's top,
  tilted at most 20 degrees), the task file pushes the coupler back off along its axis with 8 N. A locked coupler stays
  put (the lock takes it); one that is not locked comes off unless it is held on firmly. Check: with the coupler pushed
  on but held with its thumb lever pressed in (not locked) and only a ~2 N lean, the first stroke pushed it 2-4 mm off
  the ball and latched a leak; the trial scored 0.25.
- **Sealed:** the ball's centre within 1.5 mm of the coupler's axis, the seal from 1 mm into the ball's top to 0.5 mm
  off it, and the coupler tilted at most 8 degrees.
- **Shots and leaks:** a stroke is the lever pressed down past 18 degrees; it has to come back to within 10 degrees of
  rest before the next press counts. A stroke reaching 18 degrees with the coupler sealed is a shot, and for each of
  the first four a drawn dab of grease (kept under the table until then) is moved to the gap between the lug and the
  arm's side plates. A leak is the lever pressed past 6 degrees at any moment while the coupler is not sealed. It is
  latched, and a fifth dab shows it: on the nipple's boss if the coupler is within 6 cm of the nipple, otherwise under
  the coupler's mouth.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: full score, which includes both arms back
where they started, with the coupler, its thumb lever, the pump's lever and the hose still. Otherwise it gives `score()` at the end of the
episode: 0, 0.25, 0.5 or 1.0 (see `task.json`). A leak removes the greasing credit (0.5 and 1.0) for the rest of the
trial. `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grease the loader's pivot pin through its grease nipple from the pail pump, without leaking any grease at the coupler or the nipple. Leave the coupler back in its holder cup and both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grease-dream
```

The scripted demo reaches full score:

1. The right hand comes down round the coupler's hex body in its holder, its fingers across the thumb lever (which
   presses it in and opens the jaws), takes it, lifts it out and carries it over the nipple.
2. It lowers the coupler over the ball head and pushes it fully on, aiming 3 cm below the seat so that the soft arm
   leans about 12 N on it. It eases off to a light lean and lets go: the thumb lever springs out and the jaws lock
   behind the ball. The hand rises 6 cm clear and waits; the coupler stays on by itself.
3. The left hand comes down on the lever's grip pad, fingers closed, and pumps the lever down five full strokes,
   following its swing. Then it rises and goes back to where it started.
4. The right hand comes back down round the coupler and closes across the thumb lever, which opens the jaws. It pulls
   the coupler off the nipple, carries it back, steadies it just above the holder cup, lowers it in and presses it into
   the seat (aiming 6 mm below the floor, so the nose slides down the funnel even if the hose lies under it). Then it
   eases off, lets go and goes back to where it started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Grease the loader's pivot pin through its grease nipple from the pail pump, without leaking any grease at the coupler or the nipple. Leave the coupler back in its holder cup and both arms back where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grease-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/grease-dream/*.json; do inspect-robots video "$log"; done
```

Set `GREASE_DEBUG=1` to print what the score is built from every quarter second (every step while the lever is pressed
with the coupler near the ball):

- how far the lever is pressed down;
- the strokes and shots so far, and whether a leak has happened;
- where the coupler is: its offset from the ball's centre, its seal's lift off the ball, and its tilt;
- whether it is sealed, on the ball or in its holder;
- how far its thumb lever is pressed in, whether its jaws are locked, and how far it has turned about its axis;
- where each TCP is.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 1,200 steps (294 s on the clock, with Dream answering in 0.81 s on average), and about 5 minutes for the scripted demo (870 steps, 43.5 s of simulated time, 315 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-88UN9H. Its review said:

> Feasibility (not met): The submission states that the real coupler needs 50–100 N of holding force, beyond the YAM's approximately 20 N capacity, while the demo uses 15 N.
>
> What to change: The real coupler is specified to require 50–100 N to hold on, while the YAM is described as capable of about 20 N; the 15 N simulation demo therefore does not establish feasibility for the real task. Specify and simulate a two-armed embodiment capable of the required force, use the corresponding realistic push-off force, and provide a full-score demo and the required model run for that embodiment.

The review before that (TK-RJSVE3) asked for the real coupler and nipple sealing geometry, greasing credit that needs grease to reach the pin's bushing with no leak at the coupler, and a simulated seat tightened to match the seal; that was done in the previous revision and is kept.

What changed in response:

- **Feasibility (not met): the stated 50-100 N hold-on force was beyond the YAM.** The benchmark fixes the embodiment (two YAM arms), so we could not specify a stronger robot. Instead the real-world rig was brought within the YAM's capability, and the simulation now uses the same forces. The 50-100 N was our own estimate for a jawless push-type coupler held on by hand, from an assumed, unsourced 30-60 bar on its 4.6 mm seal. Our own pump could not even reach that pressure with a light press: 30-60 bar at 1 cm³ per stroke needs 70-140 N on its lever.
- **"An embodiment capable of the required force": every force the job needs is now within the YAM.** The coupler is now a lock-on coupler, the kind with four hardened steel jaws and a thumb lever. You press the lever and push the coupler on; let go and the jaws lock behind the ball head. It then stays on with no hand on it (such couplers are sold for hands-free greasing and hold at 600 bar and more; one is rated to 10,000 psi), and comes off when the lever is pressed again. So no arm has to hold the coupler on while pumping. The forces are:
  - about 12 N to push the coupler on (the demo's lean);
  - at most 10.5 N to press the thumb lever in (the gripper squeezes up to 50 N);
  - about 8 N at 18 degrees and 11 N at 26 degrees on the pump's lever (a real rig is checked with a spring scale to take at most 15 N).
- **The YAM's capability, sourced and corrected.** Its joints 1-3 are DM4340 motors (9 N m rated, 27 N m peak), joints 4-6 DM4310 motors (3 N m rated, 7 N m peak); its gripper is limited to 50 N by its SDK; its rated payload is 2 kg. Pressing 15 N down at the nipple loads joints 2 and 3 with 5.3 and 3.8 N m. Our earlier "about 20 N" was that 2 kg payload misread as a pushing limit; it is gone, and no statement that the YAM can't give a needed force remains.
- **"The corresponding realistic push-off force".** The pump's lever caps the line pressure. At most 15 N on the pad, less a return spring's ~4 N, over 4.4 cm of pad travel per 1 cm³ is at most ~4.8 bar, which pushes on the seal's 16.6 mm² with ~8 N. The simulation now pushes the coupler off with 8 N during every stroke (it was 15 N), and the coupler's jaws take it. Their lock is a joint that switches on when the coupler is sealed on the nipple with its thumb lever out, and off while the lever (a sprung pad with 2.5 mm of travel) is pressed in. A coupler that is not locked and not held on firmly is pushed off, and the stroke latches a leak: tested with the coupler pushed on but held with its thumb lever pressed in (not locked) and a ~2 N lean, the first stroke pushed it 2-4 mm off the ball and latched a leak (score 0.25). The pump's lever spring was stiffened to the 8-11 N above (it was 3-4 N).
- **Two hands.** A locked coupler needs no hand, so the earlier reason for two hands (holding the coupler on while the other hand pumps) no longer holds and has been dropped. In the demo the right arm puts the coupler on and takes it off, and the left pumps; one arm could also do the whole job, and the text now says so. The one-arm check (right as in the demo, left idle) scores 0.25: locked on, never pumped.
- **"A full-score demo and the required model run".** The new demo scores 1.0 (870 steps of the 1,200 allowed, ending on success). The coupler stays locked on the nipple with no hand on it through all five strokes (in the debug runs its seal stayed 0.15-0.2 mm off the ball, tilt 0 degrees): five shots, no leak. The Dream-1 run on the revised task scores 0.0 (1,200 steps).
- **Kept from the previous revision:** the DIN 71412 nipple, the coupler's mouth and seal geometry, the sealed-seat tolerances (1.5 mm, 0.5 mm, 8 degrees), the four sealed strokes needed for greasing credit, the latched leak rule and the real-trial checks.

# Look into a training manikin's left ear canal with a pocket otoscope

An adult ear-examination training manikin lies on its right side on a gel head ring, its left ear up. A pocket LED
otoscope (Welch Allyn Pocket Plus LED 22880 type), switched on, rests nose-down on its stand, and three 4.25 mm adult
disposable specula stand tip-down in a holder. The maneuver is familiar from routine ear exams and is practiced here
on a training manikin. Fit a speculum to the scope (push it on and twist it clockwise until it locks), draw the
outer ear up and back to straighten and open the canal, slide the speculum gently into the canal's outer part along
its axis, look, draw it out, let the ear go and put the scope back. The task is one task file,
[`look_into_the_manikins_left_ear_canal_with_the_otoscope.py`](look_into_the_manikins_left_ear_canal_with_the_otoscope.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

The instruction gives the goal only ("Look into the manikin's left ear canal with the otoscope, as you would a
patient's, then put the otoscope back on its stand and finish with both arms back where they started."). It says
nothing about the speculum, the ear or the depth.

## What it tests

- **A twist-on fit.** The scope's 15 mm tip cone takes a speculum only when pushed 6 to 16 mm into one standing in
  the holder and then turned 20 degrees clockwise (seen from the eyepiece) without lifting out: the hand has to
  swing the scope about the speculum's axis, 10 cm from where it holds the handle.
- **Opening the canal with the other hand.** The canal's outer third is cartilage. At rest its back-upper wall stands
  across the opening and leaves a 4 mm slit, narrower than the speculum's 4.25 mm tip; the speculum can't pass
  without forcing that wall. Drawing the helix up and back (world +y here) slides the wall 7 mm, to flush with the
  canal: the opening is then 11 mm. The ear is sprung (3.6 N at rest, 5.7 N fully drawn) and springs shut as soon as
  it is let go.
- **Controlled insertion.** With the canal open, the speculum goes in along the canal's axis to the trainer's
  6-13 mm scoring range and stays there for a 1 s look. A resultant tool-to-head contact force above 3 N for
  0.1 s or an insertion beyond 13 mm fails the trial. These are benchmark thresholds for the trainer.
- **Finishing the job.** Draw the speculum out before letting the ear go (the wall springs back onto a speculum left
  in), return the scope to its stand, arms home.

## Why two hands

The physics needs two hands at once at the ear: one keeps the outer ear drawn up and back against its spring while
the other inserts the speculum 3-4 cm away. Pushing the speculum through the closed slit means wedging the canal wall
7 mm aside against its 3.6-5.7 N spring: that is a wall force above the 3 N limit, and the trial fails. Holding the
ear open needs a hand on the ear for the whole insertion; the scope needs the other hand. The one-hand runs bear it
out. The right arm alone, doing exactly its part of the demo with the left at home, fitted and locked the speculum, but at the ear the speculum's tip stopped on the closed
canal wall 2.5 mm above the opening, pressing it with up to 2.9 N, and never went in (score 0.25: speculum locked).
The left arm alone drew the ear open and held it, but nothing went in (0.0). Letting the ear go with the speculum
still in springs the wall onto the speculum and trips the contact-force limit (0.0).

In the demo the left hand pinches the helix rim's back-upper part (the clinical grip) and draws it up and back to
its stop; the right hand holds the scope by its handle 10 cm from the head's axis, leaning in from the robot's side,
and brings the speculum down the canal's axis. The wrists stay about 20 cm apart throughout.

Unlike seating an ear thermometer's probe at the canal's mouth, the look needs the ear actively held open by a second
hand for the whole insertion, a speculum fitted by a twist lock first, and the speculum's depth kept inside a 7 mm
window with a force limit.

## Real-world basis

- **Societal relatability.** A pocket otoscope and disposable speculum are recognizable instruments from routine ear
  exams. The task reproduces the visible coordination of the examination on a training model. See the [manufacturer's
  Pocket LED instructions](https://www.hillrom.com/content/dam/hillrom-aem/us/en/sap-documents/LIT/80029/80029234LITPDF.pdf).
- **The otoscope.** Welch Allyn Pocket Plus LED, models 22880-BLK/-WHT/-PUR/-BLU (Welch Allyn spec sheet SM4121GB):
  total height 16.8 cm, head width 3.0 cm, head depth 5.1 cm, handle diameter 2.1 cm, head and handle 71.7 g without
  batteries, two AA alkaline cells. Its instructions for use: "To turn instruments on and off, and vary intensity,
  rotate switch at base of instrument head" (the scope starts switched on here).
- **The speculum.** Welch Allyn KleenSpec disposable ear specula, 4.25 mm adult (52434-U) and 2.75 mm paediatric
  (52432-U), listed for the PocketScope. The Welch Allyn otoscope instructions for these specula: "To attach the
  speculum onto the otoscope, twist the speculum clockwise into the TipGrip until it locks."
- **The manikin.** The physical build uses the **Nasco Life/form LF01021 A normal left-ear insert**, available
  on its own for £95 at the listed UK retailer. It has a molded flexible pinna and canal and a full-color membrane
  image. The manufacturer's photo shows the trainer's left ear being held retracted while an otoscope is inserted.
  The manual shows the normal insert and its mounting plug on p. 3. [Manufacturer](https://shop.nascohealthcare.com/products/lf01019) ·
  [Manufacturer's manual](https://www.mentone-educational.com.au/assets/brochures/LF01019.pdf) ·
  [Insert listing](https://www.anatomystuff.co.uk/products/normal-tympanic-membrane).

## Scene (as in the screenshot; the same in every trial)

A bench in front of two arms (bases 61 cm apart), seen by one overhead camera and a camera on each wrist. Distances are measured forward from the line between the arm bases and sideways from the midpoint between them.

1. A simulated adult ear-examination training manikin head with a flexible-ear mechanism, lying on its right side on a blue gel head ring on a white sheet, left ear up, face toward the far right, crown toward the far left. Fixed. Its left ear canal opens 30 cm forward, 4 cm left and 14.5 cm above the bench and runs straight down into the head. The outer 8 mm section is 11 mm across when open. The lower section is 7 mm across and reaches a representation of the eardrum 25 mm in.
2. The left outer ear (silicone, 20 g) together with the outer canal's back-upper wall: they slide up and back (toward the robot's left) as one, up to 7 mm, against a spring of 3.6 N at rest rising to 5.7 N. At rest that wall leaves the canal's opening a 4 mm slit; drawn fully up and back, the opening is 11 mm. The helix rim's back-upper part (24 mm long, 10 mm thick, rising 15 mm from the concha and leaning out 20 degrees) is 3.2 cm from the canal's centre.
3. A Welch Allyn Pocket Plus LED otoscope (22880), switched on: black ABS head 30 mm wide and 51 mm deep including its tip cone (15 mm long, 8 mm across at the end, 14 mm at the base); 21 mm handle, 138 mm long, with two AA cells; 118 g. It rests on its stand, nose down: head 25.5 cm forward and 14 cm right, handle level 6.5 cm above the bench, running toward the robot's right. The stand: a grey ABS base 5 x 18.5 cm with two V-rests under the handle and two cheeks beside the tip cone.
4. A white ABS specula holder block 70 x 30 x 60 mm, centred 17.5 cm forward and 11 cm right, with three 15 mm square pockets 2.5 cm apart along its length. In each hangs, tip down by its flange, a black polypropylene Welch Allyn KleenSpec 4.25 mm adult disposable speculum (52434-U): a 27 mm cone from a 13 mm rim with a 16 mm flange to a 4.25 mm tip. A speculum goes on by pushing the scope's tip cone into it and turning the scope clockwise (seen from the eyepiece) until it locks: 20 degrees.

Both arms start at their home pose. The layout is the same in every trial, as in the screenshot.

### Physical trainer setup

1. Use the normal LF01021 A plug-in left ear with its embedded membrane picture. The insert is the only proprietary
   training-head part required. A foam display head, a 100 × 100 × 10 mm plywood or printed PLA mounting plate,
   two small backing tabs and a bench strap form the support.
2. Trace the mounting plug at its seating shoulder onto the rigid plate, cut or print a keyed aperture to that
   outline, and round the edges. Seat the plug by hand until its flange lies against the plate. Attach two removable
   tabs to the back of the plate to retain the plug at its support edges. Set the tabs to prevent withdrawal without
   squeezing the silicone canal. Mount the plate in a recess at the foam head's left-ear position, with clearance
   behind it for the canal and all around the exposed movable pinna. Strap the foam head and its rigid plate to the
   bench so the plate cannot rock during retraction.
3. Rest the head on its right side on the gel ring. Shim the fixture to put the left opening 30 cm forward, 4 cm left
   and 14.5 cm above the bench. Align the ear to the simulated face and crown directions, then match the stand and
   holder positions to the simulated setup. Mark the specula with narrow contrasting bands at 6 and 13 mm from the
   tip. Add a removable reference dot on the fixed support near the rim so it is visible in side video.
4. The manufacturer demonstrates two-handed use with its own head. The custom mounting plate, scoring depth and
   3 N limit still need physical verification. Measure the opening at rest and retracted and confirm the pinna
   returns to rest and the plug stays seated. With a mounted 4.25 mm speculum, check for a continuous 1 s insertion
   within the 6-13 mm scoring range under the contact limit, with the embedded image visible when the pinna is
   retracted. With the ear released, repeat the same guided approach below the contact limit and record whether
   the relaxed canal obstructs the insertion. Use a build that passes both checks.

Patel et al. reported a separate US$70 trainer build with a molded silicone left ear and smartphone membrane image.
Their supplement includes reports that a mounted ear detached when pulled and that some learners found the trainer
less representative of retraction. The archived canal mesh does not establish clearance with the pinna installed.
[Study](https://doi.org/10.12688/mep.19011.1) · [Fabrication and survey archive](https://doi.org/10.5281/zenodo.5932968).

### Independent contact measurement

Use a calibrated three-axis force sensor in the right wrist between the wrist flange and the gripper, with the
original fingers and a clear view of the ear from the wrist camera. Use the force channels of a wrist force/torque
sensor where available. An inexpensive open design with printable parts, firmware and calibration utilities is the
[Hamburg TAMS printed force/torque sensor](https://github.com/TAMS-Group/tams_printed_ft). The tool and gripper must
sit on the sensing side of the wrist. The left gripper and the head fixture attach to the bench separately.

Before trials, tare and compensate the gripper, scope and installed speculum's weight across the wrist orientations
used near the head. Use a line and hanging masses to check 2.0, 3.0 and 4.0 N pulls in each signed axis and check a
3.0 N diagonal pull. Verify error within 0.2 N and record at 100 Hz or faster. Run an unloaded approach to verify
the compensated reading stays within 0.2 N, and retract and release the ear with the left hand while holding the
scope nearby without contact to verify that those actions also leave the right wrist reading within 0.2 N. A hardware
build must pass these checks before use. Synchronize the readings with the video and evaluate tool-to-head contact
from the first approach to the final withdrawal. Use the magnitude of the compensated force vector, including
sideways loading, for the 3 N threshold.

## Scoring

The score latches at 0.0 if resultant tool-to-head contact force exceeds 3 N for 0.1 s, the speculum's tip goes more
than 13 mm into the canal, the scope's bare tip cone enters the canal, or the otoscope falls (its head's centre below
4.5 cm above the bench).

Otherwise: 1.0: a speculum locked on the scope, its tip kept 6 to 13 mm inside the canal for 1 s without a break at some point, and at the end the otoscope back on its stand (its head's centre within 15 mm of its start, nose down within 15 degrees, not held), the speculum out of the canal, the ear let go (back within 1 mm of rest) and both arms back where they started (each gripper's fingertip centre within 5 cm of its start); 0.75: the 1 s look done, the end state not met; 0.5: the speculum's tip 6 to 13 mm inside the canal at some point; 0.25: a speculum locked on the scope; else 0.0.

Locked: the speculum went on by the tip cone 6 to 16 mm into it (within 3.5 mm of its axis) and a clockwise turn of 20 degrees or more, seen from the eyepiece. Inside the canal: the tip within the canal's 11 mm outline, measured from the opening along the canal. The trial ends at the run limit, or once the full score has held 1 s with the otoscope and the ear still.

In a physical trial, use the synchronized force recording for the contact latch. Check the lock by whether the
speculum stays on when the scope is turned tip-down and given a light tap after the run. Use side and wrist video to
read the depth bands against the canal rim while the tip is within the opening. The 6 mm band reaches the rim for a
scored insertion and the 13 mm band stays at or above it. Time the uninterrupted 1 s interval from the same video.
Record bare-tip entry, a fall and the final state on video. Use start-position marks on the stand, ear and arms to
measure the end tolerances. The depth-and-hold rule is a reproducible proxy for the look. The physical setup's normal
membrane image provides the view and is checked during fixture setup.

The score is computed by `score()` in the task file, every control step; `success()` adds the stillness of the otoscope and the ear over 1 s. The demo uses fixed waypoints only.

## Checks

Every check is the scripted demo with one change (environment switches in the task file's `demo()`), run in the
simulator with the same seed as the demo.

| Run | What changes | Score | Why |
|---|---|---|---|
| Demo | (none) | 1.0 | speculum locked at 12 s; tip 7-7.5 mm in for 2 s; peak force 0.29 N; scope back on its stand |
| Demo, repeat | (none) | 1.0 | the same |
| Right arm alone | the left stays home (the ear is never drawn open) | 0.25 | the speculum stops on the closed canal wall 2.5 mm above the opening (2.9 N); speculum locked only |
| Left arm alone | the right stays home | 0.0 | the ear is held open, but no speculum is fitted or inserted |
| No speculum | the scope goes to the ear without fitting one | 0.0 | nothing locked |
| Short turn | the scope is turned 12 degrees in the speculum instead of 30 | 0.0 | below the 20-degree lock: the speculum stays in the holder |
| Too deep | aimed 16 mm in | 0.0 | the tip passed the 13 mm scoring limit |
| Ear let go early | the left lets the ear go 1 s before the speculum is drawn out | 0.0 | the wall springs onto the speculum and triggers the 3 N contact latch |
| Short look | the speculum stays in 0.3 s | 0.5 | in the canal, but no 1 s look |
| Scope kept | the scope is held up instead of going back on its stand | 0.75 | look done; not on its stand, arm not home |

The bare-tip latch (the scope's tip cone in the canal with no speculum locked on) scores 0.0 for the rest of the
trial, so a speculum fitted afterwards earns nothing.

## Reproduce the runs

From this task directory, use the `yam_arena` embodiment with zero object jitter, a 20 Hz control loop and seed 0.
`task_score` reports the final partial-credit score. Both the scripted demo and the Dream-1 trial use a 1,200-step
limit. The following command records the run's camera frames and scorer output.

```bash
task=look_into_the_manikins_left_ear_canal_with_the_otoscope.py
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$task" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --max-steps 1200 --seed 0 \
  --store-frames --log-dir /tmp/otoscope-scripted
```

Use `--policy dream` and a different `--log-dir` for the Dream-1 trial. To reproduce the right-arm-only check,
set `OTO_ONEHAND=right` for the scripted run. For the early-release check set `OTO_EARLY=1` instead.

## Fidelity

Rigid-body simulation (Isaac Sim, PhysX, 20 Hz) at approximate real scale. A rigid sprung slide represents coupled
movement of the soft outer ear and back-upper canal wall. Its 7 mm travel, 4 mm rest opening and 3.6-5.7 N spring
forces are simulation design values. The physical trainer uses a continuously molded flexible canal. The simulated
canal has a square collision cross-section and a drawn round opening. The simulated speculum locks at a 20-degree
clockwise turn with the tip cone 6-16 mm in and weighs 3 g for solver stability. The light is on throughout.
Cameras are 1280 × 704 (Dream-1 uses 320 × 176). The simulator sums the force vectors PhysX reports for the scope and
its specula against the ear and head and takes the magnitude each step. The physical wrist sensor measures the
compensated resultant contact force on the held scope and speculum. The 3 N and 13 mm limits apply to the benchmark
trainer. The one-second insertion models looking. The simulated camera does not render the view through the eyepiece.

A complete Dream-1 run was previously about 10 minutes for 1,200 steps on an RTX 5090. The revised scripted demo
took 112 s on the test host and ended after 657 steps, or 32.9 s of simulated time. Isaac Sim can take longer on
its first start while it compiles shaders.

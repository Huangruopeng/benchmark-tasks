# Look into a training manikin's left ear canal with a pocket otoscope

An adult ear-examination training manikin lies on its right side on a gel head ring, its left ear up. A pocket LED
otoscope (Welch Allyn Pocket Plus LED 22880 type), switched on, rests nose-down on its stand, and three 4.25 mm adult
disposable specula stand tip-down in a holder. The job is the otoscopic look a nurse does in a physical assessment,
done the way it is taught: fit a speculum to the scope (push it on and twist it clockwise until it locks), draw the
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
- **Controlled insertion.** With the canal open, the speculum goes in along the canal's axis to 6-13 mm (the clinical
  0.25-0.5 inch) and stays there for a 1 s look, never pressing the ear or head harder than 3 N and never entering the
  bony canal (deeper than 13 mm).
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
still in, as a hurried one-hand finish would, springs the wall onto the speculum at 11 N (0.0).

In the demo the left hand pinches the helix rim's back-upper part (the clinical grip) and draws it up and back to
its stop; the right hand holds the scope by its handle 10 cm from the head's axis, leaning in from the robot's side,
and brings the speculum down the canal's axis. The wrists stay about 20 cm apart throughout.

Unlike seating an ear thermometer's probe at the canal's mouth, the look needs the ear actively held open by a second
hand for the whole insertion, a speculum fitted by a twist lock first, and the speculum's depth kept inside a 7 mm
window with a force limit.

## Real-world basis

- **Clinical technique.** EBM Consult, "Otoscope Examination": "Use the largest ear speculum that the canal will
  accommodate"; hold the auricle and "pull it upward, backward, and slightly away from the head"; "Direct the speculum
  in a slightly downward fashion as you insert it into the EAC approximately 0.25 to 0.5 inches"; "Insertion of the
  speculum into the posterior/bony portion of the EAC causes pain." JoVE Science Education, "Ear Exam": "pull the
  auricle slightly up and back to help straighten the auditory canal"; "Take care not to insert the speculum too
  deeply, as this may reach the bony canal and cause severe pain." Toronto Metropolitan University, *Introduction to
  Health Assessment for the Nursing Professional*, "Otoscopic Examination": "pull the helix up and back to straighten
  the external canal"; "Gently insert the speculum tip just inside the external auditory meatus."
- **Who does it.** OpenStax *Clinical Nursing Skills* 22.3 "Ears": nurses assess the ears and "Use an otoscope to
  examine the eardrum (tympanic membrane) for color, position, integrity, and landmarks." BLS Occupational Outlook
  Handbook: 3,465,400 registered nurse jobs (2025).
- **The otoscope.** Welch Allyn Pocket Plus LED, models 22880-BLK/-WHT/-PUR/-BLU (Welch Allyn spec sheet SM4121GB):
  total height 16.8 cm, head width 3.0 cm, head depth 5.1 cm, handle diameter 2.1 cm, head and handle 71.7 g without
  batteries, two AA alkaline cells. Its instructions for use: "To turn instruments on and off, and vary intensity,
  rotate switch at base of instrument head" (the scope starts switched on here).
- **The speculum.** Welch Allyn KleenSpec disposable ear specula, 4.25 mm adult (52434-U) and 2.75 mm paediatric
  (52432-U), listed for the PocketScope. The Welch Allyn otoscope instructions for these specula: "To attach the
  speculum onto the otoscope, twist the speculum clockwise into the TipGrip until it locks."
- **The ear canal.** About 25 mm long and 7 mm across in adults; its outer third is elastic cartilage and its inner
  two thirds bone; it runs an S-shaped course (Wikipedia, "Ear canal").
- **The manikin.** Nasco Life/form LF01019 ear examination simulator: an adult head with six interchangeable flexible
  ears, US$1,391 (3B Scientific and Anatomy Warehouse listings). A lower-cost head with a flexible pinna and an
  anatomically correct meatus is the Nasco ear syringing trainer, US$760 (Anatomy Warehouse listing).

## Scene (as in the screenshot; the same in every trial)

A bench in front of two arms (bases 61 cm apart), seen by one overhead camera and a camera on each wrist. Distances are measured forward from the line between the arm bases and sideways from the midpoint between them.

1. An adult ear-examination training manikin head (Nasco Life/form LF01019 type: life-size skin-tone vinyl head, soft silicone ears), lying on its right side on a blue gel head ring on a white sheet, left ear up, face toward the far right, crown toward the far left. Fixed. Its left ear canal opens 30 cm forward, 4 cm left and 14.5 cm above the bench and runs straight down into the head: the outer 8 mm is the cartilaginous part, 11 mm across when open; below it the 7 mm bony part reaches the eardrum 25 mm in.
2. The left outer ear (silicone, 20 g) together with the outer canal's back-upper wall: they slide up and back (toward the robot's left) as one, up to 7 mm, against a spring of 3.6 N at rest rising to 5.7 N. At rest that wall leaves the canal's opening a 4 mm slit; drawn fully up and back, the opening is 11 mm. The helix rim's back-upper part (24 mm long, 10 mm thick, rising 15 mm from the concha and leaning out 20 degrees) is 3.2 cm from the canal's centre.
3. A Welch Allyn Pocket Plus LED otoscope (22880), switched on: black ABS head 30 mm wide and 51 mm deep including its tip cone (15 mm long, 8 mm across at the end, 14 mm at the base); 21 mm handle, 138 mm long, with two AA cells; 118 g. It rests on its stand, nose down: head 25.5 cm forward and 14 cm right, handle level 6.5 cm above the bench, running toward the robot's right. The stand: a grey ABS base 5 x 18.5 cm with two V-rests under the handle and two cheeks beside the tip cone.
4. A white ABS specula holder block 70 x 30 x 60 mm, centred 17.5 cm forward and 11 cm right, with three 15 mm square pockets 2.5 cm apart along its length. In each hangs, tip down by its flange, a black polypropylene Welch Allyn KleenSpec 4.25 mm adult disposable speculum (52434-U): a 27 mm cone from a 13 mm rim with a 16 mm flange to a 4.25 mm tip. A speculum goes on by pushing the scope's tip cone into it and turning the scope clockwise (seen from the eyepiece) until it locks: 20 degrees.

Both arms start at their home pose. The layout is the same in every trial, as in the screenshot.

Real trial: the Nasco LF01019 head (US$1,391) or the Nasco ear syringing trainer head (US$760) on a gel head ring set on a digital kitchen scale in view of a camera; a Welch Allyn 22880 otoscope on a printed stand; KleenSpec 52434-U specula in a printed holder, each marked with two thin bands 6 mm and 13 mm from its tip.

## Scoring

Latched for the rest of the trial (score 0.0): the otoscope or a speculum on it presses on the manikin's ear or head with more than 3 N for 0.1 s; the speculum's tip goes more than 13 mm into the canal (into the bony part); the scope's bare tip cone (no speculum locked on) goes into the canal; or the otoscope falls (its head's centre below 4.5 cm above the bench).

Otherwise: 1.0: a speculum locked on the scope, its tip kept 6 to 13 mm inside the canal for 1 s without a break at some point, and at the end the otoscope back on its stand (its head's centre within 15 mm of its start, nose down within 15 degrees, not held), the speculum out of the canal, the ear let go (back within 1 mm of rest) and both arms back where they started (each gripper's fingertip centre within 5 cm of its start); 0.75: the 1 s look done, the end state not met; 0.5: the speculum's tip 6 to 13 mm inside the canal at some point; 0.25: a speculum locked on the scope; else 0.0.

Locked: the speculum went on by the tip cone 6 to 16 mm into it (within 3.5 mm of its axis) and a clockwise turn of 20 degrees or more, seen from the eyepiece. Inside the canal: the tip within the canal's 11 mm outline, measured from the opening along the canal. The trial ends at the run limit, or once the full score has held 1 s with the otoscope and the ear still.

In a real trial: locked: the speculum stays on when the scope is turned tip-down and tapped (end photo). Depth: on the side and wrist video, the 6 mm band on the speculum goes below the rim of the canal's opening and the 13 mm band never does; the 1 s is timed on the video. Pressing: the head stands on a tared digital kitchen scale in camera view; a reading above 306 g (3 N) for the speculum's contact is a press. The bare tip in the canal, a fall, the ear's rest position, the scope on its stand and the arms against marks at their start are judged from the video and the end photo.

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
| Too deep | aimed 16 mm in | 0.0 | the tip passed 13 mm (13.5 mm, into the bony canal; 6.7 N) |
| Ear let go early | the left lets the ear go 1 s before the speculum is drawn out | 0.0 | the wall springs onto the speculum: 11 N on the ear |
| Short look | the speculum stays in 0.3 s | 0.5 | in the canal, but no 1 s look |
| Scope kept | the scope is held up instead of going back on its stand | 0.75 | look done; not on its stand, arm not home |

The bare-tip latch (the scope's tip cone in the canal with no speculum locked on) scores 0.0 for the rest of the
trial, so a speculum fitted afterwards earns nothing.

## Fidelity

Rigid-body simulation (Isaac Sim, PhysX, 20 Hz), real sizes. The soft outer ear and the outer canal's back-upper wall are one rigid part on a sprung slide (7 mm up and back, 3.6-5.7 N, design values); the canal is drawn round and modelled square. The twist lock is a lock that engages on a 20-degree clockwise turn with the tip cone 6-16 mm in. Speculum 3 g, heavier than the real part, for solver stability. The light is on throughout. Cameras 1280x704 (Dream-1: 320x176). The contact force is the PhysX contact force between the otoscope with its specula and the head or ear bodies, summed each step; the scale reading in a real trial is its vertical part, which is the direction of insertion.

On one RTX 5090: about 10 minutes for a Dream-1 trial of 1,200 steps (592 s on the clock, with Dream answering in 0.53 s on average), and about 3 minutes for the scripted demo (657 steps, 32.9 s of simulated time, 189 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

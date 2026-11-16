# Hand-eye calibrate both of a robot's wrist cameras against a ChArUco board on the bench

A robot that grasps what its wrist cameras see has to know exactly where each camera sits on its gripper: a camera 5 mm
or 1 degree off puts every grasp that far off. That pose is the camera's hand-eye calibration. It is measured when the
robot is built, and again whenever a camera is remounted, replaced or knocked. In the standard eye-in-hand procedure a
calibration board stays fixed, and each arm moves its own camera round it, stopping at a dozen or more poses with the
whole board in view. At each stop the tool records the robot's pose (from its joints) and the board's pose in the
image, then solves for the camera's mount (AX = XB). Today a person drives each arm through those poses, robot by
robot. Here an A4 ChArUco board leans on a beech book stand between the two arms, printed face toward them, and each arm
carries the station's wrist camera. The instruction is only "Hand-eye calibrate both wrist cameras with the ChArUco
board on the bench, then take both arms back to where they started."; it does not say how.

**How this differs from the board's calibration tasks.** Each arm moves its own camera around a fixed board; nothing
is grasped. "Show a webcam its checkerboard in the views a calibration needs" (#2239) is the intrinsic calibration of a fixed webcam: two arms carry a checkerboard and present it to that camera,
and the score is the focal length. Here the board must not move at all (the method needs it fixed in the robot's
frame, and a knocked board ends the trial). The skill is moving each camera through well-spread viewpoints, under the
arm's joint limits, with its own fingers and the other arm kept out of the view and the two arms clear of each other.
The result is each camera's pose on its gripper, solved from the robot's own joint readings. #305 ("Stand a
calibration board in its holder, facing the camera") placed a blank board in a slot, with no pattern and no camera
recording. Here the print is a real ChArUco board, and every sample comes from OpenCV's detector run on what the
camera sees. #827 ("Calibrate a tool tip by touching one point from four angles") calibrates a tool's tip by contact;
no camera is involved. #1289 ("Reattach your detached wrist camera") mounts a wrist camera; this task starts with the
cameras mounted and measures where they are.

The procedure the task checks comes from the hand-eye tools' own instructions:

- MoveIt Calibration's [hand-eye tutorial][moveit]:
  - the target must be "stationary in the robot's base frame";
  - "Once five such samples have been collected, the calibration can be calculated", and it "will typically plateau
    after about 12 or 15 samples";
  - "Be sure to include some rotation between each pair of poses" and "don't always rotate around the same axis", for
    "at least two rotation axes are needed to uniquely solve for the calibration".
- OpenCV's [calibrateHandEye][cv-he]: "A minimum of 2 motions with non parallel rotation axes are necessary to
  determine the hand-eye transformation. So at least 3 different poses are required, but it is strongly recommended
  to use many more poses." Park's method (`CALIB_HAND_EYE_PARK`) is one of the solvers it offers, with Tsai-Lenz,
  Horaud, Andreff and Daniilidis.
- [easy_handeye][ehe]:
  - in eye-in-hand, the target is placed so that "it is fixed relative to the base of the robot";
  - "the end effector must be rotated as much as possible (up to 90°) about each axis, in both directions";
  - its tips are "Maximize rotation between poses", "Minimize the translation between poses" and "Use redundant
    poses".
- OpenCV's ChArUco detector ([`cv2.aruco.CharucoDetector`][cv-charuco]) finds the board, and `solvePnP` gives the
  board's pose in the camera.

Why two arms: each camera rides on its own arm, so each arm makes its own sweep. While it does, the other arm must
stay out of that camera's view and clear of the working arm. A run with one arm calibrates one camera and scores 0.4
(`HEC_ONE=1`, below).

The task is the task file [`hand_eye_calibrate_both_wrist_cameras_on_a_charuco_board.py`](hand_eye_calibrate_both_wrist_cameras_on_a_charuco_board.py)
for the `yam_arena` embodiment (scene, scoring and the scripted demo), with [`bodykit.py`](bodykit.py) next to it.

## Realism check

- **Setting.** Hand-eye calibration is done on the robot's own bench, with the board placed "near the robot, where it
  can be easily seen by the camera" (MoveIt). A board propped on a stand is common, so the camera can view it from in
  front and from the sides. The stand here is a beech book stand set to 45 degrees, with a lip under the board's
  bottom edge. It stands on the station table, 36 to 54 cm in front of the arm bases on the centre line, where both
  wrist cameras can reach it.
- **Board.** A standard ChArUco layout: 7 x 5 squares of 30 mm with 22 mm markers from DICT_5X5_250, MoveIt
  Calibration's default dictionary. It is printed on an A4 plate of 3 mm aluminium composite, a common substrate for
  calibration targets (about 3.8 kg per square metre, so 0.24 kg). Nothing is shrunk. The print fills about a third
  of the camera's width at the 30 to 40 cm the views are taken from.
- **Cameras.** They are the station's own wrist cameras, with their calibrated mounts and lens (390.7 / 390.2 pixels
  at 640 x 480). They are recorded at 1280 x 720 for the calibration, as a person would record them. Their
  intrinsics are taken as known, as MoveIt Calibration and easy_handeye take them from the camera's own calibration.
- **Every step of the real job.**
  - The board is left untouched.
  - Each arm is moved through 12 or more distinct poses, rotating about at least two axes.
  - It stops at each, with the whole board in view and nothing in front of it.
  - Each camera's mount is solved and checked.
  - The arms go home.
  - The one stand-in: a sample is taken automatically when the camera has been held still. A person would press
    "Take sample" in MoveIt's panel, or accept the sample in easy_handeye.
- **YAM limits.**
  - No forces are involved.
  - The demo's views are 30 to 40 cm from the board, with the camera tilted up to 45 degrees from the board's normal
    and rolled up to 180 degrees.
  - Its joints stay at least 14.8 degrees from their limits.
  - The working arm stays at least 15 cm from the waiting arm and 15 cm from the board.
  - In every view the closed fingertips sit at the bottom of the camera's view, below the board.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on Isaac
Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev launchable,
installs everything, including the Dream-1 policy. The scorer also uses OpenCV with its ArUco module (`cv2`, tested
with 4.14), which Isaac Sim installs. No third-party models or assets are used: every object is built in the task
file from boxes.

`yam_arena` task files place single boxes, cylinders and spheres and cannot join them. `bodykit.py` adds bodies made of
several shapes (the board with its print, the stand). Importing it hooks `yam_arena.scene` when the simulator starts,
without changing yam_arena, and lets the task file look at the board and the arms after every control step.

What is modelled and what is not:

- **The board.** A 297 x 210 x 3 mm plate of 0.24 kg, loose on the stand. Its print is laid out from OpenCV's own
  `CharucoBoard`: black squares, and in each white square a marker with its bits, as thin visual boxes. The print is
  visual only.
- **The stand.** A beech base 17.8 x 30 cm and 12 mm thick, with a 10 mm lip and a sloped rest at 45 degrees on a
  block.
- **The cameras' views for the calibration.** Each camera's pose is its gripper's pose in the simulation times its
  mount. From that pose and the lens, the print is drawn into a 1280 x 720 frame: an ideal pinhole with no lens
  distortion, blur or lighting, drawn at twice the resolution and averaged down. OpenCV's ChArUco detector runs on that
  frame.
- **The line-of-sight and touch tests.** For these the arms are boxes round each arm's links, gripper and fingers
  (from the YAM's collision meshes, placed by the arm's joints). The cameras the policy sees are Isaac Sim's own
  renders from the same mounts.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise the score at the end of the run.

A sample of a camera is taken when all of the following hold:

- the whole print is 8 pixels or more inside the camera's 1280 x 720 frame;
- no part of either arm (links, gripper, fingers) or of the stand is in front of the print as seen from the camera,
  tested by lines of sight to points 1 cm apart over the print;
- the board's image has moved less than 2 pixels over the last 0.5 s (10 control steps), about 1 mm at 40 cm;
- the camera is turned 15 degrees or more from every earlier sample of that camera;
- OpenCV's ChArUco detector finds all 24 corners.

The sample is the gripper's pose in its arm's base frame, from the arm's joint angles (the robot's own kinematics), with
the board's pose in the camera from `solvePnP` on the 24 corners.

| Per camera | Measure |
|---|---|
| 0.05 per sample, up to 12 (0.6) | samples taken as above |
| +0.1 | two rotations between its samples of 30 degrees or more whose axes (in the gripper's frame) are 45 degrees or more apart: two non-parallel rotation axes |
| +0.1 | 12 or more samples, the line above met, and OpenCV's `calibrateHandEye` (Park's method) on all its samples puts the camera within 5 mm and 1 degree of its true mount |

The task's score is the mean of the two cameras' scores (at most 0.8). Full score (1.0) needs all of these, held for
1 s:

- both cameras calibrated;
- the board moved less than 1 mm from where it settled (its pose 0.5 s into the run);
- both arms back at their start (each fingertip centre within 5 cm) and still.

The score is 0 for the rest of the run once the board has moved more than 2 mm, or once any part of an arm comes within
2 mm of the plate. Doing nothing scores 0. `-E object_jitter=0` keeps every object exactly where the task file puts
it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hand_eye_calibrate_both_wrist_cameras_on_a_charuco_board.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=704 \
  --instruction "Hand-eye calibrate both wrist cameras with the ChArUco board on the bench, then take both arms back to where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hec-dream
```

The scripted demo reaches full score. The left arm closes its fingers and moves its camera through 13 fixed views,
holding each still for 1.8 s:

- 30 to 40 cm from the board's middle;
- from straight in front, and from 15 to 45 degrees off to its left, top and bottom;
- with the camera rolled between -90 and 180 degrees.

It then goes home, and the right arm does the same on its side:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hand_eye_calibrate_both_wrist_cameras_on_a_charuco_board.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=704 \
  --instruction "Hand-eye calibrate both wrist cameras with the ChArUco board on the bench, then take both arms back to where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hec-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/hec-dream/*.json; do inspect-robots video "$log"; done
```

`HEC_DEBUG=1` prints, every 20 steps (`HEC_DEBUG_EVERY=n` to change), where the board is and how far it has moved, the
arms' nearest approach to it, whether the arms are home, and for each camera:

- its samples so far, and whether its rotations condition is met;
- its calibration error;
- why it is not taking a sample now;
- its TCP.

It also prints a line for each sample taken. `HEC_ONE=1` runs the demo with the left arm only (the right stays home):
the right camera is never calibrated, so the run scores 0.4.

## Building it for real

You need:

- the robot with its two wrist cameras, its joint angles logged at its control rate;
- a 7 x 5 ChArUco board with 30 mm squares and 22 mm DICT_5X5_250 markers, printed on an A4 3 mm aluminium composite
  plate;
- a wooden book stand set to 45 degrees.

Run the trial:

1. Before the trial, an expert calibrates both cameras carefully with the same board (30 or more samples, using MoveIt
   Calibration or easy_handeye). That gives the reference mounts.
2. During the trial, both wrist cameras record at 1280 x 720, and the overhead camera records the board and the arms.

Score it afterwards, offline:

- **Samples.** A stretch of 0.5 s or more where the board's detected corners move under 2 pixels and all 24 ChArUco
  corners are found counts as a stop. Stops are kept as samples with the rules above (15 degrees from earlier samples,
  nothing in front of the print).
- **Calibration.** Each sample pairs the robot's logged joint angles with OpenCV's `solvePnP` of the board. Run
  `calibrateHandEye` (Park) on each camera's samples and compare the result with the reference mount (5 mm, 1 degree).
- **The board and the arms.** Judge from the overhead video whether the board moved or was touched: it is a loose A4
  plate on a stand, so a 2 mm shift shows against pencil marks drawn round it. The arms' return home comes from the
  joint log.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 2,400 steps (569 s on the clock, with Dream answering in 0.83 s on average), and about 3 minutes for the scripted demo (1,881 steps, 94.0 s of simulated time, 201 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, the scripted demo is deterministic and the scorer draws no
random numbers.

[moveit]: https://moveit.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html
[cv-he]: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
[ehe]: https://github.com/IFL-CAMP/easy_handeye
[cv-charuco]: https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html

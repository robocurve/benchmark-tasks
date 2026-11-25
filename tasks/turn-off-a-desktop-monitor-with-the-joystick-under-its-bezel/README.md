# Turn off a monitor with the joystick nub hidden under its bottom edge: press it straight up, then push it to Power Off

**Instruction:** "Turn the monitor off with the joystick button under the middle of its bottom edge. If no menu is on the screen, press the button straight up once to open the menu, then push it toward the back of the monitor (Up) to choose Power Off. If the menu is already showing, just push Up; if the Settings menu is showing, push the button left until it closes first. Do not change the volume or the input. Then take your hand away."

An LG 24MR400-B monitor stands free on the desk, switched on. Its only control is the joystick button under the
middle of its bottom edge: a 9 mm nub pointing down that is both a push button and a four-way stick. The robot slides
a level fingertip in under the bezel, presses the nub straight up to open the main menu, pushes it Up (toward the
back of the monitor) to choose Power Off, and takes its hand away. If a menu is already open, it follows the screen.

## Scoring

Score 0 if the screen is still on at the end (any picture or menu) or the monitor toppled (tilted over 30 degrees). Otherwise the lowest of these numbers: 1.0; 0.5 if the volume, input or picture mode differs from the start; 0.25 if the monitor's base moved more than 25 mm or turned more than 8 degrees; 0.75 if it moved more than 10 mm or turned more than 3 degrees; 0.75 unless the hand is out from under the monitor (tool point 40 mm or more in front of the screen face, or 73 mm from the joystick) for 1 s with the monitor still. Success: 1.0 held 1 s more.

| Outcome | Score |
|---|---|
| Screen dark, volume / input / picture mode as at the start, base within 10 mm and 3 degrees, hand away 1 s | 1.0 |
| As above, but base moved 10-25 mm or turned 3-8 degrees, or the hand still under the monitor | 0.75 |
| Screen dark, but the volume, input or picture mode was changed on the way | 0.5 |
| Screen dark, but the base moved more than 25 mm or turned more than 8 degrees | 0.25 |
| Screen still on (any picture or menu, including doing nothing or opening the menu only), or monitor toppled | 0 |

Every start state shows a lit screen, so doing nothing scores 0. Turning it off and pressing again turns it back on
(the manual's power-on press): 0.

**Real trial.** From the trial video and an end photo: the screen dark; the base against a masking-tape outline,
measured with a ruler (3 degrees is 10 mm at the base's 190 mm side). Then the experimenter switches it on (one
press) and reads the volume, input and picture mode on the screen (Settings > Quick Settings / Input / Picture).

## The monitor and its control (maker's manual)

LG 24MR400-B, 23.8 in IPS, 1920 x 1080, about US$97-130 (Best Buy US$96.92 and Newegg US$106.91 listed at
https://pangoly.com/en/product/lg-24mr400-b ; US$129.99 at
https://us.smartprix.com/monitors/lg-24mr400-b-24-inch-full-hd-monitor-ppd1t8xdb5bn). A cheap current model on
purpose; any monitor with the under-edge joystick is the same job.

Owner's manual (22MR410 / 24MR400 / 27MR400; https://www.trovaprezzi.it/manuali/entry_lg_24mr400.pdf , same text at
https://www.manualslib.com/manual/3418390/Lg-24mr400-B.html ):

- Product Description: "A: Joystick Button", drawn at the centre of the bottom edge.
- "Power on: Press the joystick button once with your finger to turn on the monitor." "Power off: Press and hold the
  joystick button once more than 5 seconds with your finger to turn off the monitor." "Volume control: You can
  control the volume by moving the joystick button left/right." Up/down: "Input information check".
- "Activating the Main Menu: 1 Press the joystick button ... 2 Move the joystick up/down and left/right to set the
  options. 3 Press the joystick button once more to exit the main menu." The main menu drawing: Up [Power Off], Left
  [Input], Right [Settings], Down [Picture Mode], centre [Exit].
- Settings: "To return to the upper menu or set other menu items, move the joystick to Left or pressing it. If you
  want to exit the OSD menu, move the joystick to Left until you exit." Tabs: Quick Settings, Input, Picture, General.
- Dimensions: with stand 540 x 200 x 414 mm (W x D x H), without 540 x 50.5 x 320.6 mm; 2.9 kg with stand. Tilt -5
  to 20 degrees.

The task uses the menu route the instruction names; the 5 s hold, also in the manual, is accepted as well.

**Estimates** (each checked in a minute with a ruler on the real monitor): screen bottom edge 93.4 mm above the desk
(414 - 320.6); nub 9 mm across with a domed end, 4.5 mm below the bezel, its axis 12 mm behind the front face; stand
neck 57 mm behind the face; base plate 210 x 190 x 10 mm starting 10 mm behind the face; menu Up = pushing the nub
toward the back of the monitor, Left = toward the left of a person facing the screen.

**The switch.** LG does not publish it. It is modelled on a typical four-way + centre-push switch (data sheet:
https://www.mouser.com/datasheet/2/15/SKRH-1370966.pdf ): a direction at 1.23 N and 0.25 mm measured 4.3 mm up the
stem (3.3 degrees of tip), the push at 2.35 N and 0.15 mm. In the task the nub rocks on two sprung axes about a point
10.5 mm above its tip (registering at 3.3 degrees, stop at 5) and slides on a sprung axial button (registering at 0.15
mm, stop at 0.4 mm). Whatever registers first is the event; nothing more until the nub is back near centre (under 1.5
degrees and 0.07 mm). A press acts when released, or after 5 s held. So a push leaning more than about 13 degrees off
the axis, or a fingertip slipping 0.5-0.6 mm sideways on the nub while it presses (friction 0.3), registers a
direction, as on the real control.

## Menu rules in the task

| Screen | Press | Up | Down | Left | Right |
|---|---|---|---|---|---|
| Picture (no menu) | main menu | input info | input info | volume -1 | volume +1 |
| Main menu | exit | **Power Off** | Picture Mode list | Input list | Settings |
| Settings (tabs) | open tab | cursor | cursor | exit | open tab |
| Settings page | back | - | - | back | - |
| Input / Picture Mode list | select (changes it) | cursor | cursor | back to main | - |
| Off | power on | - | - | - | - |

## Reach (why the hand is level)

The nub is 89 mm above the desk under a screen whose bottom edge is 93 mm up, so the gripper cannot point up at it.
The right hand comes in level, pointing toward the back of the monitor, fingers closed and stacked: the back of the
upper finger is a 15 mm wide flat face 4 mm above the tool point over the last 13 mm. Pressing, it lies 2 mm below the
bezel at the screen's front edge. From the robot's collision meshes and the arm's inverse kinematics, this level
pose reaches the nub only 0.40-0.44 m in front of the right base with the joystick near the centre line; the
monitor's turn of 10 degrees toward the right arm is how a person sits at it.

## The scripted demo

Run demo-88dd9305-a1 (no jitter, full length): 1.0. Button down at 10.15 s with the nub tipped 3.05 deg toward the user and 1.55 deg sideways (the threshold is 3.3 deg) and pushed 0.154 mm; main menu at release, 10.40 s; Up at 14.70 s: off; hand out and monitor still, success at 18.2 s. The monitor did not move. From the other two start states (same code, jittered 10 mm, run monoff-wrong-88dd9305 episodes 2-3): main menu open -> Up -> off, 1.0; Settings page -> Left, Left, press, Up -> off, 1.0. The straight press is only just straight enough: the arm's softness leaves about 0.25 deg of margin on the nominal layout.

The demo reads the screen before each action and picks the next: picture -> press; main menu -> Up; Settings or a
list -> Left. It presses by rising 5 mm/s until the button registers, then lets go; it pushes Up with the fingertip's
end face and Left with its side, 2.5 mm up the nub's dome. While settling between actions it corrects its plan by the
error between where it sent the hand and where the joint readings put it (the arm sags when reaching level and low).
While pressing or pushing it holds the fingertip where the action began along the two axes the action does not
drive, from the same joint readings: the level arm is soft (about 200 N/m upward), and without this the fingertip
slid off the dome under an Up push and dragged the nub's tip sideways during a press (GPU probes).

## Wrong way

A diagonal jab (rising and moving 7 mm to the side at once, from the picture) registers LEFT before the button: volume 30 -> 29. The hand then presses straight and pushes Up, so the monitor goes dark, but a setting changed: 0.5 (run monoff-wrong-88dd9305, episode 1). In that run's Settings-page episode the jab reached the button's 0.15 mm and the 3.3 deg tilt in the same 20 Hz reading and counted as a press (the reader checks the button first).

## What is simplified

- The picture and menus are drawn shapes on the panel face (no text, no logos), not emitted light. The monitor has no
  documented power light, so none is drawn.
- One rigid monitor body (panel, stand, base) of 2.9 kg; the stand's tilt hinge is fixed at 0 degrees; desk friction
  0.5 / 0.45 (estimate).
- The joystick's joints are read every control step (20 Hz). No menu time-out is modelled (the manual gives none).
- Start menu state by episode number (none, main menu, Settings page).

Simulation scores: Dream-1 run: 0.0 (1,600 steps); scripted demo: 1.0 (off at 14.7 s); wrong way (diagonal jab): 0.5

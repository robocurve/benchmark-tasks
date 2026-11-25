# Lock out a travel adapter's UK pins with its side toggle, then push the adapter fully into a UK socket

Push the middle toggle on the side of the travel adapter all the way down until the UK pins click out and lock, then plug the adapter into the extension lead's socket, push it in until it sits flat on the socket, and let go.

## Scoring

Judged at the end of the run from the trial video, a photo and a ruler. Locked out: the UK toggle sits at the end of its slot and the UK pins stand fully out (earth pin 22.3, the others 17.7 from the adapter's base); pressing the pins with a finger does not move them. In the socket: all three pins are inside their openings. Gap: how far the adapter's base stands above the socket face at its highest corner (ruler or feeler card). Released: the centre point between each gripper's fingers (about 13 mm back from the fingertips) at least 3 cm from the adapter, and the adapter has been still for 1 s. Score 0 if the pins are not locked out, the adapter is not released or still moving, or it fell off the table (doing nothing scores 0). Otherwise: 1.0 if in the socket with a gap of 1.5 mm or less (flush); 0.75 if in the socket with a gap over 1.5 and up to 6 mm; 0.5 if in the socket with a gap over 6 mm; 0.25 if the pins are locked out but not in the socket's openings.

How `score()` reads each condition (adapter frame: origin at the centre of its pin face, x along its 71.1 mm length,
y toward the toggle side, z up; socket frame: origin under the centre of the socket, z up, socket face at z = 33 mm):

- **Locked out:** the UK pin carrier's slide is within 1 mm of the end of its 22.8 mm travel and the catch has dropped in
  (catch slide past half of its 2.5 mm). The catch only drops in during the last 0.8 mm, so the carrier can back off
  at most 0.8 mm and still count.
- **In the socket:** each pin's tip, moved with the adapter into the socket frame, lies inside its opening (opening size
  plus 0.3 mm) and at least 1 mm below the socket face.
- **Gap:** the highest of the four corners of the adapter's base, above the socket face.
- **Released:** both grippers' centre points at least 30 mm outside the adapter's bounding box. **Still:** over the last
  1 s it moved under 1 mm and turned under 2 degrees. **Fell:** its lowest point ever more than
  20 mm below the table top (remembered).

`success()` is a score of 1.0. Why the shortcuts fail: a toggle stopped short of the catch leaves the pins free, and
the socket's contact springs push them back into the adapter as it goes in, so it ends with its pins not locked out
(0); pushing the adapter in without the pins out cannot get anything into the openings (0). Wrong-way run (a copy of this file that pushes the toggle only half way): the toggle stopped at 12.5 of 22.8 mm, unlocked; plugged in and pressed flush, the contact plates pushed the free pins back 4.8 mm, and it scored 0.

## Run and score

From this task folder, with the official YAM Arena environment installed:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lock_travel_adapter_uk_pins_out_and_plug_it_in.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Push the middle toggle on the side of the travel adapter all the way down until the UK pins click out and lock, then plug the adapter into the extension lead's socket, push it in until it sits flat on the socket, and let go." \
  --scorer task_score --max-steps 2400 --epochs 1 --store-frames --log-dir ./demo-run
```

For a model trial use `--policy dream`. Each trial allows 2,400 control steps (120 s at 20 Hz);
the scripted demonstration takes 46.9 s. `pytest` here checks the scorer on named layouts.

## The scene

Products, positions and estimates are in task.json's `setup`. Both arms start at their ready pose.

- **Travel adapter (free body with two sliding links, 190 g, friction 0.6/0.5):** body
  71.1 x 50 x 52.1 with its universal socket and USB ports as dark marks; three toggle tabs
  7.5 wide, 3.5 proud, 9 tall, 11 mm apart on the side facing the robot. The UK pin carrier
  (15 g) slides 22.8 mm along the adapter's height with 2.0 N static / 1.5 N sliding friction;
  it carries the middle toggle's tab and the three BS 1363 pins (line and neutral 6.4 x 4, 17.7 long,
  22.2 apart; earth 4 x 8, 22.3 long, 22.2 mm from their line). The lock is a
  2 g catch on a 2.5 mm slide, sprung toward a rail on the carrier (300 N/m, 4.5 mm
  preload, ~0.4 N of drag); in the last 0.8 mm of travel it drops over the rail's top, which then bears on it if the
  pins are pushed back. The toggle takes about 2.5 N to push; nothing in the model springs the pins back in.
- **Extension lead (fixed):** body 90 x 60 x 33, black, with a short length of its cable. Socket
  openings 0.75 mm wider than each pin on every side, with a 1 mm 45-degree lead-in, in the BS 1363 layout. The
  line and neutral openings each hold a 2.5 mm contact plate on a 3 mm slide (2000 N/m,
  2 mm preload), bevelled 20 degrees from upright over its top 6.9 mm, friction 0.3/0.25:
  about 6-8 N to push the pins past the bevels and about 5.5 N to slide home (estimates). Only the openings matter;
  the earth opening is a plain slot.

## Scripted demonstration

The right hand holds and carries; the left works the toggle. The demonstration reads (never moves) the adapter's pose
and the UK carrier's slide position to aim each move. The right hand, top-down, pinches the adapter's end faces (jaws
closed 3 mm past each face; the push tilts it about 2 degrees in the hand). The left hand, pitched 50 degrees down with
its gripper open, pushes the UK tab with one fingertip (about 6 mm thick; the neighbouring tabs are 11 mm away) in
10 short steps aimed from the read slide position, the last held at the end of the slot. During the push the
tool points come within 73 mm; in the offline hull check the two hands stay at least 6 mm apart.

| Time (s) | Hand | Step |
| --- | --- | --- |
| 0.0 to 2.0 | left | park |
| 0.0 to 5.0 | right | over the adapter, fingers down at its ends, pinch them |
| 5.0 to 8.0 | right | lift it to the hold point |
| 8.0 to 12.0 | left | high over, over and beside the UK toggle (pose read) |
| 8.0 to 21.5 | right | hold still for the toggle |
| 12.0 to 18.3 | left | push the toggle down in short read-aimed steps; hold it at the end |
| 18.3 to 23.5 | left | lift the finger, back off, clear to the left, park again |
| 21.5 to 24.9 | right | lift with the pins out and carry it over the socket |
| 24.9 to 29.2 | right | down over the openings; line the earth pin up (two pose reads) |
| 29.2 to 31.5 | right | pins 3 mm into the openings, let go |
| 31.5 to 40.7 | right | fingers up, close, fingertips on the top, press it down flush, ease off |
| 40.7 to 43.9 | right | lift off and park |

Demonstration result (version 35b633f0d022, no jitter): 1.0 at 41.3 s (844 steps). By 22 s the pin carrier was at the end of its slot with the catch in; once let go the adapter scored 0.5 (pins in the openings, over 6 mm proud) from 32.2 s, and the press brought the gap to 0.3 mm; both grippers ended over 3 cm clear.

## Simulation and the real objects

Rigid bodies, no electricity. The pin carrier is a slide with 2 N of stiction; the lock is a sprung catch that drops over a rail on the carrier in the last 0.8 mm (inner layout estimated; the release button is not modelled). The socket's line and neutral contacts are sprung plates (~8 N to push home, estimate); its shutters are not modelled. Opening clearance 0.75 mm with 1 mm lead-ins, toggle and pin positions are estimates from the maker's drawings.

- Adapter: EPICKA TA-105 universal travel adapter, 2.8 x 1.97 x 2.05 in, 6.7 oz, $22.99
  (https://www.packhacker.com/travel-gear/epicka/travel-adapter/); use and lock from the maker's manual: "Push the
  desired toggle (A) all the way out until a 'pop' sound is heard and locked" and, to put it away, "Press & hold the
  lock button (B) and slide the toggle (A) all the way back"; drawings there show three toggles on one side, the UK
  set on the middle one (https://www.epickatech.com/blogs/user-manual/ta-1052-user-manual). Listings give 5.1 to
  7.0 oz; 190 g is used.
- UK pin sizes: BS 1363, as summarised at https://en.wikipedia.org/wiki/BS_1363 (line and neutral 6.4 x 4.0 mm,
  17.7 mm long, 22.2 mm apart; earth 8.0 x 4.0 mm, 22.3 mm long, 22.2 mm from the line-neutral line).
- Extension lead: Masterplug EXS1315B-01, one shuttered BS 1363 socket, 33 x 90 x 60 mm, 5 m cable
  (https://www.masterplug.com/uk/products/work/heavy-duty-extension-leads/EXS1315B-01); GBP 12.89
  (https://alertelectrical.com/masterplug-permaplug-1-socket-heavy-duty-5m-extension-lead-black-exs1315b.html).
  Both objects together cost under $45 and nothing is used up.
- Estimates, each checkable in a minute: toggle tab size and spacing, slot travel and where the pins sit on the base
  (ruler); the toggle's push, about 3 N (press it with a finger standing on a kitchen scale's pan, the adapter held
  down by the other hand, and read the peak); the push to plug in, about 8 N (extension lead on a kitchen scale, read
  the peak while pushing the adapter in); the openings' clearance (feeler card or calipers); frictions.
- Code: the run-context, grip-rotation and pose-read helpers follow the task folder garnish-glass-rim-with-lime-wedge;
  the rest is written for this task.

## Reset

About 30 seconds, nothing is used up. Pull the adapter straight out of the socket. Press and hold its lock button and slide the UK toggle back up to the top of its slot so all the pins are inside, then let go of the button. Stand the adapter on its pin face at its start mark (a pencil outline on the table, 71 x 50 mm, centred at (200, -140)), toggles facing the robot. Check the extension lead is still taped down at its mark, socket face up and unplugged from the mains. Return both arms to the ready pose.

# Clip an IV line to the mattress, leaving a slack loop

A hospital-bed mock-up: a dark-blue mattress pad covers the table in front of the robot. A skin-coloured forearm block
lies on its right side with a cannula hub resting in a small holder (the dressing) on top. A clear 5 mm IV tube leaves
the hub and lies straight across the mattress to the robot's left; its far end is free (the line to the pump is not
modelled). A white U-clip is glued to the mattress at the robot's midline. The robot must press the tube into the clip
so that between the clip and the hub the tube lies on the mattress as a loose loop with 5-15 cm more tube than the
straight clip-to-hub distance, without lifting the hub out of its holder, and let go. It is a mock-up only: there is
no patient and no fluid. The Merck Manual (Professional) procedure for inserting a peripheral IV catheter says: "Loop
the IV tubing (or saline lock tubing) and tape it to the skin away from the IV insertion site, to help prevent
accidental traction on the tubing from dislodging the catheter." Here a bed clip stands in for the tape, and the
5-15 cm slack band is a task choice, not a clinical value. A meta-analysis of 76,977 adult peripheral IV catheters
(Marsh et al., J Adv Nurs 2020) gives a pooled dislodgement rate of 6.0%. This task tests a target slack length
between an anchor point and a fragile end. It is not winding slack up or removing
it, and it is not a drip-loop height: a taut, clipped tube scores 0 (checked offline on
a synthetic shape; see below).

The scripted demo uses both arms at once. A CPU inverse-kinematics check with a top-down grip shows that the pin point
18 cm right of the clip (the loop's clip end) is out of the left arm's reach, and that the tube from about y = 9 cm along
its starting line, including the point that must end in the clip, is out of the right arm's reach. So the right
hand pins the loop end while the left hand draws the tube over the slot and lowers it in. A one-arm strategy (for
example, pushing the loop into shape first) has not been tested or ruled out.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 installed (`source IsaacLab-Arena/yam.env`).
- `cablekit.py` and `cabledemo.py` are in this folder (the cable/tube simulation and the scripted-demo runner).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 \
    --instruction "Press the patient's clear IV tube into the white clip on the mattress, leaving a loose loop on the mattress between the clip and the cannula hub on the forearm: 5 to 15 cm more tube than the straight clip-to-hub distance. Do not lift the hub out of its holder. Then let go of the tube."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out --max-steps 1200
```

The demo is closed-loop: grasp points and the sideways correction come from the live tube state. The target arclength
at the clip is the straight hub-to-clip distance plus 10 cm. Both hands take the tube at the same time. The right hand
grips 18 cm before the target (hub side) and the left hand 4.5 cm after it. The right hand then slides its point low
along the mattress, so the tube leaves the hub straight and turns in a wide U, and pins it 18 cm right of the clip.
Meanwhile the left hand draws the tube over the slot to 3.3 cm past the clip. Both wrist cameras face the clip. The
left hand lowers the tube into the slot, correcting sideways from the measured tube position, and then both hands let
go and lift away. Success holds for 1 s and ends the episode at about step 250 (~12.5 s) with task_score 1.0 and a
slack of about 10 cm. `demo/` holds the videos, `setup.png` and `result.json`.

Shortcut variants (test only, `IV_CHEAT=<name> python cabledemo.py task.py out --max-steps 600`), all score exactly 0:

| `IV_CHEAT=` | what it does | result |
|---|---|---|
| `nothing` | waits | 0 (tube not in the clip) |
| `taut` | pins the tube 9 cm short of the clip with it running straight from the hub, then lays the next part over the clip and lowers it | 0 (a taut tube runs diagonally into the slot and does not seat; an offline check of `score()` on a seated, taut shape gives slack 1.5 cm, which also scores 0) |
| `unclipped` | forms the same 10 cm loop but lays the tube on the mattress 4 cm beside the clip | 0 (not seated) |
| `dislodge` | the right hand first lifts the tube next to the hub, which pulls the hub out of its holder (it ends beside the forearm, off it), then runs the normal demo | 0 (hub out of its holder; the tube also ended unseated, so this run fails two conditions) |
| `hold` | does the demo but keeps both hands on the tube | 0 (grippers not released; the log also shows the tube not seated in the slot while held, so it fails two conditions) |

## Scoring (`score()`, evaluated at the end; `success()` must hold 1 s)

- **0** if any of these holds:
  - a gripper's centre is within 3 cm of the tube or hub;
  - the hub has left its holder (its underside more than 7 mm above the forearm, or its centre off the holder
    footprint);
  - the tube is not seated in the clip (it must run through at least 12 mm of the 20 mm slot, with its top not
    above the 12 mm walls);
  - any tube point between the hub and the clip lies off the mattress or hangs below its top;
  - the tube is still moving (mean segment speed above 5 cm/s, or mean spin above 0.5 rad/s).
- Slack = tube length from the hub face to the middle of the tube's run through the slot, minus the straight distance
  between those two points.
- **1.0** for a slack of 5-15 cm (success).
- **0.5** for a slack of 3-5 cm or more than 15 cm.
- **0** for a slack under 3 cm (taut: no loop).
- `distance()` is |slack - 10 cm| once the tube is seated, else 1 m. `progress()` reports the stages hub_in_holder,
  tube_in_clip, loop_slack_ok, loop_on_mattress and released.

## Runtime and seeds

- Run limit 1,200 steps (1 min at 20 Hz). The demo finishes at about step 250. One trial takes ~5-10 min wall time on
  one GPU, including the ~1-2 min Isaac Sim start-up.
- The layout is fixed with `object_jitter=0`, and the tube always starts straight, so every trial starts identically.
  `VARIATION` in task.py lists no variation: the hub sits in a fixed holder.

## Sources

- Merck Manual Professional, How To Insert a Peripheral Intravenous Catheter:
  https://www.merckmanuals.com/professional/critical-care-medicine/how-to-do-peripheral-vascular-procedures/how-to-insert-a-peripheral-intravenous-catheter
- Marsh N, et al. Peripheral intravenous catheter non-infectious complications in adults: a systematic review and
  meta-analysis. J Adv Nurs 2020;76(12):3346-3362. https://pubmed.ncbi.nlm.nih.gov/33016412/
- O*NET 29-2061.00, Licensed Practical and Licensed Vocational Nurses (task: "start intravenous fluids"):
  https://www.onetonline.org/link/summary/29-2061.00

# Heat-shrink a sleeve over a crimped cable lug: one hand turns the cable, the other works a heat gun along it

A common last step of a power-cable termination: an adhesive-lined heat-shrink sleeve, already slid over a crimped lug's barrel and the jacket behind it, is
shrunk with a heat gun. The maker's steps (TE Connectivity 114-120022): "Start to shrink the tubing starting from one end of the assembly. Work
progressively to the end of the tubing ensuring uniform heat application. This can be achieved by either rotating the assembly or the heat gun. Ensure
that the end of the heat gun does not touch the exterior of the tubing otherwise splitting may occur."

The difficulty is an even dose of heat from a tool that must not touch:

- **Every side, every part.** The jet heats the side of the sleeve facing it. The far side recovers only when the cable is turned or the gun is taken round to it, and the
  whole length only when the jet is worked from one end to the other, by moving the gun or by moving the cable through it.
- **Not too little, not too much.** Too far or too fast, a part stays cold; held still, it chars. The nozzle must stay at a working distance and never
  touch the sleeve, or it splits.
- **The holding hand stays out of the hot air.** It holds the cable by its jacket behind the sleeve.
- **The gun is switched on first** with its switch, and switched off when it is back in its stand. It blows only while it is
  grasped by its grip (fingers on both sides of it), off its stand and clear of the table: left running in the stand, slid along it, or
  nudged out of it, it heats nothing.

**The arms.** In the demo the left arm holds and turns the cable, and the right arm works the switch and the gun. The scoring asks for the result only:
any division of the work is allowed.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`).
- `mechkit.py` and `demokit.py`, in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying them.
  This copy of `mechkit.py` adds a centre-of-mass option to `free()`, used for the gun.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 3600 \
    --instruction "Switch the heat gun on, shrink the sleeve over the lug and cable with it, then put the gun back in its stand, switch it off and move both hands away."
```

The scene is the same for every seed.

## The demo

From this folder:

```bash
python demokit.py task.py OUT                                   # the full-score run, with video
python demokit.py task.py OUT_checks --mode still,touch,stand,hover,nudge --fast   # the failure checks
python check_score.py                           # the scorer on hand-built states, with no simulator
```

The demo is a scripted reference, not a policy. It reads the cable's and the gun's poses from the simulator; both are in view of the cameras. It never
reads the scorer.

1. The right hand presses the gun's push switch with closed fingertips: the gun is on.
2. The left hand takes the cable by its jacket end and lifts it level to the work line; the right hand takes the gun by its grip, fingers along the grip,
   jaws above and below it.
3. Six passes. Before each, with the jet raised off the sleeve, the left hand turns the cable about its own axis to -120, -72, -24, +24, +72 and
   +120 degrees. The right hand brings the jet's aim onto the cable's axis 50 mm from the nozzle, where the cable really is (the 745 g gun sags 2 to 4 cm
   and tips a few degrees in the hand), and then holds the gun still. The left hand slides the cable through the jet, from 3 cm past the lug end to
   1 cm past the jacket end, at 11 mm/s on the two outermost turns and 18 to 21 mm/s on the others. The gun is held with its jet turned 20 degrees from
   square, so that its barrel angles away from the left hand.
4. The right hand lays the gun back in its stand, slides off its grip, and presses the switch again: the gun is off. The left hand lays the cable back on
   its V-blocks. Both hands back away.

It scored 1.0 in GPU physics in 129.5 s of simulated time (2,590 steps): all 8 stations recovered, the lowest side's dose 1.94, the highest
3.72 (it chars at 4.5), and the fingers' dose 1.79 (it fails at 3).

Failure checks (`--mode`, CPU physics):

| mode | what the script does | score |
|---|---|---|
| `still` | switches the gun on and holds the jet on mid-sleeve for 10 s, without turning the cable | 0.0 (nothing recovered: the far side gets no heat) |
| `touch` | switches the gun on and drives the nozzle onto the sleeve | 0.0 (split) |
| `stand` | switches the gun on and leaves it in its stand; the left hand slides the cable through the stand's jet, turning it | 0.0 (no heat) |
| `hover` | as `stand`, with the right hand open round the gun's grip, not closed | 0.0 (no heat) |
| `nudge` | as `hover`, after closed fingers push the gun 6 cm out of its stand by its rear cap (it ends on the table) | 0.0 (no heat) |
| `none` | does nothing | 0.0 |

`check_score.py` checks the scorer without the simulator: the tiers, the 2 s hold, the grasp, and the stand gate on hand-built gun poses (slid
3.5 or 5 cm along its cradles, or lifted 5 mm, the gun still lies on its stand and gives no heat; lifted 1.5 cm, it is off).

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold: `score()` is 1.0 only once the 1.0 state has
held 40 control steps (2 s) in a row (at most 0.95 until then), and `success()` is true from then on, so the two agree.

The sleeve is 8 stations of 7.5 mm along its length, each of 6 sides. A side recovers once its dose of hot air reaches 1 (one dose: about 1 s
square in the jet within 50 mm of the nozzle), and chars past 4.5. A station has
recovered when all six of its sides have. The gun gives heat only when it is switched on, grasped by one hand with its fingers closed on both sides of
its grip (within about 3 mm of it), off its stand (no part of it lying on the stand's blocks) and clear of the table.

- **0:** if any part of the nozzle (its tip, rim or side, or the gun body's front rim) ever comes within 3 mm of the sleeve, the gun running or not (it
  splits), if the fingers take more than 3 doses (the hottest fingertip of either hand at each moment, added up), or if the cable or the gun falls off
  the table. Otherwise the tiers below add.
- **0.0625 for each recovered station,** 0.5 when all 8 have.
- Once at least half have recovered: **0.2** more when both end stations have (adhesive showing at both ends); **0.2** more when none has charred;
  **0.1** more when the gun lies in its stand (on its cradles, its nozzle within 3 cm of its place, aimed and rolled within 15 degrees), switched off, with every fingertip at least
  5 cm from the gun's and the cable's centre lines. The adhesive tier follows the dual-wall guidance "Installation is complete when the tubing conforms to
  splice and sealant flow is apparent" (Thomas & Betts Shrink-Kon installation guidelines).
- **1.0** once all 8 stations have recovered, both ends, none charred, the gun in its stand switched off and every fingertip clear, held 2 s.

Real trial: mark the 8 stations' middles on the sleeve before the trial, 3.75 mm from its jacket end and every 7.5 mm after (stations 1 to 5 lie over
the jacket, 6 to 8 over the barrel). Afterwards, measure each
station's outside diameter at its mark with a caliper twice, 90 degrees apart. A recovered station conforms to its core: at most 15.0 mm across over the
jacket and 16.0 mm over the barrel (ATUM's 2.25 +/- 0.55 mm recovered wall, shrunk onto an 11 or 12 mm core, gives about 13.1 to 14.8 mm and 14.0 to
15.6 mm; the sleeve as supplied is over 19 mm across). Look for adhesive at both ends and
for char. A 60 C indicating label on every finger pad of both hands must not change. From the film: heat counts only
while the gun is in a hand and off its stand (a sleeve heated by a gun left running in its stand, or slid along it, scores 0); contact, the switch off at the end (the switch at OFF, the
fan stopped) and the 5 cm clearance are read from the film too.

## Real trials: the build

- **Cable.** A 15 cm stub of 2 AWG battery cable with a compression ring lug crimped on (barrel about 12 mm across, tongue about 35 x 17 mm), and a 60 mm
  piece of TE Raychem ATUM 19/6 adhesive-lined 3:1 tubing (19.0 mm bore as supplied, 6.0 mm recovered, 2.25 mm recovered wall with 0.76 mm of
  adhesive, full recovery at 110 C), slid over 35 mm of jacket and 25 mm of the 30 mm barrel. A new stub each trial.
- **V-blocks.** Two steel V-blocks at x 27 and 37 cm, y 3 cm, holding the cable's axis 13 cm above the table, the jacket's cut end at x 22 cm; the first
  under the jacket, the second under the lug's tongue.
- **Heat gun.** A Weller WLHG1212A (1,200 W, 745 g, 250 x 230 x 80 mm, a 45 mm body; two stages, OFF / I / II), on
  stage I (check the nozzle's temperature with a thermocouple), with its plain nozzle; route its cord away from the work and the arms. Its stand:
  two cradles under the body and a block under the grip, holding it on its side with the nozzle at (31, -3) cm, 10 cm up, pointing +y.
- **Calibration.** With the real gun at its setting, find the time square in the jet at 50 mm that recovers the sleeve (the model's dose 1) and the time
  that chars it, and scale the doses to them.
- **Safety.** A fire-safe mat under the work, the stand's nozzle aimed away from people, the gun's power on a switch the supervisor holds, the e-stop
  in reach, and heat-resistant pads on the holding gripper.

## The model behind it

The cable and lug are one rigid body; the sleeve is drawn on it, does not slide and has no collider of its own (the nozzle touching it is read in
code, from points round the nozzle). The shrink is code: each side's dose rate falls off with distance
along the jet (full to 50 mm, none past 120 mm), with distance off the jet's axis (Gaussian, 12 mm) and with the square root of how squarely the side faces
the jet. A station that has recovered is drawn shrunk onto the barrel and jacket, and the adhesive is drawn at an end once its station has recovered. The
gun is a rigid body of 0.745 kg (the Weller's mass), modelled 52 mm across and 20 cm long (the maker's 45 mm body and 25 cm), its centre of mass set over
its grip; its switch is code, a push toggled by a fingertip between OFF and stage I (the real switch slides), and it blows only when switched
on, grasped by one hand with its fingers closed on both sides of its grip, off its stand and clear of the table. Its cord is not simulated. Any part
of the nozzle within 3 mm of the sleeve splits it, running or not.

## Attribution

The task statements quoted in `task.json` are from the O*NET database (O*NET OnLine, task statements for 47-2111.00,
https://www.onetonline.org/link/summary/47-2111.00) by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the
CC BY 4.0 licence. O*NET is a trademark of USDOL/ETA. Sources quoted:

- the heating steps: TE Connectivity application specification 114-120022
  (https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=114-120022&DocType=SS&DocLang=EN);
- the sleeve's figures: TE Raychem ATUM data sheet (https://www.esto.de/media/pdf/c8/1e/40/ATUM.pdf);
- the adhesive guidance: Thomas & Betts Shrink-Kon installation guidelines
  (https://tnb-canada.com/en/pdf-catalogues/wire-termination/heat-shrinkable-tubing/installation-guidelines-and-cross-reference.pdf).

- the heat gun: Weller WLHG1212A product sheet (https://assets.testequity.com/te1/Documents/pdf/weller/Weller_WLHG1212A_Product-Sheet_0125.pdf); its
  mass and size, Weller's datasheet (https://www.weller-tools.com/us/en/product_datasheet?nid=60646: "Weight (Catalog) 26.28 oz", 745 g; 9.84 x 9.0 inch).

TE Connectivity, Raychem, Thomas & Betts, Shrink-Kon and Weller are trademarks of their owners, named for identification only.

## Assets

Every mesh is generated by `task.py` and `mechkit.py`. There are no textures and no third-party assets.

## Licence

MIT

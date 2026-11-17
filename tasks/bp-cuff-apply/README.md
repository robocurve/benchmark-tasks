# Put a blood-pressure cuff on a mannequin's upper arm: wrap it round, close it over the artery mark, then let go

Instruction: "Wrap the blood-pressure cuff round the mannequin's left upper arm with its arrow on the marked line and its lower edge 2 to 4 cm above the elbow crease, fasten it, then let go."

Files in this folder: `task.json`, `bp_cuff_apply.py` (scene, closure rule, scorer, scripted demo), `labkit.py` (multi-shape parts
with joints), `pitch_demo.py` (full-orientation waypoints), `arena_assets_fix.py` (library asset paths), `test_bp_cuff_apply.py`
(offline tests), `README.md`.

## Environment
Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Brev launchable), inspect-robots with the `yam_arena` embodiment, station scene.
Task file SHA-256 (first 12): see the freeze log.

## Offline check (no GPU)
`python -m pytest test_bp_cuff_apply.py` with the yam_arena runtime on the path: the start layout scores 0; each tier (0.3 / 0.6 /
1.0) at its threshold on constructed cuff states; the closure gate (`engages()`) stays open for fingers open on the tab, a closed
fist 10 mm away or only 20 mm of overlap, and closes for a closed fist on 60 mm of lying overlap; the listed shortcuts (nothing, sweep, lift and drop, half a wrap) score 0; the scoring-text numbers appear in the scorer;
`demo()` reads constants only.

## Run and score one Dream-1 trial
```
inspect-robots run --embodiment yam_arena -E task=bp_cuff_apply.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy dream \
  --instruction "Wrap the blood-pressure cuff round the mannequin's left upper arm with its arrow on the marked line and its lower edge 2 to 4 cm above the elbow crease, fasten it, then let go." \
  --scorer task_score --max-steps 740 --seed 0
```

## Scripted demo
```
inspect-robots run --embodiment yam_arena -E task=bp_cuff_apply.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --policy yam_scripted \
  --instruction "Wrap the blood-pressure cuff round the mannequin's left upper arm with its arrow on the marked line and its lower edge 2 to 4 cm above the elbow crease, fasten it, then let go." \
  --scorer task_score --max-steps 740 --seed 0
```
`demo()` reads constants only (scene geometry), never the simulation state. The right hand pinches the curled free end and holds it
on the table (it anchors the cuff); the left hand pinches the curled start end, lifts it up the left-arm side, over the top of the
patient arm and lays it about 30 degrees down the far side, lets go and parks. The right hand then draws the free end up the far
side (the cuff's middle comes up under the arm and round it), over the top, and lets it go over the near side; with its fingers
closed it presses the loop face onto the hook face (the closure engages), smooths it 25 degrees down the near side and parks.
Plan 28.2 s; the run_limit is the plan x 1.3 rounded up to whole seconds (740 steps).

## Scoring
Judged at the end of the trial; the run ends with full credit once 1.0 has held for 1 s (see `task.json`).
- 1.0: closure engaged; one full turn round the arm; snug: no part more than 10 mm off the skin (or off the layer under it); the
  artery arrow faces out and is at most 20 mm (along the skin) from the artery line; the cuff's lower edge 20 to 40 mm above the
  elbow crease; both grippers' tool points and pad tips at least 3 cm (30 mm) from the cuff.
- 0.6: engaged and snug, but the arrow or the edge height is out of range, or a gripper is still on the cuff.
- 0.3: one full turn, no part more than 30 mm off, and not closed or not snug. 0 otherwise.
- The closure engages only when at least 60 mm of the loop face lies within 6 mm of the hook face and a gripper with closed fingers
  (each finger within 1.5 mm of closed) presses on it (pad tip within 4 mm). It opens again on a peel (loop face lifted over
  15 mm with over 4 N) or a shear over 10 N.

The real-trial checks are the clinical ones: tape measure from the crease to the lower edge (20-40 mm, the guideline's 2-3 cm),
ruler from the artery line to the arrow, and the two-finger check ("Two fingers should fit under the cuff", CDC NHANES 2019 Blood
Pressure Procedures Manual p. 3-12, https://wwwn.cdc.gov/nchs/data/nhanes/public/2019/manuals/2019-Blood-Pressure-Procedures-Manual-508.pdf;
lower edge 2-3 cm above the antecubital fossa, arrow over the artery: https://medcrine.com/blood-pressure-measurement), a closure
tug check for the simulated engage rule (release the cuff, hook a spring scale on the free end and pull 4 N outward: the cuff stays
closed), and a photo.

## Runtime per trial
Runtime per trial: 740 control steps at 20 Hz (37 s of simulated time); the scripted demo reaches 1.0 at about 23 s and ends
after it has held for 1 s.

## Seeds
Seeds: 0 for the scripted demo and the Dream-1 trial; object_jitter=0 (the cuff starts on its marks).

## Reset between trials
Peel the cuff open, lay it flat across under the arm with the start end on the left-arm side and the skin face up, fold each end
piece up about 80 deg by hand and hold the fold with one strip of masking tape on its inner side, and set it with its centre line 100 mm above the crease.

## Mechanism and sources
| part | real item | model | precedent |
|---|---|---|---|
| patient arm | full-body male display mannequin with detachable arms, 223.90 USD (https://www.amazon.com/dp/B00IFJ9STC, listing price); upper-arm circumference not published, measured on arrival | fixed tapered upper arm, radius 55 -> 49 mm (325 mm round the middle, within the cuff's 25-34 cm range), elbow, forearm 35 -> 29 mm; artery line and crease ring drawn on | manikin tasks #1912 #1981 #1837 |
| arm props | arm elevation foam wedge, 30 deg, 40.58 USD (https://www.ebay.com/itm/407333981686, opened 2026-11-17) under the forearm, plus any 7.5 cm box under the shoulder end | two fixed blocks; arm axis 100 mm above the table, underside 45-51 mm | -- |
| cuff | adult size-11 reusable cuff, 25-34 cm arm, 24.20 USD + 5.99 shipping (https://mfimedical.com/products/welch-allyn-flexiport-reusable-blood-pressure-cuff, opened 2026-11-17); length 50-55 cm (https://www.promed.ie) | 26 rigid 20 mm segments, 140 mm wide, 3 mm thick, on hinges (bending stiffness 0.002 N m/rad, 0.10 kg; estimates); end pieces propped curled up as a staging aid (sprung 0.012 N m/rad, 1.45 rad; not sourced) | segment chains #7 #1916 |
| hook-and-loop | the cuff's own closure: hook patch on the outer face of the first 220 mm, loop face on the skin face of the last 100 mm | contact-gated, releasable task logic (see Scoring): on engage each hinge is held at its angle by a stiff drive (1 N m/rad) | contact-gated attachment #2126, #1717 #1465 #1862 |

## Real-world build
Parts: mannequin arm 223.90 + cuff 24.20 + 5.99 shipping + wedge 40.58 = 294.67 USD (listed prices, not quotes); the shoulder box
and the pen for the artery line and crease ring are on hand. Detach the left arm from the mannequin, lay it on the wedge and the box
with its axis 100 mm above the table; draw the artery line along the top of the upper arm and the crease ring at the elbow;
measure the upper-arm circumference. Cuff thickness is not published (modelled 3 mm; measured on arrival); the demo keeps the free-end
pull low (it lets the free end go before pressing).

## Fidelity
Isaac Sim 6.1 rigid-body simulation (PhysX, 20 Hz control). The cuff is a chain of rigid segments with weak hinge springs; the
hook-and-loop is task logic that holds the hinges once the loop face is pressed onto the hook face, and the curled end pieces are
sprung hinges. Two staging simplifications: (1) the cuff starts already passed under the arm, as when an aide slides it under a
lying patient's arm; the robot does not pass it behind the arm; (2) each 20 mm end piece is propped curled up (an 83 deg sprung set,
not sourced from a real cuff) so a down grip can pinch it; on the real cuff it is folded up by hand and held by one strip of masking
tape across the fold (reset step), and a real flat cuff is picked up by its edge. Open-loop demo: a few
transition waypoints (lifting off and parking) are out of reach by up to 30 mm in the official IK; every grasp, lay and press pose
is within 5 mm (test_demo_key_poses_reachable). The demo depends on the estimated friction (skin 0.6, cuff 0.7). The bladder, air pressure, fabric stretch and the hook-and-loop's real peel curve are not modelled; the arm is a
rigid mannequin arm (no soft tissue).

## Licence
All shapes are primitives built in `bp_cuff_apply.py`; no third-party assets.

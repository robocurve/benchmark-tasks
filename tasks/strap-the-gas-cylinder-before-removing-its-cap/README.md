# Strap the gas cylinder before removing its cap

A green gas cylinder stands in a dark cradle left of the table's midline, with a loose grey cap on top, a yellow strap
bar drawn open beside it and a yellow tray to its left, nearer the arms. The robot is told to strap the cylinder in,
then take the cap off and put it in the tray. The order is the point: a cylinder standing loose with its cap off is the
dangerous object, so the strap goes across before the cap comes off, and the cylinder stays upright throughout.

The task needs both arms: the right pushes the strap bar across by the tab on its free end (which starts 4 cm right of
the midline) and the left lifts the cap off and sets it in the tray.

## Files

| File | What |
|---|---|
| `strap_the_gas_cylinder_before_removing_its_cap.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | Tools built from several boxes with sliders (the strap bar and its push tab), and per-step hooks |
| `plantkit.py` | A step clock, slider and placement checks, and hand-built states for the tests |
| `test_strap_the_gas_cylinder_before_removing_its_cap.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=strap_the_gas_cylinder_before_removing_its_cap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Secure the cylinder: push the strap bar across the cradle until it stops, then take the cap off the top of the cylinder and put it in the tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/strap-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strap_the_gas_cylinder_before_removing_its_cap.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Secure the cylinder: push the strap bar across the cradle until it stops, then take the cap off the top of the cylinder and put it in the tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/strap-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/strap-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once the strap is across, the cap is in the tray, the
cylinder is standing in the cradle and both have stood still for 1 s; otherwise it scores the last step.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_strap_the_gas_cylinder_before_removing_its_cap.py
```

## Scoring

Scored from the end state, with two latches set during the run. The strap is across when the bar has moved at least 70%
of its 8 cm travel (5.6 cm). The cylinder is standing in the cradle when its axis is within 3 cm of the plate's centre
in any direction, its base under 1.5 cm above the plate and it leans under 20 degrees. The cap is off when, measured in
the cylinder's own frame from the centre of its top, the cap's centre is more than 2 cm off the cylinder's axis or its
underside more than 1 cm above the top; it is in the tray when its centre is inside the collar and its underside under
1.5 cm above the table, however it lies.

| Outcome | Score |
|---|---|
| Strap across, cap in the tray, cylinder standing in the cradle | 1.0 |
| Strap across, cap off but not in the tray (on the table, on the cradle, or still held) | 0.5 |
| Strap across, cap still on (lifted under 1 cm over the cylinder counts as on) | 0.5 |
| Cap came off at any step while the strap was not across, or the cylinder not standing in the cradle | 0 |
| Cylinder leaned more than 20 degrees at any step | 0 |
| Strap not across at the end (never pushed, or pushed and pulled open again) | 0 |
| Cylinder out of the cradle at the end | 0 |
| Nothing done | 0 |

## Design notes

- **The order is the rule.** The step hook latches "unsecured" the moment the cap counts as off while the strap is not
  across or the cylinder is not standing in the cradle, so strapping afterwards earns nothing: 0, below every honest
  partial result. The strap itself earns credit only if it is still across at the end.
- **"Off" is tight, and read in the cylinder's frame.** A cap counts as off once it is 1 cm clear of the top or 2 cm
  off the axis, so it cannot be lifted and parked just over the cylinder while the strap goes across. Measuring from
  the centre of the cylinder's top, in its own frame, means a cap riding on a cylinder that leans 15 degrees against
  the bar has not moved at all, so a lean never latches it.
- **Knocking the cap off scores no better than leaving it on.** Strapped, a cap on the table is 0.5, the same as
  stopping after the strap. Unstrapped, it is the hazard: 0.
- **Tipping is permanent.** Over 20 degrees at any step is 0 whatever the end state. Strapped, the cylinder can lean
  about 15 degrees forward before it meets the bar (1.5 cm away, 6 cm up) and about 6 degrees sideways before it meets
  a post (1 cm away, 10 cm tall), so an honest run cannot trip the latch; with the bar open, the cylinder can fall out
  of the front.
- **Nothing springs back.** The bar has no spring and stays where it is pushed; its joint stop is the "until it stops"
  in the instruction. The score reads the cylinder in the cradle's own frame, so shoving the cradle changes nothing.
- **The push tab keeps the fingers clear.** The right arm pushes a 3 cm tall tab on the bar's free end, 5 cm out from
  the bar, so its fingers stop at y 4 cm: 3.4 cm short of the near side post, 2 cm over the ledge the bar rides on
  (the tab is raised 1.5 cm for that), and never over the plate. The side posts start 3 cm behind the plate's front
  edge, so the bar's channel is clear of them too.
- **Reach, and a deviation from "each arm on its own side".** The tab's free end starts 4 cm right of the midline and
  stops 4 cm left of it, so the right arm's fingers end 4 cm onto the left side, at bar height only; the IK test proves
  the whole push and the lift from there. The left arm waits over the tray until the right arm has parked, and the two
  tool points are never closer than 22 cm.
- **The grasp.** The cap is narrower than the cylinder under it, so the pads close on the cap's upper half with their
  tips 1 cm above the cylinder's top. The lift takes the cap's underside 3.2 cm clear of the top, 2.2 cm past the "off"
  line. The cap is let go from 2 mm over the tray's rim, a 1.2 cm drop, so the fingers never enter the collar.
- **Unintended strategies.** Lifting the cylinder out of the cradle, with or without its cap: 0 at the end, and taking
  the cap off while it is out latches unsecured. Tipping it to shake the cap off: 0. Throwing the cap at the tray:
  counts only if it ends inside the collar. A cap balanced on the collar's rim: not in. Strap pushed across and pulled
  open again: 0. Cap lifted under 1 cm and held over the cylinder: still on, 0.5. Partial push just under 5.6 cm: not
  across, so taking the cap off then is unsecured.
- **The rule behind it.** [OSHA 29 CFR 1910.253(b)(2)](https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.253)
  keeps cylinders where they will not be knocked over (ii) and valve protection caps in place except when the cylinder
  is in use or connected for use (iv); [1910.101(b)](https://www.osha.gov/laws-regs/regulations/standardnumber/1910/1910.101)
  applies CGA P-1, which has cylinders secured upright before the cap comes off.
- **What isn't modelled.** The cap lifts off rather than unscrews, the cylinder is not pressurised and has no valve,
  and a bar on a slider stands in for a chain or strap. The ledge the bar rides on, its legs and foot, the pin-and-slot
  guide and the 1 cm marks a person would read are not drawn: the sim's prismatic joint has the same stops.

## Runtime and seeds

- The scripted demo takes about 23 s of simulated time (458 steps at 20 Hz): the right arm's push, lift and park take
  the first 9 s, the left arm's pick and place the rest.
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.

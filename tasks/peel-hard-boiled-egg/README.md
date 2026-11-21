# Peel a hard-boiled egg intact

Peel one cooked egg and serve it whole. Break the shell with a light tap on a board, start peeling at the exposed edge and support the egg while removing the rest. The separation must preserve the albumen. Two hands share the work of supporting the egg and pulling the shell away. [American Egg Board method](https://www.incredibleegg.org/recipes/cooking-school/how-to-hard-boil-eggs/)

## Physical setup and reset

Use the standard bimanual YAM station with the two bases 610 mm apart and the working surface 747.5 mm above the floor. X is distance forward from the robot bases, Y is positive toward the left arm. In millimetres:

| Item | Specification and location |
| --- | --- |
| Board | 244 × 225 × 12 hardwood cutting board, centered at X 300, Y −50 with sides parallel to the table. |
| Egg | One large hard-boiled chicken egg, 56–60 long, 41–44 wide, 55–64 g in shell. Start with dry intact shell on its side, at X 282, Y −48 on the board. Long axis along Y, blunt end toward negative Y. |
| Serving plate | 154 diameter blue ceramic side plate, centered at X 320, Y 148. Keep it empty, clean, and dry. |

Prepare cooked eggs from the same carton following the referenced method, cool them completely, then bring each to the station dry and uncracked. Fit clean food-contact covers to both pads before the trial. Use a fresh egg and clean the plate, board and both covers after each trial. All equipment touching the food is cold.

## Score

Measure the result by inspection, a ruler and the trial video. Count any visible white torn away through the shell and any puncture that exposes yolk as damage. A dropped or damaged egg scores 0. So does a whole unchanged egg, even if it is moved to the serving plate.

| Score | Outcome |
| --- | --- |
| 0.20 | Shell cracked by tapping the held egg on the board. |
| 0.50 | At least one patch at least 10 mm across peeled free. |
| 0.80 | Entire intact white exposed, with no visible remaining shell fragment at least 2 mm across. |
| 1.00 | The peeled egg rests on the blue plate and all removed shell is off that plate. The arms stay 66 mm clear of the egg center for one continuous second. |

The hulls, membrane bonds and egg rest on native contacts. The modeled shell cracks into a blunt cap, an irregular central peel section and a pointed cap. Each is a thin curved rigid shell coupled to the cooked white by a native breakable joint with a force and torque limit. Motion and loading are continuous and the material approximation discretizes fragmentation and adhesion. The physical score is based on the resulting shell and egg, viewed after release.

## Run

Use Isaac Lab-Arena, install this one task with `python -m pip install -e . --no-deps`, and run the native trials. The control frequency is 20 Hz, physics uses 240 Hz, the action limit is 820 and the demonstration seed is 1415158438.

```bash
python run.py --device cuda:0 --physics-device cpu --output /tmp/peel-egg-demo
python run.py --device cpu --no-cameras --policy hold --output /tmp/peel-egg-idle
python run.py --device cpu --no-cameras --policy tap-only --output /tmp/peel-egg-tap-only
```

Inspect Robots embodiment is `peel_egg` and the scripted policy is `peel_egg_scripted`. Task solids are designed in `peel_egg/builder.py` and are released under the task's MIT license. The YAM robot and calibrated wrist cameras come from the pinned YAM adapter. [YAM license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

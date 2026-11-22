# Separate and stage nested stainless skillets

## Impact on jobs

Food Preparation Workers (O*NET 35-2021.00; 908,500 employees in 2025, BLS data per O*NET OnLine) "Carry food supplies, equipment, and utensils to and from storage and work areas" (task importance 87) and "Receive and store food supplies, equipment, and utensils in refrigerators, cupboards, and other storage areas" (74). https://www.onetonline.org/link/summary/35-2021.00
Skillets are stored nested to save shelf space, so getting a set ready on the prep station starts by separating the stack: lift the smaller pan off, draw the larger one out from under it and set each down flat in its own place.

On a crowded prep counter, the mat may be the only clean landing surface for hot or oiled pans. The nested pair covers the smaller skillet’s staging spot at the start. Pulling the bottom skillet first drags the pair, risks tipping the smaller skillet or scrapes it against the rim. The scripted demo lifts the smaller skillet level by its riveted handle with the left hand and keeps it aloft while the right hand extracts the larger skillet onto its staging spot, then rests the smaller skillet level in the cleared spot (one arm in sequence also works, see below). The two single-object shortcuts leave either the pans nested or the lower skillet short of its staging spot.

## Instruction

Release both skillets upright and flat in their matching marked spots on the mat.

## What is not readily accessible here

The closest rejected idea separated nested bowls; its review read: "Separating the readily accessible upper bowl does not add a sufficiently distinct physical skill." [#2214](https://github.com/robocurve/benchmark-tasks/pull/2214)
In this task the upper skillet is the easy part. The skill is getting the lower skillet out from under it. Measured from the task geometry:

| Quantity | Value |
|---|---|
| Lower skillet well depth (inside, sole top to rim) | 39.5 mm |
| Share of the upper bowl's 42 mm height that sits below the lower rim | 39.5 mm (94%) |
| Radial gap between the two bowls | 20 mm at the soles, about 24 mm at the lower rim |
| Upper skillet mass resting on the lower sole | 0.80 kg |
| Handle bars above the mat (upper / lower) | 57.5 mm / 54 mm, 84 degrees apart on the near side |
| Overlap of the nested pair with the upper skillet's staging spot | the spots are 107 mm apart, less than the 101.5 mm upper rim radius plus the 127 mm lower rim radius |

The lower skillet cannot be lifted, tilted or slid without carrying or scraping the 0.8 kg pan in its well. It has to be drawn out by its 310 mm handle only once the upper pan is clear, and set down within 16 mm of a spot. Moving only one skillet scores 0.0, and the offline shortcut runs (do nothing, sweep across the mat, lift and drop each skillet, stop the demo halfway, undo it) all score 0.0.

## One arm or two

The scripted demo holds the smaller skillet aloft with one arm while the other arm extracts the larger one. One arm can also do the task in sequence: set the smaller skillet down clear of both spots on the mat, move the larger one, then move the smaller one to its spot. The scorer accepts either order. Two-arm coordination is not required for credit.

## How this differs from #2214

#2214 (nested bowls, not accepted) separated two light stacked bowls and laid out place settings: the upper bowl was readily accessible and the rest was ordinary pick-and-place. The numbers below for this task come from the scripted demo replayed in offline MuJoCo (score 1.0 at step 387).

| | #2214 | This task |
|---|---|---|
| Objects | Two stacked bowls, two spoons | A 0.80 kg skillet nested in a 1.10 kg skillet, handles 84° apart |
| Where each item is held | By the bowl | Only by its handle bar, 138 mm (upper) and 163 mm (lower) from the bowl center |
| Load at the grip | Small | The pan hangs off the grip: center of mass 89 mm (upper) and 118 mm (lower) from the gripper, peak gravity moment 0.70 N·m and 1.27 N·m |
| Separation | Lift the upper bowl off | The left hand lifts the upper skillet 107 mm clear and keeps it aloft while the right hand lifts the 1.10 kg lower skillet by its handle and carries it to its spot. After the upper skillet leaves, the two pans never touch (zero contact force in every step) |
| Placement | Bowl centered on a mat | Each center within 16 mm of its spot, tilt at most 3.0°, held 2.0 s, handles inside the mat |

The skill is holding and placing heavy pans level by their handles, against a moment of up to 1.27 N·m at the grip, in an order that keeps the 0.8 kg pan off the larger one. Moving only one skillet scores 0.0 in the offline tests.

## Nearest tasks

- The saucepan task slides one pan onto a burner and turns the handle within the counter edge. Here two hollow pans start nested and finish separately supported on the mat. [#721](https://github.com/robocurve/benchmark-tasks/pull/721)
- The pancake task steadies a frying pan by its handle while a turner inverts a pancake. Here both cookware bodies move, and credit depends on separation and placement. [#1435](https://github.com/robocurve/benchmark-tasks/pull/1435)
- The removable-handle task captures a saucepan rim with a detachable jaw, lifts the pan and releases it on a trivet. Both skillets here have permanent riveted handles. The difficult contact is clearing the lower rim while extracting the lower pan. [#2042](https://github.com/robocurve/benchmark-tasks/pull/2042)

## Bench setup

Place the 609.6 × 457.2 mm mat with its long side along Y. Positive Y is toward the left-hand side. Positive X is away from the front of the bench. Use the mat center as the ruler origin. The centers are specified relative to it.

| Position | X (mm) | Y (mm) |
|---|---:|---:|
| Initial nested center | 4 | 151 |
| Upper skillet staging center | 105 | 186 |
| Lower skillet staging center | −55 | 2 |

| Mechanism | Construction |
|---|---|
| Upper skillet | 203 mm rim, 155 mm sole, 42 mm bowl wall, 0.80 kg |
| Lower skillet | 254 mm rim, 195 mm sole, 46 mm bowl wall, 1.10 kg |
| Handles | Riveted steel bars with the smaller pointing near-left and the larger pointing near-right, 84° apart |
| Support | Silicone mat lying on the bench |

Nest the smaller skillet flat inside the larger one and begin with both grippers in their ready poses. The skillets have full-depth wells and free rigid-body motion. There are zero mechanical joints or springs.

## Simplifications

| Simplification | Real value it stands in for | Source | Effect |
|---|---|---|---|
| Skillet masses 0.80 and 1.10 kg | Individual masses are not published; the set ships at 4.41 lb (2.00 kg) including packaging | Pan listing (Real trial hardware) | Sets the grip moments above (0.70 and 1.27 N·m); the pair can weigh at most 2.00 kg |
| One friction value, 0.5, for pan on pan and pan on mat (simulator default material) | Steel on steel, clean and dry: static 0.5 to 0.8, kinetic 0.42. No published silicone-on-steel value | https://www.engineeringtoolbox.com/friction-coefficients-d_778.html | Offline runs with friction scaled by 0.7, 0.8, 1.25, 1.3 and 1.5 hold the same seated pose ([DEVIATIONS.md](DEVIATIONS.md)) |
| Collision shapes: flat stepped soles and 2.7 mm sloped wall facets | Smooth spun stainless bowls | Estimate ([DEVIATIONS.md](DEVIATIONS.md)) | Faceted wall contact; seated offsets stay within 0.01 mm of the mat top |
| Handles: straight rigid bars 24 × 16 mm ending 267 and 310 mm from the bowl center | Curved, tapered riveted handles (dimensions not published) | Estimate | Fixes the grip points and the levers above |
| Wall 2.7 mm, soles 5.5 and 6.5 mm | Laminated stainless walls and soles (not published) | Estimate | Sets the nesting gap (20 mm at the soles) and the well depth (39.5 mm) |
| Mat: rigid 1.4 mm slab with the listed 24 × 18 in footprint | Compliant silicone on the bench | Mat listing | No sinking; seating allows −0.5 to +3.0 mm |
| Cool, dry pans | Hot or oiled cookware | — | Oil lowers steel friction (0.15 to 0.23 lubricated, same table); not modeled |

## Scoring

| Criterion | Simulation | Real trial |
|---|---|---|
| Flat and upright | Tilt at most 3.0°; lowest collider point and sole-center origin −0.5 to +3.0 mm relative to the mat top | Check sole contact with a 3 mm gauge and level; soles must not sink into the mat |
| Mat boundaries | Each skillet, including its handle, at least 10 mm inside the mat | Use a ruler on an overhead view |
| Separation | Centers at least 235 mm apart; at least 2 mm of horizontal clearance between bowls, shoulders and grips | Measure center to center and check the closest gap from overhead |
| Lower skillet progress | At least 65 mm along the staging line and within 50 mm crosswise | Measure the lower center against the line from the nested center to its spot |
| Separate spots | Each center within 16 mm of its staging center | Measure from the center of each circular rim |
| Gripper clearance | Both tool centers at least 45 mm from each skillet | Measure each gripper midpoint to the nearest skillet surface |
| Success hold | Full-credit final geometry for 2.0 continuous seconds | Time the supported, released pose for 2.0 seconds |
| Whole-run latches (every control step) | Unheld skillet (no gripper within 40 mm of its handle bar) tilted over 20°, or its lowest point over 20 mm above its support; a skillet off the mat or 10 mm below its top; either skillet faster than 1.0 m/s; lower skillet over 30 mm from its start while the upper is still nested | Watch the video: the run fails if a pan tips or drops when released, leaves the mat, is flung, or the nested pair is dragged |

The trial scores 0.0 if any whole-run latch fires, or if support, boundaries, separation or lower-skillet progress fails. It scores 0.4 if those checks pass and either center is outside its spot. It scores 0.7 when both centers are inside their spots before the clearance hold completes. The success timer starts when the clearance is met and resets when it is lost. Completing the timer earns 1.0.

## Real trial hardware

| Item | Listed size | Quantity | Cost |
|---|---|---:|---:|
| Two-piece uncoated stainless-steel skillet set | 8 in and 10 in rim diameters | 1 | $43.00 |
| White and turquoise silicone baking mat | 24 × 18 in | 1 | $7.99 |
| Custom parts | None | 0 | $0.00 |
| **Total** | | | **$50.99** |

Both listings returned HTTP 200 on 2026-11-19. The mat’s published product data reports `InStock` and $7.99. The pan listing gives the exact set and packaged weight, and the public storefront product and fulfillment responses returned HTTP 200, $43.00 and `IN_STOCK` for shipping and Niles, Ohio pickup. [DEVIATIONS.md](DEVIATIONS.md)

https://www.target.com/p/2pk-8-34-38-10-34-uncoated-stainless-steel-frying-pan-set-figmint-8482/-/A-94307810

https://www.ikea.com/us/en/p/baktradition-baking-mat-white-turquoise-80577861/

## Run a trial

From the package directory, use the official Arena environment.

```sh
inspect-robots run --embodiment yam_arena -E task=nested_stainless_skillets_lift_upper_extract_bottom.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted --scorer task_score --max-steps 600 --instruction "Release both skillets upright and flat in their matching marked spots on the mat."
```

```sh
inspect-robots run --embodiment yam_arena -E task=nested_stainless_skillets_lift_upper_extract_bottom.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --scorer task_score --max-steps 600 --instruction "Release both skillets upright and flat in their matching marked spots on the mat."
```

Runtime per trial is capped at 30 seconds of simulation time, or 600 steps at 20 Hz. Wall time depends on the runtime and policy.

Demo results: the scripted demo scored 1.0 in the GPU simulator (388 steps, success) and 1.0 in offline MuJoCo. Dream-1 scored 0.0 (600 steps, time limit).

Seeds: fixed initial layout for official resets (`object_jitter=0`).

## Offline tests

With Arena on the import path, run `python -B -m pytest -q -p no:cacheprovider`. The suite exercises the scoring tiers, missing gripper observations, unsupported poses, and the two parked-arm replays when the offline physics runner is available.

Rebuild the brushed-steel and silicone assets with Blender 5.2.3 and the Arena checkout path in `ARENA_CHECKOUT`.

```sh
blender -b --factory-startup -t 4 --python-exit-code 1 --python assets/build_visuals.py -- "$ARENA_CHECKOUT"
```

## Licence

| Path | Origin | Licence |
|---|---|---|
| assets/* | Original modeled meshes and procedural textures | MIT |
| *.py, task.json, README.md, DEVIATIONS.md | Task, tests, documentation and bundled Arena helpers | MIT |

[LICENSE](LICENSE)

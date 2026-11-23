# Shell an edamame pod into a serving bowl

**Societal relatability.** Preparing edamame requires opening a flexible pod without scattering the beans, transferring the beans to a serving bowl, and clearing the shell as food waste.

## Pod source and preparation

Use Seapoint Farms plain frozen Edamame in Pods, 14 oz (397 g). If unavailable, use Seapoint Farms plain Organic Soybeans in Pods, 12 oz (340 g), or another frozen, plain, whole-pod product whose package allows the same boil preparation. Apply the same specimen checks to an equivalent. Photograph the package and record brand, size and lot or best-by code for each batch. [Publix product and cooking directions](https://www.publix.com/pd/seapoint-farms-edamame-in-pods/RIO-PCI-164643) · [Seapoint Farms frozen products](https://salesinfo.seapointfarms.com/frozen/)

1. Keep the pods frozen until cooking. Bring 6 US cups (about 1.4 L) of potable water to a rolling boil per 14 oz (397 g) of pods. Add the frozen pods directly. Start a timer when the water returns to a rolling boil and cook for **5 minutes**, the upper end of the listed 4–5 minutes. Use plain water, with no salt, oil or seasoning. Record boil restart and drain times. [Published package directions](https://www.publix.com/pd/seapoint-farms-edamame-in-pods/RIO-PCI-164643)
2. Drain in a clean colander, rinse briefly under cold potable running water, and drain for 60 seconds. Use clean tongs to transfer pods in one layer to shallow food-safe containers. Refrigerate unused pods promptly in a covered container at **4°C or below**, verified with a refrigerator thermometer. Use the batch within 8 hours of cooking and discard the remainder. Remove only the selected pod from cold storage. Warm it on a clean, dry dish under a cover at room temperature. Do not reheat it or put a staged pod back with unused pods. [USDA cooling and storage guidance](https://ask.fsis.usda.gov/article/How-do-I-handle-leftovers-safely)
3. Measure the selected pod just before placement using a clean, dry contact surface thermometer accurate to ±1°C. Verify the thermometer against a reference before the session. Touch the center of each broad face until the reading stabilizes, without puncturing or crushing the pod. Record both readings and use the pod only when both are **20–25°C**. Blot visible surface water with a clean, lint-free sheet before placement. Begin within 10 minutes of the temperature check and within 30 minutes of removing the pod from refrigeration. Limit cumulative time out of refrigeration after cooking to 60 minutes, including draining, selection, staging and the trial. Discard specimens that exceed either staging limit. Handle soy as an allergen, keep food prep separate from the robot work area, and discard all robot-handled food. [USDA food handling guidance](https://ask.fsis.usda.gov/article/How-do-I-keep-my-foods-safe-to-eat)

## Specimen selection

Choose an intact, unopened pod with exactly three distinct, filled bean bulges. Reject pods with an ambiguous count, splits, damaged seams, loose beans or visibly overcooked tissue such as mushy, collapsed or disintegrating walls. Use clean calipers with light contact. Measure straight from the stem shoulder to the tip, excluding the free stem, then measure the maximum profile width and maximum thickness across the broad faces. Record the three measurements to the nearest millimetre.

Use **60–72 mm length, 14–20 mm width and 10–16 mm thickness** as proposed tolerances around the simulated **66 × 17 × 14 mm** pod. Published sizes vary. Iowa State describes a quality pod length around 64 mm, while Virginia Tech reported cultivar means below these target ranges. The retail listing gives package mass, and does not specify pod dimensions or three-bean yield. Before scoring a new lot, screen up to 20 intact pods or the whole bag if smaller, and record how many meet the bean-count and all three size criteria. If yield is insufficient for the planned trials, try another lot or qualifying equivalent and record the shortfall. Commercial-lot fit and the proposed protocol have not been physically validated. [Iowa State size reference](https://www.extension.iastate.edu/organicag/files/documents/edamamenk02.pdf) · [Virginia Tech cultivar measurements](https://doi.org/10.1002/plr2.20140)

## Physical setup, reset and pad care

Use the stock two-arm YAM tabletop station with overhead and wrist cameras, a 185 mm ceramic plate, a 156 mm ceramic serving bowl and an open square 165 mm plastic food-waste caddy about 133 mm high. The tabletop is at Z = 0.7475 m. In station coordinates (metres), center the plate at (0.357, −0.052), the bowl at (0.445, 0.124), and the caddy at (0.515, −0.222).

1. Stop motion and follow the station's safe isolation procedure before reaching into the workspace. Remove all previous specimens and debris. Empty, clean and dry the plate, serving bowl and scraps caddy. Wipe and dry the work surface, then restore the three vessels to their specified positions.
2. Protect shared pads by using a dedicated food-trial set of the same stock pads, with the same material, geometry and surface finish. Use the stock contact faces with no coatings, sleeves, adhesives or grip aids. Fit and inspect pads per the station procedure. Blot every pod before contact. After each trial, with the station safely isolated, wipe the pads with a dry lint-free cloth and inspect for moisture, residue, damage or a loose pad. For residue that a dry wipe cannot remove, remove the pad under the approved maintenance procedure and use only a cleaning method approved for that pad material. Keep liquid away from the installed gripper and electronics. Refit only clean, fully dry pads. Stop trials if the pads cannot be restored to that condition.
3. Place one new qualifying pod at the plate center, long axis along X and stem toward −X. Rest it on its broad side with the seam intact. Take an overhead reset photo showing orientation and the empty receiving vessels. Never reuse a manipulated pod.
4. Clear the workspace and restore both arms to the benchmark ready pose using the station's safe reset procedure. In the benchmark joint convention, each arm starts at `[0, 1, 1, -0.6, 0, 0]` rad in joint order 1–6, with the jaws open. Use the station's calibrated mapping for each physical arm. Verify the reset and start the trial within the staging limits above.

### Trial preparation checklist

- [ ] Record package, lot and cook, drain and storage times. Verify ≤4°C cold storage.
- [ ] Select an unopened three-bean pod, measure all three dimensions and confirm both faces are 20–25°C.
- [ ] Blot the pod, check clean and dry stock pads, reset all vessels and photograph placement.
- [ ] Verify both arms are ready and the workspace is clear. Record trial start and end, then discard the specimen.

### Physical trial record

| Record | Entry |
| --- | --- |
| Trial ID, date/time zone, operator | |
| Product, package size, lot or best-by code, package photo | |
| Boil restart, drain, refrigeration start, storage temperature | |
| Pods screened, three-bean pods passing all dimensions | |
| Specimen ID, length × width × thickness (mm), three bulges, seam and condition | |
| Cold-storage removal, temperature check time, readings on both faces (°C) | |
| Pad type and condition, cleanup, ready pose verified, setup photo | |
| Trial start and end, staging time, disposition, score, deviations | |

## Scoring

Use the overhead and wrist views to confirm robot contact on each half, opening and bean removal. Verify the final bean count by direct inspection or from the overhead view. Use a clock or video frames to verify the hold. Every positive tier requires meeting the opening criterion. Award the highest tier met.

| Score | Required outcome |
| --- | --- |
| 0.00 | The pod stays closed or the trial invalidates. |
| 0.25 | The robot handles both halves, exposes the interior along the seam, and opens the halves by more than 35°. |
| 0.50 | One extracted bean is wholly inside the serving bowl. |
| 0.70 | Two extracted beans are wholly inside the serving bowl. |
| 0.85 | All three extracted beans are wholly inside the serving bowl. |
| 1.00 | All three beans remain in the bowl, the empty shell is within the caddy, and the final state holds with both arms clear for at least one second. |

A bean escaping while the pod remains closed, or any pod section or bean leaving the tabletop work area, invalidates the trial. The simulator checks the three independent cohesive seam links as the opening witness. It counts beans only when the whole width fits inside the bowl boundary. Shell retention uses the four physical shell links, their motion, clearance from both arms and contact reports.

## Run the trial

Run these commands from this task folder with the provided Isaac Lab-Arena environment and stock YAM embodiment installed.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
uv pip install --python "$VIRTUAL_ENV/bin/python" --no-build-isolation --no-deps -e .
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
export EDAMAME_JOINED_SHELL=1 EDAMAME_PLATE_RECOVERY_LAYOUT=1

EDAMAME_LIFT_FIRST=1 EDAMAME_END_GRASPS=1 EDAMAME_POUR_AFTER_LIFT=0 \
  EDAMAME_ANGLED_APPROACH=0 EDAMAME_PLATE_RECOVERY=1 EDAMAME_MEASURE_ARM_CONTACT=1 \
  EDAMAME_GRIP_FORCE_LIMIT_N=3 EDAMAME_RELEASE_ABOVE=1 \
  inspect-robots run --embodiment yam_arena --policy edamame_scripted --scorer task_score \
  -P hold_s=2 -P oracle=true -P plate_transfer=true \
  -E task="$PWD/arena_task.py" -E headless=true -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --seed 0 --max-steps 2000 --no-prompt --store-frames \
  --log-dir /tmp/edamame-expert

inspect-robots run --embodiment yam_arena --policy dream --scorer task_score \
  -E task="$PWD/arena_task.py" -E headless=true -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --seed 0 --max-steps 2000 --no-prompt --store-frames \
  --log-dir /tmp/edamame-dream
```

The run limit is 2,000 actions, or 100 s at 20 Hz. The expert reads shell and bean poses, seam states and fingertip loads, and sends stock YAM joint and gripper commands. It opens the pod over the plate, collects each bean, regrips the empty shell, and lifts it above the caddy rim before opening the jaws. The first full-score run reached 1.00 in 1,502 actions with zero monitored inter-arm contacts. Seeds and evaluation scores are recorded in `task.json`.

## Simulation fidelity and attribution

Adjacent convex sectors approximate the continuous curved wall. The three upper sections bend about compliant hinges. Equal and opposite cohesive forces act on the seam until measured deformation breaks each bond. The three beans are separate rigid bodies. Material stiffness, seam forces, bend limits and friction are development estimates. A physical reproduction should record the opening and bean-extrusion forces on several pods from the selected batch.

The pod, beans and containers are procedural task geometry. Run `python edamame_geometry.py` to regenerate the USDs. This folder uses the stock YAM and camera models from the installed embodiment and does not redistribute them.

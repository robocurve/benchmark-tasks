# Set a drill-press depth stop and prove the depth twice

Hold the orange catch open and slide the blue depth stop to the green 10 mm mark. Release the catch, take both hands off the press, then press and release the yellow test button twice, waiting for the spindle to return to the top each time.

Machinists set a physical downfeed limit so they can drill a series of blind holes without breaking through the workpiece. Quick-adjust depth stops allow the operator to release a catch, slide the stop, and lock it again. This trainer tests that setup operation and whether the same stop arrests the spindle at the intended depth under two different loads. The nearest submissions use a selected spacer to set shaft end play or lock a measured valve gap. The outcome here is a repeatable downfeed limit under load, with the probe returning to its home position between strokes. [Powermatic drill-press manual, §8.4](https://www.purdue.edu/bidc/wp-content/uploads/2022/07/Drill-Press_manual-2.pdf) · [Shaft end play PR #347](https://github.com/robocurve/benchmark-tasks/pull/347) · [Valve lash PR #361](https://github.com/robocurve/benchmark-tasks/pull/361)

## Fixed setup and reset

Use the calibrated bimanual YAM station with its 747.5 mm worktop. Coordinates in the table are millimetres from the midpoint between the arm bases. X points forward, Y points toward the left arm, and heights are above the table. All parts start unrotated. The compound parts and local offsets in `task.py` specify the full scene geometry and starting configuration.

| Assembly | Construction and start |
| --- | --- |
| Frame and workpiece | Fixed aluminium base, 203 × 285 × 18 mm, centred at X/Y 322/−15. Column 36 × 63 × 152 mm with a 44 × 85 × 35 mm head and a 38 × 91 × 60 mm catch support. The aluminium workpiece is two 73 × 35 × 21 mm cheeks forming a 19 mm through slot, with the reference surface 39 mm above the table. |
| Spindle | Steel spindle, 160 g in simulation. The 48 × 46 × 12 mm feed pad is centred at X/Y 300/−85, height 152 mm. The 16 × 16 × 102 mm blunt probe starts with its tip 5 mm above the reference surface. A 26 × 70 × 8 mm strike plate contacts the stop. A vertical guide gives 25 mm of travel and a spring returns the spindle. |
| Depth stop and scale | Blue polymer carrier and handle, 100 g with the steel rack. The 53 × 66 × 10 mm handle starts at X/Y 246/−16, height 102 mm. Its shoulder starts 7 mm below the strike plate. The five steel teeth are 11 × 24 × 1.3 mm on a 4 mm pitch. The initial detent stops at 2 mm below the workpiece face. The 28 × 22 × 100 mm fixed scale has a green witness mark at the third detent, which gives 10 mm depth. |
| Catch | Captive spring-return catch, 80 g, with a 128 mm bar, 14 × 22 × 1.2 mm steel pawl and 30 × 42 × 18 mm orange polymer paddle at X/Y 354/153, height 113.5 mm. Push the paddle away from the arm bases to disengage the pawl. Its horizontal guide allows 11 mm of travel. |
| Test switch | Yellow polymer cap, 49 × 48 × 10 mm, centred at X/Y 248/−244, height 55 mm, over a fixed polymer pedestal. It has 10 mm of spring-return travel and triggers after 5 mm of depression. |

Reset by letting the spindle and button rise to their upper rests, opening the catch, and lifting the blue stop back to the first detent. Release the catch and reset the test sequence to 8 N. The setup is fixed, with no object jitter and no consumables. Use the initial top camera frame as the setup image.

## Proof and scoring

Each press from home initiates one downfeed stroke. The first applies 8 N axially to the spindle, and the second applies 12 N. The force ramps over 0.5 s, holds to the 1 s mark, then releases. The spindle must settle on the intended shoulder for the last 0.25 s of the load and return to within 2 mm of home by the 2 s mark. The button must be released and the catch must stay engaged. Keep both hands clear of the press during each proof. Invalid strokes may be retried. Changing the stop or opening the catch discards prior proof readings.

- **1.0** if the shallowest settled depth and deepest observed depth of both strokes are 9–11 mm, inclusive, and the two settled readings differ by at most 1 mm.
- **0.5** if both strokes meet the same proof conditions and stay within the broader 5–15 mm depth window during settling, without exceeding 15 mm, and the trial does not qualify for 1.0.
- **0** initially and otherwise. A hand holding the spindle, a jam away from the stop shoulder, an open catch, or a modified or damaged part cannot qualify.

Depth is tip travel minus the 5 mm initial clearance to the workpiece surface. The simulator checks that the strike plate and stop shoulder are within 1 mm, spindle speed is at most 3 mm/s in the settled interval, and the pawl is within 2 mm of its engaged position. Both gripper TCPs must remain outside X 180–470 mm, Y −193 to 231 mm, below height 210 mm during proof. The yellow button is outside that region.

For a physical reproduction, use an unpowered fixture with the blunt probe, then apply the two loads to the spindle with a spring scale after the robot releases it. The evaluator can apply the loads manually in place of the yellow button's automatic test sequence. Film the strike-plate contact and read the probe depth against a millimetre scale referenced to the coupon surface. Use the same fixture for both strokes and confirm the spindle returns between them.

## Run and score on Isaac Lab-Arena

From the task directory with the program's Isaac Lab-Arena environment active:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  width=320; height=176
  if [ "$policy" = yam_scripted ]; then width=640; height=360; fi
  python run.py run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width="$width" -E cam_height="$height" \
    --instruction "$instruction" --scorer task_score \
    --max-steps 600 --seed 0 --no-prompt --store-frames --log-dir "runs/$policy"
done
python -m pytest -q test_scoring.py
python probe.py
python export_media.py --demo runs/yam_scripted --dream runs/dream --out media
```

Each trial permits 600 actions (30 simulated seconds at 20 Hz). Use new log directories when repeating a trial. Arena records the final `task_score`. The fixture also writes a `proof-1.jsonl` trace with the gripper, catch and spindle states, the applied load and the verified depths. The exporter uses the recorded camera frames and adds a readout from the trace to the videos.

`run.py` uses PhysX rigid contact and prismatic constraints for the fixture, with spring drives for return and a bounded force for each test stroke. The stop height comes from contact between the spindle, stop rack and catch. The simulated bodies are rigid and the guide joints replace manufactured bearings. The trainer represents depth setup, with no motor, cutting or modeled wear. The scripted policy sends ordinary YAM joint targets through Arena.

## Validation

The scripted YAM trial on the L40S scored **1.0** in **420 actions** (21.0 simulated seconds, 78.3 s wall time). After both arms withdrew from the fixture, the 8 N stroke stopped at 10.09 mm and the 12 N stroke at 10.15 mm. The spindle returned to its upper rest after each stroke.

The Dream-1 trial on the same layout and seed scored **0.0** at the **600-action** limit. It did not complete a proof stroke.

The three commissioning controls preset the stop at reset and apply the same simulated loads. The nominal 2, 6 and 10 mm detents scored 0.0, 0.5 and 1.0, respectively. These controls isolate the mechanical proof from the robot's setup motion. Twelve offline tests passed, including checks for unsupported travel, an open catch, assistance, overshoot, and changes to the setting after proof. The validation runs use simulation. The physical reproduction procedure has yet to be tested on hardware.

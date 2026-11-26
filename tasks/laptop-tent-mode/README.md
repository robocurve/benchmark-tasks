# Turn a 2-in-1 laptop into a tent display

**Societal relatability.** Set up the open laptop for watching a video or following a recipe. Fold the display past flat, put the two front case edges on the desk with the hinge above them and the screen facing out, and release both hands. The desk can support the laptop between grasps.

## Reference laptop

The reference is the metal edition of the 13.3-inch Lenovo Yoga 6 13ALC7. Lenovo specifies a 304 × 218 × 17.45 mm closed envelope, a starting mass of 1.37 kg, a 360° hinge, an aluminum upper case, and a PC-ABS lower case. Its user guide shows tent mode for watching movies. The requested Lenovo Support article describes the screen-rotation setting for ThinkPad Yoga computers. The model-specific Yoga 6 guide describes automatic rotation in tent mode. [Lenovo PSREF](https://psref.lenovo.com/Product/Yoga/Yoga_6_13ALC7), [Yoga 6 modes](https://download.lenovo.com/pccbbs/pubs/yoga_6_13alc7/html_en/EN/feature_intro_yoga_modes.html), [Lenovo Support HT500431](https://support.lenovo.com/in/en/solutions/HT500431), [Yoga 6 screen rotation](https://download.lenovo.com/pccbbs/pubs/yoga_6_13alc7/html_en/EN/work_with_windows.html)

| Item | Model geometry and material |
| --- | --- |
| Chassis | 304 mm wide, 208 mm from shell front to shell rear and 11.20 mm thick. The chassis, key bed and palm rest move as one 0.94 kg rigid link. Four 15 × 18 × 1.4 mm rubber feet are fixed to it. |
| Display | 304 × 208 × 5.70 mm shell, 0.43 kg. Glass, bezel, aluminum back and case edge move as one rigid link. The active graphic is 286 × 179 mm. |
| Hinge | One passive revolute joint, centered between the two visible hinge barrels. The distance from the common axis to either front edge is 214 mm. The rear of the hinge hardware is 4 mm behind the axis. |
| Grip surfaces | The left hand pinches the chassis side lip about 80 mm from the hinge. The right hand pinches the display side frame about 158 mm from the hinge. Real grip surfaces are the solid rim around the keyboard deck and the display’s bezel and back shell. Lenovo specifies a 2.8 mm side bezel. The modeled rigid surface shares a wider jaw footprint. [Lenovo datasheet](https://psref.lenovo.com/syspool/Sys/PDF/datasheet/Yoga%206%2013ALC7_datasheet_EN.pdf) |

The individual masses, shell thicknesses and centers of mass are estimates. The display's center of mass is modeled 110 mm from the hinge. Its maximum gravity moment is therefore about 0.46 N·m, using 0.43 kg × 9.81 m/s² × 0.110 m. Lenovo shows settings in which the screen holds its angle against gravity. The simulation uses an estimated static breakaway torque of 0.65 N·m, about 1.4 times that maximum, plus 0.52 N·m sliding resistance and 0.04 N·m·s/rad viscous resistance. These values are inferred from the model's dimensions and intended use. They have no bench measurement behind them. The shell uses static/dynamic friction of 0.55/0.42 and the rubber feet use 0.86/0.66 before combining with the desk material. [Lenovo mode illustrations](https://download.lenovo.com/pccbbs/pubs/yoga_6_13alc7/html_en/EN/feature_intro_yoga_modes.html)

## Reset and controls

Coordinates are in millimetres, with x forward, y left and z up. The 800 × 800 × 40 mm wood worktop spans x = −50 to 750 and y = −667 to 133, with the top at z = 750. The arm bases are at (0, 520, 750) and (0, −420, 750). The left base is on a fixed 180 × 180 × 40 mm steel mounting plate, with its top at desk height. The scene also has a floor and dome light. There are no other workpieces.

The chassis starts horizontal on all four feet. Its front edge is at x = 146 and the hinge is at (360, 0, 762.6). The lid starts 112° open, the keyboard faces the arms, and the left case rim overhangs the side of the desk by 19 mm. The feet end at y = ±131 mm, leaving the near feet on the worktop. The overhang exposes the underside of the case for the initial pinch. The arms start in the YAM ready pose with the grippers open. The fixed oblique side camera uses the standard `top_cam` slot. Its eye is at (605, 665, 1055) and it points at (347, −40, 839). The wrist camera names and joint action layout are the standard YAM ones. [Setup screenshot](setup.png)

The demo pinches the display and chassis side rims, opens the lid partway with the chassis braced on the desk, and releases and regrips the display to change the wrist roll. Hinge friction holds the intermediate screen angle. The arms lift the assembly, move it inboard, fold past flat, form the tent, and lower the two edges before withdrawing. The policy computes Cartesian targets from the declared setup geometry with YAM inverse kinematics. Public `joint_pos` is used for the initial tool poses via forward kinematics and for tracking corrections. Physics poses, hinge positions and contact forces are read by the scorer and trace recorder. Those readings are never passed to the controller.

## Scoring

At each 120 Hz physics step, the scorer checks the following conditions.

- The hinge is at 238–304° from closed and at least 125 mm higher than the front edges. The two edge midpoints are 100–360 mm apart in plan. Both corners of each edge are on the desk footprint and within 4 mm of the top. The screen normal points outward along the line from the chassis edge to the display edge with a horizontal projection of at least 0.55, and its upward component is at least 0.15.
- Upward desk reactions in the frontmost 5 mm of each shell carry at least 15% of the laptop weight per edge and at least 90% combined. The total normal force from robot links is below 0.05 N. Contacts elsewhere on the desk or with the surroundings must also total below 0.05 N.
- Both links move at no more than 8 mm/s and spin at no more than 0.10 rad/s.

The score is 0.5 after 0.5 continuous seconds and 1.0 after 2 continuous seconds. Losing any condition clears the hold and returns the current score to zero. A passing full hold terminates the trial. A side photo can check the angle, ridge height, edge separation and screen direction. Desk contact and stability can be checked with a thin paper strip at each edge and a two-second observation after release. The simulator uses contact forces to distinguish edges that carry load from edges that merely appear close to the desk. [Scorer](scoring.py), [Physics recorder](physics_monitor.py)

## Run locally

Use the pinned environment in the workspace `RUN_LOCAL.md`. The USD ships with the package. `python build_asset.py` regenerates it in the activated environment. The task is run at 20 Hz with six PhysX substeps per action and a fixed reset. From the laptop worktree root, choose an available physical GPU index and run the following.

```bash
workspace=$(cd ../.. && pwd)
task="$PWD/tasks/laptop-tent-mode"
runs=$(mktemp -d "$workspace/runs/laptop-tent-mode-XXXXXX")
gpu=1
instruction=$(jq -r .instruction "$task/task.json")

for mode in normal flat held; do
    out="$runs/$mode"
    mkdir -p "$out"
    LAPTOP_TENT_CONTROL="$mode" LAPTOP_TENT_TRACE="$out/trace.jsonl" \
        "$workspace/scripts/run_on_gpu.sh" "$gpu" inspect-robots run \
        --embodiment yam_arena --policy yam_scripted --scorer task_score \
        -E "task=$task/task.py" -E control_hz=20 -E object_jitter=0 \
        -E cam_width=640 -E cam_height=360 \
        --instruction "$instruction" --seed 0 --max-steps 800 --store-frames \
        --no-prompt --no-rerun --no-live-log --log-dir "$out"
done

source "$workspace/env.sh"
python "$task/verify.py" --normal "$runs/normal" --flat "$runs/flat" \
    --held "$runs/held" --out "$runs/results.json"
(cd "$task" && python -m unittest -v test_scoring)
inspect-robots video "$runs"/normal/adhoc_*.json --out "$runs/video"
```

The run limit is 800 actions, or 40 simulated seconds. The controls use the same physics, reset, and scorer. `flat` unfolds the display while the chassis rests on its feet. `held` stops with the laptop in the tent geometry while the grippers still carry load. [Task entry point](task.py), [Scripted controller](demo.py), [Trial verifier](verify.py)

## Measured results

The three scripted runs used seed 0, Isaac Lab-Arena with GPU PhysX, the YAM adapter at `0059d02`, and the standard `task_score` scorer. They finished within the original 750-action cap with zero evaluation errors. [Measurements and source hashes](evidence/results.json)

| Trial | Score | Actions | Simulated time | Evaluator wall time | Final hinge angle |
| --- | ---: | ---: | ---: | ---: | ---: |
| Dream-1 | 0.0 | 800 | 40.00 s | 56.47 s | 112.00° |
| Bimanual YAM demo | 1.0 | 706 | 35.30 s | 59.78 s | 270.64° |
| Flat control | 0.0 | 519 | 25.95 s | 42.99 s | 181.70° |
| Held tent control | 0.0 | 684 | 34.20 s | 56.80 s | 274.40° |

The Dream-1 trial used the standard `dream` policy (`dreamlabs/dream-1-beta`), one epoch at seed 0, and 320 × 176 observations at 20 Hz. It reached the 800-action cap with zero evaluation errors. The hinge stayed at 112° and the recorder measured zero robot contact force across all 4,800 physics samples. The recording shows the fixed side view above the two wrist views. [Model recording](evidence/model_run.mp4), [Dream-1 measurements](evidence/dream-1.json)

The normal trial crossed 180° at action 502. At action 613, before release, the chassis and display front edges carried 5.53 N and 6.68 N. The final tent stood 158.3 mm high and held for 2.008 s without robot contact. The final edge loads were 8.01 N and 5.43 N. [Side-camera demo](evidence/demo.mp4), [Final tent frame](evidence/tent.png)

The flat control finished released with its display front edge at desk height and the ridge 11.2 mm above the two front edges. The held control finished in the tent geometry with both edges at desk height. The desk carried only 4.52 N of the 13.44 N laptop weight, and the grippers were still in contact. The remaining weight was borne by the robot. [Flat control frame](evidence/flat.png), [Held control frame](evidence/held.png)

## Fidelity

The simulation represents both panels as rigid shells with box colliders and a shared fixed-axis friction hinge. The modeled travel is 0.5–359.5° from closed. A real Yoga uses paired hinge assemblies and offsets the panels in the closed and fully back-folded stacks. One fixed axis gives a common transform a full revolution apart. The neighboring shells have interlink collision disabled to approximate that travel, and the scored pose stays away from both stops. The visual keys, bezel, hinge covers and screen graphic are simplified and non-colliding. Chassis flex, glass damage, wiring and screen auto-rotation are outside the model. The arm mount spacing and side overhang are deliberate access choices for the YAM grippers.

## Live duplicate check

The signed-in board refresh at 15:38:20 UTC on 22 November 2026 found 90 accepted and 220 non-final rejected tasks reviewed in the previous 48 hours. The read also screened 358 pending tasks. The motion comparison found zero equivalents. [Board check](evidence/board-check.json), [RoboCurve task board](https://tasks.robocurve.org/board)

TK-J987L9 was reviewed at 13:05:21 UTC on 20 November, just over 50 hours before the refresh. It carries an open laptop from books to a separate stand, caps lid movement at 25°, and bars touching the lid. The convertible trial rotates its hinge from typing through flat and transfers the weight onto the two case rims. [TK-J987L9](https://tasks.robocurve.org/board?task=TK-J987L9)

The accepted birthday card and tablet tasks are older mechanical analogues. The card shares the final edge-supported shape and uses a springy paper crease. The tablet unfolds a dedicated back support to stand in portrait orientation. [Birthday card task](https://tasks.robocurve.org/board?task=TK-LSCM4T), [Tablet task](https://tasks.robocurve.org/board?task=TK-Y8KTZZ)

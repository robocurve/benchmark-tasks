# Fit a split cable guide and verify drawer travel

Fit a split guide around the cable at the panel opening and lock it with the orange keeper. Keep both plugs seated throughout. Withdraw both hands, then press the yellow release and let the drawer open to the marked line on its own.

The task models restoring edge protection on wiring that feeds a service drawer. The installed guide must surround the cable and protect the opening while leaving enough clearance for the thicker sleeve to travel. The unpowered apparatus evaluates installation and travel clearance, drawing on commercial split cable entries and guides for moving drawers. [Split cable entries](https://www.icotek.com/en-us/products/cable-grommets) · [Guides for moving drawers](https://www.evoline.com/en/products/smartdrawers/wing/)

- PR #8 feeds and hands off a free plug through a fixed grommet. Here the cable is already routed and connected, and the removable guide must be retained while the drawer moves. [PR #8](https://github.com/robocurve/benchmark-tasks/pull/8)
- PR #179 assembles a split spacer around a stationary captive shaft. Here the sleeve must physically pass through the fitted guide during a subsequent stroke. [PR #179](https://github.com/robocurve/benchmark-tasks/pull/179)
- PR #347 measures and reduces rigid shaft end play with a spacer. Here the robot preserves the full service stroke of a connected cable. The installed guide may block the thicker sleeve despite fitting around the cable at rest. [PR #347](https://github.com/robocurve/benchmark-tasks/pull/347)

## Setup and reset

Use the calibrated bimanual YAM station with a table height of 747.5 mm. Both inserts start on the left arm’s side, and the release is on the right arm’s side. Coordinates are measured from the midpoint of the arm bases. X points forward, Y points toward the left arm, and Z points up. All positions and dimensions in `task.py` are in metres. Its `OBJECTS`, `SHAPES` and `JOINTS` describe every fixture, collider, joint, and initial pose. Coordinate values describe body origins. A shape's dimensions and offset are listed relative to its body. All bodies start unrotated except the cable links, whose yaw follows the slack loop.

| Part | Construction and start |
|---|---|
| Panel receiver | Aluminium web, 66 × 10 × 52 mm, with a 61 × 33 mm sill, side rails and end stops. The cable axis crosses it at x = 255 mm, y = 0, 55 mm above the tabletop. The receiver's sill closes the open side of the fitted insert. |
| Two guide inserts | PETG training guides, 36 g each, 44 mm wide × 22 mm deep with grip tabs. The clear widths are 16 mm (blue tab) and 10 mm (black tab). The inserts start upright at x/y = 247/285 mm and 365/205 mm, with both leg tips on the table and the openings facing down. |
| Keeper and release | Orange polymer sliding keeper with a 53 mm stroke from y = −53 to 0 mm. Its bar overlaps a guide shoulder when closed. The yellow polymer release tab is 52 × 45 × 11 mm, at x/y = 245/−311 mm above a fixed post, and has 10 mm of vertical travel. Press more than 5 mm to release. |
| Drawer | Aluminium training carriage, 140 g, with a 65 × 55 × 27 mm base on a linear rail. It starts centred at x/y = 255/153 mm and stops 50 mm farther along +Y. The white pointer reaches the green mark at the open stop. |
| Cable and sockets | 8 mm black PVC jacket, blue sleeve 12 mm in diameter and 12 mm long, and a U-shaped slack loop on a flat tray. Sleeve centre starts 24 mm on the fixed side of the panel. Two 23 × 15 × 18 mm dummy plugs seat in sockets on the fixed post and on the drawer. |

At reset, place both inserts on their start marks, open the keeper, return the drawer to the closed stop, seat both plugs, and set the cable loop and sleeve to their start marks. Use the same fixture and object poses each trial. Nothing is consumed. The photo should show both inserts, both sockets, the loop, the keeper, the release and the travel mark. Use contrasting witness marks and a fixed side view to resolve any plug or sleeve gap the overview hides.

## Score

The guide is seated if its body origin is within 2 mm in X/Z and 2.5 mm in Y of the receiving pose and orientation error is at most 4°. The keeper is closed when it is within 2 mm of its closed stop. The grippers must withdraw from the fixture. The yellow release tab is outside the monitored fixture region. The region covers x = 182–328 mm and y = −105 to 258 mm over the stroke, and x = 105–285 mm and y = −251 to −78 mm over the slack loop, from the table up to 150 mm above it.

- **1.0** if the guide is seated and locked before release, the drawer starts within 2 mm of closed and reaches at least 48 mm of travel within 3 s, the entire blue sleeve passes the far face of the guide, and this state is held without assistance for 2 s.
- **0.5** if the guide is seated and locked with hands clear, and the trial does not qualify for 1.0.
- **0** otherwise. A plug backing out more than 2 mm, a guide falling below the bench, the guide or keeper coming loose during proof, or either gripper entering the protected region during proof makes the trial score permanently zero. The release is single-use per trial. Read the physical score with the guide, keeper, plug and stroke witness marks and the recorded video. In a physical trial, a broken or modified part or a fixture shifted off its taped footprint also scores zero.

## Run on Isaac Lab-Arena

Run from this task folder with the program's Isaac Lab-Arena and `yam-isaac-lab-arena` environment activated. `run.py` supplies the compound collision shapes and fixture joints, so use it as the entry point for both policies.

```bash
instruction='Fit a split guide around the cable at the panel opening and lock it with the orange keeper. Keep both plugs seated throughout. Withdraw both hands, then press the yellow release and let the drawer open to the marked line on its own.'
for policy in yam_scripted dream; do
  python run.py run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$instruction" --scorer task_score --seed 0 --max-steps 800 \
    --no-prompt --store-frames --log-dir "runs/$policy"
done
```

For a larger setup image, record the scripted trial at 640 × 360 while keeping Dream-1 at 320 × 176. Use new log directories for repeated trials.

The limit is 800 actions (40 simulated seconds). The recorded L40S trials took 130 s for the scripted policy and 231 s for Dream-1 including simulator and inference overhead. The scripted controller sends normal 14-component YAM joint targets. Arena writes scores to its trial log and per-step fixture state to `proof-1.jsonl`. Run `python -m pytest -q test_task.py` to test scoring. Run `python probe.py` to test wide, narrow and missing guides with the same physical drive across resets. The probe presets the guides and keeper to isolate the collision test.

The cable's travel section is rigid in simulation and its slack loop uses short, jointed rigid links. The drawer spring and release latch are force models applied to a constrained physical carriage. The plugs use compliant joints to measure withdrawal. Contact between the sleeve, guides and fixture determines travel. The scoring region uses measured gripper TCP positions as a proxy for assistance. The bench does not model jacket wear, connector electrical behaviour or material fracture.

## Recorded validation

Seed 0 on the calibrated Isaac Lab-Arena YAM station, 20 Hz control. The scripted demo scored 1.0 in 463 steps (23.15 simulated seconds). The drawer traveled 50.0 mm, reached the mark 0.25 s after release, and held for 2.0 s. The largest plug displacement was 0.871 mm. Dream-1 completed the 800-step limit with a score of 0.0. The preset physics controls scored 1.0 with the wide guide, 0.5 with the narrow guide (9.8 mm travel), and 0.0 with the guide absent (50.0 mm travel). The five offline scoring tests passed. Additional YAM trials with the narrow guide failed to seat it, so narrow-guide travel is verified from a preset installation. Physical YAM hardware remains untested.

## Review media

```bash
python export_media.py --demo runs/yam_scripted --dream runs/dream --out media
```

The exporter checks the task revision and trial scores, writes H.264 MP4s and the initial top-camera screenshot, and decodes both videos to validate them. Upload `model_run.mp4`, `demo.mp4` and `setup.png` with the repository's `tools/upload_media.py` to populate the media ID in `task.json`.

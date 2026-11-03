# Take the diagonal cutters off a crowded pegboard

Taking a hand tool off a pegboard, the way a technician does many times a day. Twelve real-scale NVIDIA SimReady hand
tools of six kinds hang across a 90 cm board in front of both arms: pliers, cutters, screwdrivers, a wrench and a
hammer, on pegs at different heights, with a screwdriver hanging right above the cutters. Each peg ends in an upturned
tip, so the cutters have to be lifted over the tip before they can slide off. Pulled straight out, they catch on the
tip and stay on the board. The neighbouring tools must stay hung.

| File | What it is |
|---|---|
| `cutters_off_pegboard.py` | The `yam_arena` task file: scene, scoring (`success`, `score`) and the scripted `demo()` |
| `simready.py` | Loads SimReady props into a task file (`SimReady(...)`), downloading them once from Hugging Face |
| `wrist.py` | `TiltWaypoint`: scripted-demo waypoints whose grip tilts forward, to reach into a vertical board |
| `test_cutters_off_pegboard.py` | Offline checks: the file loads, every scoring outcome, the demo's order and reach (no GPU, no network) |
| `task.json` | The task's details for the board |

## Assets

The twelve tools come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`handTool_pliers_diagonal_cutting_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_diagonal_cutting_a01)
- [`handTool_pliers_combination_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_combination_a01)
- [`handTool_pliers_long_nose_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_long_nose_a01)
- [`handTool_pliers_long_nose_curved_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_long_nose_curved_a01)
- [`handTool_pliers_groove_joint_10in_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_groove_joint_10in_b01)
- [`handTool_screwdriver_phillips_ph1_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph1_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)
- [`handTool_screwdriver_slotted_55mm_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_slotted_55mm_a01)
- [`handTool_screwdriver_torx_t10_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_torx_t10_a01)
- [`handTool_screwdriver_torx_t20_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_torx_t20_a01)
- [`handTool_wrench_combination_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_wrench_combination_a01)
- [`tool_hammer_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tool_hammer_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
- the pliers' two handles are hinged in the files. The hinge is removed and the handles become one rigid body.
- the tools are turned to hang: pliers jaws up, screwdrivers blade down, the wrench by its ring end, the hammer head up.
- masses are set to typical values: cutters 200 g, combination pliers 250 g, long-nose pliers 180 g each, groove-joint
  pliers 300 g, wrench 150 g, hammer 450 g, and screwdrivers 60 g (15 cm) or 100 g (19 cm).
- each tool's spin is damped (angular damping 2 /s), standing in for the friction a real tool has on its pegs. PhysX
  has no rolling friction, so a round handle resting on two pegs would otherwise keep turning.
- the invisible grasp-annotation curves are deactivated.

The pegboard, pegs and mat are `yam_arena` boxes. The files (about 500 MB, mostly textures) are not committed.
`simready.py` downloads them into `~/.cache/simready` the first time the scene is built, which needs network
access, as Arena's own library assets do.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=cutters_off_pegboard.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the diagonal cutters off the pegboard and lay them on the mat." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/cutters-off-pegboard-dream

# Scripted demo (right arm), 400 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cutters_off_pegboard.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the diagonal cutters off the pegboard and lay them on the mat." --scorer task_score \
  --max-steps 400 --store-frames --log-dir ~/yam-eval/cutters-off-pegboard-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed.
- The demo (right arm) grips the cutters' jaws from the front, lifts them 3 cm, pulls them 8 cm clear, turns the grip
  down so they swing flat, and lays them on the mat. It reaches 1.0 and ends on success at step 231 (11.6 s at
  20 Hz). A trial takes about 55 s on an L40S, including startup.
- The same demo without the lift (`LIFT = 0.0`) scores 0: the cutters catch on the peg's tip and stay on the board.
- Videos: `inspect-robots video ~/yam-eval/cutters-off-pegboard-demo/<log>.json` writes one MP4 per camera next to
  the frames.
- Offline checks: `python -m pytest -q test_cutters_off_pegboard.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left, from the midpoint of the
arm bases), and heights are above the table top. The cutters' peg is 15 cm up and every peg ends at x = 0.42.

- **Fallen:** a tool's centre more than 5 cm below where it hangs (`HANGING_Z`, measured in the simulator after the
  scene settles: 13.5 to 35.5 cm above the table).
- **0** if any tool has fallen off the table (centre more than 2 cm below the top) or the state is not finite.
- **1.0 (success, held 1 s):** the cutters' centre over the mat (24 x 14 cm, centred at (0.20, -0.26)) and less than
  3 cm above the table, the cutters still (under 1 cm/s and 0.5 rad/s), and no other tool fallen.
- **0.5:** the same, with exactly one other tool fallen. Two or more fallen score 0.
- **0.25:** the cutters off their peg (centre nearer the robot than x = 0.40), held or set down off the mat, and no
  other tool fallen.
- **0** otherwise, including doing nothing, lifting the cutters without taking them off, or dropping them under the
  board.

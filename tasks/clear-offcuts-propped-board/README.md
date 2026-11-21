# Swap the scrap prop for a bearer under a pine batten before clearing the offcuts

A task for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene. The batten rests on one proper pine bearer and one plywood offcut being used as a prop. The task is to clear both offcuts while leaving the batten supported by the proper bearer pair. Any manipulation sequence is valid if the batten never touches the bench and ends on both bearers.

## Importance and sources

- The U.S. Bureau of Labor Statistics reports 889,700 carpenter jobs in 2025. Carpenters cut and shape wood, work on many construction projects, and often lift and move materials ([BLS Occupational Outlook Handbook](https://www.bls.gov/ooh/construction-and-extraction/carpenters.htm)).
- OSHA requires scrap lumber and other debris to be cleared from construction work areas, and combustible scrap to be removed at regular intervals ([29 CFR 1926.25](https://www.osha.gov/laws-regs/regulations/standardnumber/1926/1926.25)).
- The sources establish carpentry and cleanup duties, not how often a scrap prop supports a finished workpiece. This is a task scenario, not a claim about prevalence. The 36 cm batten is at real size and mass; the task does not represent handling full-length stock.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/clear-offcuts-propped-board/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the dark plywood offcuts in the bin and leave the pine batten level on the two pale bearers." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clear-offcuts-dream
```

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/clear-offcuts-propped-board/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Put the dark plywood offcuts in the bin and leave the pine batten level on the two pale bearers." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/clear-offcuts-demo
```

## Scoring and validation

Any contact between the batten and the bench at any point latches the score at 0. Otherwise each released offcut resting inside the bin earns 0.25. If both offcuts are in the bin and every support condition is met, score 0.75 until the complete state holds continuously for 3 seconds, then score 1.0. Each bearer must be flat, crosswise within 5 degrees, overlap the batten by at least 3 cm along both its width and length, and meet its underside within 2 mm. The bearers must be released; their centres must straddle the batten midpoint and be at least 15 cm apart. The score uses a side video and ruler for the 2 mm contact-gap check; the remaining orientation, overlap, location and release conditions are visible in the setup or overhead video. Doing nothing scores 0. Other outcomes score 0.25 or 0.5.

The trial limit is 2,400 control steps (2 minutes at 20 Hz). The scripted demo is deterministic with `object_jitter=0`; it reaches the full state in 1,018 steps (51 seconds simulated). Typical wall time on one RTX PRO 6000 is about 2 minutes per scripted demo and 4 to 5 minutes per Dream-1 run, including startup. The score does not measure finish damage: table contact is a directly observable interruption of the support condition. No manipulation order is required; the demonstrated policy lifts the batten while replacing its support, but another sequence is valid if it preserves the scored support condition.

The batten, bearers, offcuts and bin are rigid boxes. The mug, tape and screwdriver are NVIDIA SimReady assets from [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd), revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

| Object | Package |
| --- | --- |
| Mug | `dishware_mug_a01` |
| Tape roll | `tape_clear_a01` |
| Screwdriver | `handTool_screwdriver_phillips_ph2_a01` |

At load time each prop's mass is set to the value in `task.json` with its authored inertia scaled to match; invisible grasp-guide curves are deactivated, and the screwdriver uses convex-hull collision. Wood friction is set above typical real timber, and no force sensing is modeled. Gripper release is measured from fingertip-point clearance. This simulation does not model wood flex, surface damage, or full-length stock handling.

The helper modules `mechkit.py` and `simready.py` provide gripper fingertip positions, per-trial reset hooks, and the three distractor props.

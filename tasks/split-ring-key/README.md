# Thread a household key onto a split ring

Use the two grippers to open a split ring, pass the key hole over the free end, wind the spiral through the hole, and release the key on both turns. The bench vise holds one strand. Its crank and spring-return splitter are operated by hand or gripper. The key, ring and fixture move through contact.

## Reproduce the setup

Buy the **RS PRO 866-579** nickel-plated carbon-steel split ring. Its catalog OD is 38 mm, ID 33.9 mm, wire size 1.98 mm and thickness 3.7 mm. The simulation represents round wire in two touching turns with a short crossover. It uses 340° of overlap, 34.04 mm ID and a 3.96 mm stack. Match the lot-specific cut end and verify the opening fit when building the hardware. [RS datasheet, page 2](https://docs.rs-online.com/0117/A700000007926496.pdf) · [Fixture parts, cost and assembly](fixture/BUILD.md) · [Dimensioned drawings](fixture/fixture-drawings.pdf) · [STEP assembly](fixture/fixture.step)

At reset the ring centre is at world (0.400, 0, 0.9875) m, 240 mm above the table. The green rear tip points along +Y. The red front tip is 20° above it. The jaw grips only the rear turn. The 47.5 × 18 × 2 mm brass key with a 7 mm hole starts flat at hole centre (0.375, −0.140, 0.9675) m. An unpowered four-grip wheel has an 85 mm crank radius. The splitter grip has 140.5 mm radial travel and up to 8 mm axial lift. [Reset views](evidence/setup.png)

The 0.06 mm nose approaches the outside cusp between the touching round wires. It thickens to 1.40 mm over 5 mm. The guide points 45° above +Y, placing the catch 7.9 mm along the near turn behind the red tip. This leaves the tip clear for the key bow. The wide shoe supports the rear wire with 0.10 mm nominal clearance at rest. Push the tapered catch in, lift it to open the free end, and pass the key hole over the tip. Withdraw the catch before turning the crank. [Opening section](fixture/fixture-drawings.pdf)

## Grade a trial from the video

Record a front view square to the ring and an edge view at the marked free ends, each with 1 mm graduations at the working depth. Use an oblique view for the hole and an overview for release. At reset, note the two outer diameters along the vise axes. After the attempt, align the images to the same vise axes, park the splitter, and clear the grippers. Compare the relaxed shape once the key settles. A ring carrying a retained key hangs under the key's own weight. Ordinary opening under the splitter is part of the task.

All credit requires a visible passage of the marked tip through the real hole consistent with the remaining strands. Follow the tips from one face of the 2 mm bow to the other. Undoing a passage removes it. Count wires through the hole and measure the straight-line distance from its centre to the closer marked tip, using the front and edge views if the two features are at different depths.

| Final observed state | Score |
|---|---:|
| Any damage condition below, failed release/inspection, no verified passage or no qualifying capture | 0 |
| One retained crossing with nearest tip less than 10 mm away | 0.25 |
| One retained crossing with nearest tip at least 10 mm away | 0.50 |
| Two overlapping turns crossing in the same direction, at least 12 mm from both tips | 0.75 |
| The preceding two-turn state persists while the released key hangs more than 100 mm above the tabletop for two continuous seconds | 1.00 |

**Damage overrides every tier.** In the unloaded views, either cut end visibly separated from its mating turn by more than 0.50 mm, a visible wire break admitting more than 0.25 mm of daylight, or a change greater than 1.00 mm in either measured OD or in roundness earns zero. Here roundness is the absolute difference between the two marked diameters. The diameter and roundness limits are relative to the starting ring. The tip gap and break limits are absolute. These are provisional inspection bands; the build sheet specifies the repeats on an intact sample needed to check them.

For the inspection, both grippers and fingers must be at least 20 mm from the key and ring. The lift must be less than 0.70 mm from home and the radial slide within 5 mm of park. Settled means key translation ≤15 mm/s, key rotation ≤0.35 rad/s and wheel rotation ≤0.03 rad/s. A hand may steady the wheel away from the key. Two accepted crossings are one winding apart (290–430° along the spiral). A pair closer than 12 mm to an end earns zero. Grade at the end of the trial; the native scorer awards full success only after the uninterrupted hold.

## Run

Use the pinned YAM adapter at `0059d02`, Arena at `c7edf3a`, Isaac Lab at `91a0906`, Isaac Sim 6.1 and Python 3.12. The examples run from the repository root in its activated environment. On the designated workstation, prefix simulator commands with `scripts/run_on_gpu.sh 0` from the RoboCurve workspace. The commands select CPU physics and GPU-rendered cameras. Every output path must be fresh.

```bash
python tasks/split-ring-key/build_assets.py
python tasks/split-ring-key/test_scoring.py
python tasks/split-ring-key/demo.py --variant direct --device cpu --out /tmp/ring-direct --record-stride 2
python tasks/split-ring-key/demo.py --variant full --device cpu --out /tmp/ring-full --record-stride 2
python tasks/split-ring-key/demo.py --variant partial --device cpu --out /tmp/ring-partial --record-stride 2
python tasks/split-ring-key/demo.py --variant wrong_hole --device cpu --out /tmp/ring-miss --record-stride 2
```

Seed 0, fixed layout, 20 Hz control, 1,000 Hz physics, maximum 2,400 actions (120 simulated seconds). With a stride of two the video contains one frame per 0.10 simulated seconds. Each trial saves the native camera recordings, actions, measured state trace and result. [Revision outcomes](VALIDATION.md)

The controller uses the declared geometry, elapsed action index and robot IK. Scoring and recording read object poses independently. Contact geometry comprises 20 sprung wire links with curved capsule spans and a finite-thickness polygonal hole. The simulator checks the aperture through the plate thickness. The retained-strand check allows 0.16 mm of numerical rim contact, equal to the combined contact offsets of the wire and brass colliders. A marked-tip passage still allows only 0.05 mm through both faces. The elastic bending model uses sprung joints and zero rest offset. Metal yielding and fracture require physical validation. Transient angles and joint distances are recorded as diagnostics.

For an authorized model evaluation, use the sim's native scoring path from the same environment.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/split-ring-key/task.py -E object_jitter=0 -E device=cpu \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Thread the brass house key fully onto the steel split ring. Use the orange spring-return splitter and unpowered crank as needed. Release the key and splitter, with both overlapping turns retained and both grippers clear of the key and ring. You may keep holding the crank." \
  --scorer task_score --max-steps 2400 --seed 0 --epochs 1 \
  --store-frames --no-prompt --fail-on-error 1 --log-dir /tmp/ring-dream
```

Dream-1 returned zero model actions on the revised fixture. The service rejected the first inference request with `DA-1000 UNAUTHORIZED (disabled)`. [Evaluation record](DREAM_EVALUATION.json) · [Prior overlap review](OVERLAP_REVIEW.json)

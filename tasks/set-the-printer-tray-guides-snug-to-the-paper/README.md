# Set a printer paper tray's guides snug to a fresh A4 stack

Refilling a household or office printer is a familiar chore. Paper put in crooked, or guides left at another paper size, can feed poorly. This task uses a custom cassette with loose A4 paper, linked width guides and a rear guide. The robot must square the paper at the feed edge, bring all guides against it without excessive pad compression, and release the latches and paper.

The apparatus is the original fabrication design in [fabrication/README.md](fabrication/README.md), not an obtainable OEM cassette or an HP replacement part. Its sprung guide faces make excessive pressure observable as displacement. The package supplies dimensioned drawings, editable manufacturing profiles, materials, fasteners, finished spring specifications, assembly and calibration procedures. It is an unbuilt engineering design; no physical performance or calibration measurements are claimed.

Idea credit: [PR 1781](https://github.com/robocurve/benchmark-tasks/pull/1781), TK-WGL4TM. The category is **Societal relatability**. This implementation uses four loose paper groups, squeeze-to-release guide latches and a reproducible custom apparatus. No code or assets from that submission are used.

## Apparatus and coordinates

Coordinates below are millimetres relative to the table: x forward from the arm bases, y left, z upward. The paper path centre is x=270; the inside feed face is y=-160. Fabrication coordinates are U=x-270, V=y+160 and Z=z-35. The 35 mm enclosed plinth accommodates shafts, racks and pawls; the 3 mm floor has its underside at z=35 and paper surface at z=38. Floor footprint is 304×417, with a 292×396 opening inside the walls. The handle adds 10 mm beyond the front footprint. The front panel ends at z=100; side/back walls at z=65.

| Component | Specification |
|---|---|
| Width guide faces | Start x=161 and 379, 218 apart; span y=-70..30; each travels 70 inward, linked 1:1 |
| Rear guide face | Starts y=196, 356 from feed face; span x=200..340; travels 150 toward the feed edge |
| Pads | Two 100×32.5×3, one 140×32.5×3; faces z=38.5..71; 5 mm compression travel |
| Cartridges | 30 deep behind each free face; backplates recessed 18..22; two guided pins and two springs per pad; no fixed stock bypasses the pad |
| Compression springs | Combined target 1.500 N/mm per pad, nominal 3 N at 2 mm; finished custom spring specifications and load tests in fabrication/SPRINGS.md |
| Latches | Near width and rear guide: 30 wide post/lever, top z=97, hinge z=71; squeeze releases near 0.10 rad; stop at 0.17 rad |
| Positive locks | 0.5 mm pitch square-tooth racks; physical engagement must be verified, including a small settling slide if the pawl lands on a crest |
| Paper | Four separate loose groups of 25 A4 80 g/m² sheets; each model group 210×297×2.5, 125 g; no handles, bindings or rigid physical substitute |

The latch bridge, pin clearances and scale shelves are included in the moving collision/release envelope. Internal linkage dynamics and small hardware use declared numerical or conservative solid-envelope approximations. Pivot bores, stop slots and coil deformation are not contact-solved; the hinge and guide laws represent their function. The rod-eye/stem use a conservative swept envelope attached to the carrier. All exposed surfaces remain in collision and hand-release checks; see fabrication/MISMATCHES.md. The base must be clamped to the table before a physical trial; no clamp may enter the opening or robot path.

## Reset lots

Offsets are set at the leading-edge **midpoint before yaw**, not the nearest rotated corner. Align each group's centre to x=270 plus the lateral offset, place its leading midpoint the listed distance behind y=-160, then rotate about the group centre. Groups are listed bottom to top. The simulation starts layers at z=39.25+i×2.7; the 0.2 mm initial gaps settle under gravity. Actual unbound groups rest on one another.

| Lot | Feed offsets | Lateral offsets | Yaw, degrees |
|---|---|---|---|
| A | 3, 20, 37, 54 | +0.8, -0.6, +0.5, -0.8 | +0.4, -0.5, +0.7, -0.3 |
| B | 54, 37, 20, 3 | -0.8, +0.6, -0.5, +0.8 | -0.4, +0.5, -0.7, +0.3 |

1. Inspect the fixed cassette, clean dry pads and unobstructed guide travel. Complete fabrication/CALIBRATION.md before using a newly built or adjusted mechanism. Remove damaged or curled sheets.
2. Squeeze both latches; set free width faces 218 apart and rear face 356 from the feed edge. Release and verify both pawls are seated. Confirm all three compression pointers read unloaded zero.
3. Count four groups of 25 sheets. Place the selected lot using removable rulers/angle templates; remove all setup aids before the trial. Keep calibration and paper stock the same for both lots.
4. After a trial, remove paper and return all guides and pointers to the same datums. Do not change springs, tooth phase or drag between lots. Photograph/record any altered or damaged mechanism before repeating.

## Mechanism and numerical approximations

The physical construction has guided sprung pads, square-tooth pawls lifted by squeeze levers, opposed racks/pinion for width, and adjustable split bushes for sliding drag. All finished dimensions, load requirements, fits and sources are in the fabrication package. Spring rates, masses, friction and torque values are design estimates or acceptance targets, never measured results.

The simulation uses a spring hinge (0.4 N·m/rad), unlocked guide damping of 30 N·s/m, and a locked drive of 50,000 N/m with 60 N·s/m damping. A released lever alone does not certify engagement: the guide must reach an aligned tooth window, or pass a tooth between two 20 Hz checks made with the lever released, in which case the pawl holds at that tooth as a real pawl would catch it. The far width guide follows the near coordinate through a one-way active PD drive. Far-guide resistance and inertia are not reflected into the near guide; gear backlash and passive contact are absent. A 3 N static far load gives 0.06 mm drive error, while each 20 Hz target update advances 1 mm at 20 mm/s before dynamics. No transient tracking bound is claimed; native diagnostics must report actual tracking error. Physical backlash target is ≤0.2 mm and must be inspected over the full stroke. Actual end gaps are measured independently, so backlash does not enlarge the 1 mm score limit.

The physical split-bush target is 0.60±0.10 N at 20 mm/s for the whole released guide/linkage. It is dry friction. The numerical viscous force is 0.6/1.5/3.0 N at 20/50/100 mm/s, differing from nominal 0.6 N dry drag by 0/0.9/2.4 N. No matched force-speed curve or prediction of hardware success is claimed. Measure force at all three speeds, both directions, before any hardware/model comparison. Numerical carriage masses are 110/70/120 g, pads 13/13/16 g and levers 7 g; these are stock/hardware estimates pending weighing. Pad damping 1 N·s/m, paper friction 0.45/0.35 and guide material friction 0.35/0.30 are uncalibrated.

Four rigid groups approximate sheet sliding; individual sheet buckling, curl, feed transport and print quality are absent. Compression over 2 mm is an explicit excessive-squeeze proxy, not a measured onset of paper bowing. Real trials retain loose paper and the same displacement outcome; do not assert that 3 N is a universal bowing threshold. The two lots change the initial fan/skew, but share target settings; replay remains possible.

## Scoring and real inspection

Scores are evaluated at the end and averaged over lots A and B. Every tier requires:

- Readable finite paper, joint, link, hand and hold state; both pawls engaged and both levers released.
- Every gripper/fingertip face more than 5 mm from every moving guide component and paper for a full second. Face samples are spaced about 3 mm, including hull corners. Unknown hands give zero.
- Motion below 5 mm/s for the paper centres and guide/pad/latch displacement (latch angle converted at 26 mm radius); each paper group's linear speed below 5 mm/s and angular speed below 0.1 rad/s, throughout that second.
- All four groups inside the cassette and above its floor. Each layer must rest on the floor or the preceding layer in actual height order, with its centre supported by the layer below. Actual contacting surface separation must be between -0.25 and +1 mm; collapsed/unsupported groups fail. Identity order is unrestricted.
- Paper tilt ≤2°, highest corner ≤11.5 mm above the floor, no pad compression >2 mm, and no paper/guide or feed/wall penetration beyond the 0.25 mm numerical allowance.

Then award **0.5** for rear-guide contact within 1 mm, every group's yaw within 1°, and every leading edge within 1 mm of the feed face. Independently award **0.5** only when every group is within 1 mm of each width guide over its finite contact span. Oppositely fanned groups touching opposite guides do not earn this tier. A genuine width-only or rear/squaring-only result can score 0.5. Contact is measured over each guide's actual 100/140 mm span, not a remote corner beyond the guide. For width, each group must approach both guides within 1 mm at some point of their actual spans. The rear pad must approach the stack within 1 mm; the whole face need not have zero gap.

On hardware, inspect all groups, support, tilt, alignment, latch engagement, release and settled video. Measure guide contact gaps with a 1 mm feeler within the stated contact spans; do not use a corner outside a guide. Read each calibrated direct displacement scale; its 2 mm red mark is a boundary, not a force sensor. For compression reading d with measured uncertainty u, certify the ≤2 mm gate only when d+u≤2 mm. Otherwise use a finer independently calibrated indicator before awarding credit. Apply the same conservative upper-bound procedure at gap/angle boundaries. A missing or ambiguous measurement cannot certify that tier. This procedure does not widen the simulated thresholds.

## Running and evidence

```
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/set-the-printer-tray-guides-snug-to-the-paper/task.py \
  -E object_jitter=0 -E control_hz=20 \
  --instruction "<instruction from task.json>" --scorer task_score --max-steps 600 --epochs 2
```

Use `--policy yam_scripted` for `demo()`. `--epochs 2` runs A then B; `TASK_LOT=A` or `B` pins a lot. The demo uses published constants and visible settings, with no lot, joint or object-state reads. It first squeezes the linked width guides on the loose groups, which takes out their skew and lateral spread, eases to light pressure and seats that latch; then it pushes the stack to the feed edge with the rear guide, eases to light pressure, seats the latch and releases both hands.

Squeeze is judged on the final state only: the demo presses the pads by up to about 5 mm while squaring the stack, then eases them to under 1 mm before letting go, and only the final reading counts against the 2 mm limit.

**Native results for this source.** Scripted demo (600 steps, epochs 2): 1.0 in lot A and 1.0 in lot B. Dream-1 with the same source and settings: 0 in both lots; it did not reach the cassette within 30 s. The old geometry, media and scores do not apply to this revision.

`task.py` defines the scene/rules/demo; `parts.py` and `lots.py` are task-local helpers; `near_misses.py` checks invalid state, escaped/unsupported paper, penetration, partial tiers and face-based release. Run it directly, then the repository's factory/check.py. Fabrication source and drawings are MIT licensed; code is MIT. No vendor CAD or font files are included.

# Change a bassinet fitted sheet

Strip a used sheet, put it in the laundry zone, and fit a fresh sheet onto all four mattress corners. This is a real-size bassinet fixture, chosen so the entire mattress fits within the two fixed YAM arms' working area.

## Reproduce the scene

| Part | Material, dimensions and fixed reset |
| --- | --- |
| Mattress | Foam, 660 × 275 × 44 mm on a 14 mm plywood platform, 59 mm above tabletop at its top. Long dimension along Y. The near and far X edges are 206 and 481 mm from the arm bases. Side edges are at Y ±330 mm. No infant or bassinet side rails are present. |
| Used and clean fitted sheets | Two lightweight cotton jersey fitted sheets for this mattress, with sewn elastic hems and 37 mm fabric pull tabs at the four corner seams, so the jaws can pinch outside the foam. Coral begins fully seated under all four corners. The blue sheet is accordion-folded on the narrow shelf behind the far edge of the mattress, with four distinguishable pull tabs at the fixed coordinates in `task.py`. |
| Clean shelf | 45 × 526 × 20 mm horizontal shelf centred at X = 518 mm, beyond the mattress. Place the folded blue sheet on top. |
| Laundry zone | 207 × 600 mm pad with yellow edges, centred at X = 90 mm on the robot side of the mattress. Leave it empty at reset. |

Coordinates are from the midpoint of the arm bases, +X away from the robot and +Y to its left. Reproduce fabric folds by placing the four hem corners at the listed coordinates. The mattress separates clean-sheet staging from the laundry pad. Carry the used sheet toward the robot. Unfold the clean sheet from the far edge only after the used sheet is set aside.

## Run

From this directory, in the RoboCurve YAM Arena environment with Inspect Robots 1.0:

```bash
python run_task.py yam_scripted --log-dir runs/demo --setup setup.png
python run_task.py dream --log-dir runs/dream
python run_task.py yam_scripted --case miss-one-pocket --log-dir runs/miss
python run_task.py yam_scripted --case leave-used-sheet --log-dir runs/leave
python make_evidence.py runs/demo demo.mp4 --title 'Scripted YAM demo'
python make_evidence.py runs/dream model.mp4 --title 'Dream-1 baseline'
```

The runner uses seed 42, 20 Hz and a 1,400-step (70 s) limit, with zero object jitter.

## Scoring and fidelity

Full credit requires the used sheet completely clear of the mattress, at least 90% of its projected silhouette inside the pad, all four clean pockets fitted and released, and at least 95% visible coverage of the mattress top. Partial milestones are 0.17 for removal, 0.08 more for the pad, and 0.155 for each released clean pocket after removal. The video scoring protocol below describes each observable.

Cloth uses a mass-point grid with stretch and shear constraints. Swept contacts against the mattress, plywood, shelf and pad resolve at vertices, sampled edges and face centres. Triangle contacts resist penetration between the two sheets and between non-adjacent folds of the same sheet. The folded clean stack has a contact volume while it rests on the shelf. The cloth mesh is coarse, so fine folds and some edge-to-edge contacts remain approximate. The fitted hem remains caught until it is stretched clear; installation follows an outside-to-under motion and a release. The loose clean corners are kept in their initial folds until picked, representing tray friction. Jaw pinch and elastic edge retention are geometric approximations; the model omits fibre mechanics and seam damage. A denser visible fabric mesh follows the underlying contact grid. The sewn tabs droop when released. The elastic skirt has extra fabric at the perimeter to wrap down the mattress sides; no geometry is projected onto the mattress at seating. Both arms and their cameras run in IsaacLab. The scripted policy commands fixed robot waypoints.

## Revised native runs

On the revised cloth model and opposite-side clean shelf, the scripted YAM demo
scored **1.00**. All four clean pockets stayed seated after release, 95.1% of the
mattress top was covered, and the used sheet was clear of the mattress with
91.1% of its projected silhouette inside the laundry pad. Dream-1 completed the
same setup at the 1,400-step limit with a score of **0.00**. Both runs have native
video and a setup image.

On the L40S, the scripted run took 4m 09s and Dream-1 took 4m 08s,
including startup.

## Historical results before the pocket revision

The attached IsaacLab recordings and setup image show the earlier arrangement, with the clean shelf on the robot side. The opposite-side layout scored 1.0 in a local replay of the scripted motion using the YAM URDF kinematics and cloth solver.

| Trial | Score | Control steps | Runtime including startup |
| --- | ---: | ---: | ---: |
| Scripted YAM | 1.00 | 987 | 368 s |
| Dream-1 | 0.00 | 1,400 | 489 s |

The released used sheet was clear of the mattress with 86.8% of its mesh in the laundry zone. Four clean pockets were fitted, with 90.9% of the clean interior in the mattress surface band. In separately replayed contact-model controls, leaving one clean pocket loose scores 0.715 and leaving the used sheet on the mattress scores 0.0.

## Revision scoring from video

Record a fixed square-on overhead view with the four mattress corners and the full laundry pad visible. At the released finish, mask the blue cloth above the mattress top using the mattress corners as a crop. At least 95% of the top must be visibly covered. In the same view, outline the coral sheet and count the fraction of its projected silhouette inside the yellow-edged pad. It must be entirely clear of the mattress and at least 90% of its visible footprint must be inside the pad. Inspect all four corners from low side views to verify that each elastic pocket wraps below the mattress edge. If the shelf or a tool occludes the relevant fabric, use a clear view before assigning credit.

The model retains a compliant constraint on each fitted hem corner only. Pulling a corner outward or upward beyond the retaining travel releases it, including when that motion comes through the rest of the sheet. Interior vertices and shoulder vertices receive no corner-driven position commands. The pre-revision results above are historical.

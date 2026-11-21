# Roll a pasta sheet with a hand-crank roller

One arm lifts a loose dough sheet, squares it to the two steel rolls and guides it into the nip. The other turns the crank against bearing friction and rolling load while the stock gains traction. The released sheet is longer and thinner. This is the standard successive-pass operation of a household manual pasta machine. Its nominal stock and finish dimensions use settings 0 and 1 from the [Marcato manual](https://www.marcato.it/sites/default/files/node/documentation/files/2023-08/Atlas%20Ampia_instruction%20manual_1.pdf).

## Physical setup and reset

Use a full-size 150 mm hand-crank pasta roller such as a Marcato Atlas Roller 150, its regular table clamp and a 22 mm diameter hand grip. The simulated generic machine has a 188 × 202 mm steel footprint, 34 mm diameter counter-rotating rollers and 72 mm crank throw. Lock the housing with the normal clamp. Set the nip to the nominal 3.7 mm setting before each run. Orient the rollers along Y, with the hand crank on the right. The scene code contains the exact simulated dimensions.

| Item | Starting layout in the simulation |
| --- | --- |
| YAM pair and ordinary worktop | Arm base centers at X = 0, Y = ±305 mm, Z = 750 mm. The table surface is Z = 747.5 mm. Positive X points away from the bases and positive Y points to the robot's left. Both grippers start open and clear. |
| Clamped manual roller | Nip center at (445, 25, 920) mm. The 150 mm steel roll pair uses a 3.7 mm gap. The 63 mm dark spinner sleeve turns freely on the crank pin. |
| Wooden preparation board | 176 × 170 × 165.5 mm. Its center in plan is (296, 38) mm, and its top is at 913 mm. The raised board exposes an ordinary corner of the unengaged sheet to the fingertips. A stack of securely supported cutting boards can provide this resting height. |
| Plain fresh pasta sheet | One 92 × 86 mm piece, 4.8 mm thick, centered at (272, 112) mm with the 92 mm dimension along X. It projects 32 mm beyond the left board edge. No edge touches a roller. |
| Receiving board | One 223 × 150 × 11 mm smooth food-grade cutting board with a 14 mm low end lip, supported on wooden rests. The top center is (520, 25, 849) mm. The board slopes down at 24° as X increases. The near edge lies below the rolls and leaves the exit open to the board. |

Prepare the stock by mixing 100 g fine durum flour with 50 g water and kneading for 5 minutes, as in the plain-dough condition from [Berk, Glaue and Ünlütürk](https://doi.org/10.17113/ftb.64.02.26.9220). Use the widest setting to bring the prepared sheet to about 4.8 mm, then cut the regular stock patches with a ruler and straightedge off the robot rig. Keep unused patches covered, and use removable clean food-safe jaw pads and the clean unpowered roller. Calibrate the finished stock recipe on one spare strip before the robot runs. Use a new patch each trial and remove the preceding sheet from the receiving board. Set the roller to setting 1 after preparing the stock.

## Scoring

Use the trial video to confirm the continuous sheet rests entirely on the receiving board with both grippers clear of the dough for the final 0.4 seconds. The end-to-end distance must span at least 90% of the centerline length, and no fold may lift the sheet by more than 13 mm above the board. Measure the centerline with a ruler, divide it into six equal lengths, and take a light-touch caliper reading at the midpoint of each. Keep the sheet on the board and avoid pulling or flattening it while measuring.

- Each of the six readings in the 3.45–4.15 mm band earns one sixth of a point. Five passing readings earn 5/6 (about 0.83).
- Full credit also requires at least 107 mm of centerline length. A shorter sheet with six passing readings earns 5/6.
- Award zero when the sheet fails the receiving-board, release, flatness or straightness checks, or when all six readings are outside the band.

Either hand may crank. The simulator tracks actual contacts between the sheet and both grippers throughout each action and credits the lowest qualifying count across the eight actions of the released hold. Contact of at least 0.02 N breaks the hold. The physical trial uses the same hands-off requirement, observed on the trial video. See `pasta_sheet/dough.py` for the scorer and diagnostics.

## Run

Use the pinned [YAM IsaacLab Arena adapter](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730) and install the task with `python -m pip install -e .` from this directory. The registered Inspect Robots embodiment is `pasta_sheet`. Set `OMNI_KIT_ACCEPT_EULA=YES` after accepting NVIDIA's runtime terms. The commands use seed `1415158438`, 900 actions maximum at 20 Hz, and native physics at 480 Hz. Use `--device cuda:0` for native RGB and `--physics-device cpu` for physics. Output folders must be new. The simulation command prints `final_metrics.score` and writes a receipt, action trace and camera recordings. Camera runs also create `demo.mp4` at 1280 × 1080 and 20 fps with the overview above both 640 × 360 wrist views, plus `setup.png`.

```sh
python -B run.py --policy demo --device cuda:0 --physics-device cpu --output demo
python -B run.py --policy hold --no-cameras --physics-device cpu --output hold
python -B run.py --policy feed-only --no-cameras --physics-device cpu --output feed-only
```

The reference sends the standard 14-dimensional joint and finger commands using read-only held-stock pose and crank sleeve feedback. `hold` leaves both arms at reset for the complete budget. `feed-only` attempts to feed a stock piece while leaving the crank alone. The standard observations for hosted policies are three RGB camera views and the normal YAM joint positions from Inspect Robots.

The published recordings were verified by replaying their original robot actions through the revised scorer. The scripted demo scores 1.0 in 589 recorded actions, and the Dream-1 trial scores 0.0 in 900 recorded actions.

## Simulation fidelity and asset licenses

Six connected sections bend locally through native joints. Contact compression and opposing roll force under forward counter-rotation cause permanent thickness loss in each section. Between neighboring sections, length grows in inverse proportion to retained thickness to approximate volume conservation at fixed width. The surface mesh follows the connected sections. The steel rolls, free spinner, force-limited gear coupling, grippers, stock collision faces and supporting boards are physically simulated. The reduced dough model represents bending, pinching and permanent extension without modeling its full rheology or tearing.

| Asset | Source and license |
| --- | --- |
| Machine housing, rollers, crank and grip | Original procedural geometry, project MIT. |
| Stock board and output board | Original procedural geometry, project MIT. |
| Dough collision patches and continuous mesh | Original procedural geometry, project MIT. |
| YAM arm meshes and gripper splits | [i2rt YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) and [mesh provenance](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/dream/SOURCE). |
| Worktop and station room | Procedural room from the [pinned YAM adapter](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/scene.py). The adapter declares no repository license. |
| Overview camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) and task-authored extrinsics. |
| Left wrist camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE). |
| Right wrist camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE). |

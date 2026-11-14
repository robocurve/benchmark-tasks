# Saw a clamped wooden strip to length

A robot closes a screw vise on a loose wooden strip, picks up a hand saw and
cuts through the strip at a measured location. Full credit requires a separate
offcut and a retained piece measuring 150 ± 1 mm. This is a regular carpentry
operation: the [O*NET carpenter task list](https://www.onetonline.org/link/details/47-2031.00)
names measuring and marking cut lines and cutting materials to specified
measurements with hand tools as core tasks.

The scene has a 300 × 20 × 20 mm wooden strip, 80 mm vise jaws and a 200 mm
hand-saw blade. The strip rests partly in the open vise and partly on a low
support. A pencil line crosses its top 150 mm from the left end. The saw starts
in a rack. The robot must turn the screw to close the
moving jaw, then hold the saw and reciprocate it at the desired line. The saw
has no guide. A [75 mm commercial mechanics vise](https://www.faithfulltools.com/p/FAIMV75SG/Mechanics-Vice-75mm-%283in%29-with-Magnetic-Jaws)
and a [210 mm hand saw](https://silkysaws.com/silky-gomtaro-210-large-tooth-hand-saw/)
are scale and form references; the simulated geometry is procedural rather than
a replica of either product.

The scripted demonstration aims the blade centre on the waste side of the
150 mm pencil line. This allows for the 1 mm tooth-strip width and the measured
lateral deflection of the hand-held saw; scoring still measures the actual
retained wood, not the commanded blade position.

## Run

With `yam-isaac-lab-arena` and the Inspect Robots embodiment installed, from
this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E device=cpu \
  -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 3200 --seed 1 --store-frames \
  --instruction "Clamp the wooden strip in the vise, then saw it so the retained piece is 150 mm long, within 1 mm. Leave the saw clear of the wood when finished."
```

The scripted robot demonstration uses only the 14-D arm and gripper action
interface to move objects. It reads object poses and the simulated material
grid to align and feed the saw; these reads choose actions only. The task
physics decides material removal and scoring.

```bash
python demo.py --mode full --out demo-out
```

The same runner has `idle`, `grasp` and `clamp` diagnostic modes. It writes
`result.json`, `physics.json`, camera images and videos in the output folder.
On the pinned development simulator, the isolated scripted run reached a
1.0 end score in 2,170 actions (108.5 simulated seconds). The retained end
measured 149.25–149.50 mm across all sampled layers. On the official cloud
simulator, the final scripted run reached 1.0 in 2,310 actions, with two
physical pieces and a 149.25 mm retained end. Dream-1 was attempted with the
same task and 3,200-action limit. With Inspect seed 1, the standard policy
completed 3,200 actions and the task scorer returned 0.0; a full three-camera
model video was recorded. Earlier seed-0 trials stopped when the standard
policy rejected out-of-range gripper commands, so those trials have no score.

## Scoring and physical model

The task scores once at the end. Wood removed before the vise is tightened
precludes credit. Exactly two pieces must remain. For every
sampled layer across the retained end face, the distance from the original left
end must be 150 ± 1 mm. The moving jaw opening must be 19.2–20.2 mm, the screw
must have turned more than 1.5 rad, and the retained strip must remain within
3 mm of its original position. The whole saw tooth line
must be more than 5 mm above the original top face. Otherwise the score is 0.
These conditions can be checked with calipers, an index mark on the screw and
top and side images in a physical trial.

Each physics step uses the saw's measured pose relative to the freely moving
wood, its actual motion along the stroke and the measured saw–wood contact
force. The task removes only material under the tooth strip, in 0.25 mm by 2 mm
columns and 0.5 mm height increments. Collision shapes are rebuilt from the
remaining wood whenever a layer is removed. A separate rigid offcut is created when the remaining solid
disconnects across the board's width. The removal law is a reduced-order,
uncalibrated model of sawing; it does not resolve individual teeth, grain,
splintering, bending or heat. The saw is 200 mm long and 150 g, compared with
the cited 210 mm, 200 g reference; its contact strip is a rectangular proxy.
The 6 mm-per-turn screw drive and its friction are approximations, with a 100 N
force cap against the reference vise's stated 750 N maximum. The box-built
vise, saw, rack and support use task-owned
procedural geometry.

`task.py` holds the scene and public scorer. `runtime.py` and
`cutting_runtime.py` operate the physical screw, contact sensor and dynamic
colliders. `stock.py` is the material and connected-solid model.
`partkit.py` is a task-local derivative of the existing component helper and
does not modify the installed SDK. One layout is supplied; use
`-E object_jitter=0` for the stated dimensions.

## Third-party assets and licenses

| Asset | Use | Source and revision | License | Bundled here? |
| --- | --- | --- | --- | --- |
| YAM arms, grippers and meshes | Robot embodiment | [i2rt](https://github.com/i2rt-robotics/i2rt), commit `120c3c81400171174604e503943f8d1ebc891058`, via the installed YAM SDK | MIT, as shipped in `yam_arena/assets/yam/LICENSE.i2rt` | No |
| Station table, floor, cameras and lights | Base scene generated by the installed YAM SDK | [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), tested at `bd62771e60320433893bd1d2df0fc79b5ddcd6d1` | SDK repository states that it has no license yet; its i2rt robot assets are MIT | No |
| Isaac Lab-Arena and PhysX | Simulation runtime | Installed simulator, Isaac Lab-Arena 0.3.1 | Apache 2.0 for Isaac Lab-Arena; PhysX supplied with Isaac Sim | No |
| Wood, vise, screw, saw, rack and task code | Task geometry and logic | This folder | MIT; see `LICENSE` | Yes |

No third-party object mesh, texture or photograph is bundled. The task does
not load any object from the YAM SDK's station library.

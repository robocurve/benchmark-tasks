# Rehang two detached grommets on a full-size shower liner

**Category** — Societal relatability

> Two hooks on the shower liner have come loose. Rehang the liner properly without taking the whole curtain down.

## Hardware and layout

| Component | Selection and dimensions |
| --- | --- |
| Liner | iDesign Pebblz PEVA Shower Curtain, white, model 26580. 72 × 72 in (1,828.8 × 1,828.8 mm), 100% PEVA, reinforced top hem and twelve metal grommets. [Manufacturer](https://idesignhome.com/products/26580-pebblz-peva-sc-white) |
| Hooks | Twelve Moen SR2201CH double shower curtain hooks, chrome. The manufacturer specifies a 78.74 × 72.14 mm envelope, 3.56 mm steel wire and 8.89 mm end balls. For this single-liner service bay, face the deeper return toward the arms and leave the other return empty. Check approach clearance before drilling the mounts. [Moen drawing](https://assets.moen.com/shared/docs/product-specifications/sr2201sp.pdf) |
| Rod | Signature Hardware 902840 straight brass rod with wall flanges. Cut the 84 in option to fit a 1.93 m clear span, allowing for flange insertion. 25.4 mm tube diameter. [Manufacturer](https://www.signaturehardware.com/straight-solid-brass-shower-curtain-rod/902840.html) |
| Robot | Stock bimanual YAM. Each arm bolts to its own 100 × 100 mm steel column and 240 × 240 mm footplate. The simulated footplate is 15 mm thick. Mounting surfaces are 1.880 m above the floor and 610 mm apart. |
| Overview | Independent mast with a 45 × 45 mm column and 240 × 240 mm footplate. Camera optical center at `(-1.900, -0.400, 1.970) m`, aimed at `(0.390, 0, 1.080) m`. Keep the stock overview intrinsics and wrist camera mounts. |

Use the floor between the robot columns as the origin. Positive X points into the shower, positive Y points left as seen from the robot, and positive Z is up.

```text
Elevation from the arms                Plan viewed from above

1.978 m  ======== rod ========                      back wall X = 0.72 m
1.929 m  oooo o  o o ooooo                          return |  rod  | return
               ^  ^                           X = 0.414 m  +-------+
            loose at reset                          left YAM   right YAM
1.880 m  robot mounting plane                X = 0  Y=+.305     Y=-.305
         |  full-height liner |
0.000 m  ===================== floor         X = -1.90 m   overview camera
```

The rod axis is at X = 0.414 m, Z = 1.978 m. The fixture represents a service alcove with side returns at about Y = ±0.99 m and a back clearance panel at X = 0.72 m. The hook seating line is nominally at X = 0.378 m and Z = 1.929 m. Use centered 147 mm installed hook spacing, with the outer two hooks 1.617 m apart. The sheet can gather between hooks. Measure the spacing and clear bore on the actual liner before building. The model uses 152.4 mm of material width per grommet, a nominal 16.6 mm eye opening and a cap center 16.5 mm above the seating reference. These two values are estimates to verify on the purchased parts. The curved steel profile and two caps use the manufacturer’s overall dimensions.

The estimated liner mass is 0.32 kg. A 0.10 mm PEVA film at a representative 930 kg/m³ gives approximately 0.31 kg over 3.345 m², with the balance in the reinforced header and grommets. Weigh the dry commercial liner on a scale before physical replay. The manufacturer specifications establish material and nominal dimensions. The proposed installation has no physical trial data.

## Build and reset

1. Use a tape measure, level, calipers, scale, the manufacturer's flange hardware, floor and wall anchors matched to the substrate, socket and hex tools, and the YAM base adapters. Have the tall columns, adapters, and anchors sized for the installed arms and their combined static and dynamic loads. Secure both columns independently to the floor and independently anchor the camera mast. Use a temporary support during column installation.
2. Locate the rod axis with a level. Install flanges into structural backing or a test stand of equivalent rigidity. Keep the region in front of the rod unobstructed. The shower returns sit just outside the rod ends and the back wall is behind the liner.
3. Attach all twelve hooks, set their centered pitch with a tape measure, and attach the liner in grommet order. Label hooks and grommets 1–12 from right to left as seen from the robot. Reset by slipping eyes 6 and 7 off their respective returns, leaving all ten neighbors attached. Remove hands and let the unsupported pair settle. The whole liner remains in the scene. Keep the space beneath the hem clear.
4. Record an overview before manipulation. For physical scoring, verify ring capture below the tip, visible clearance between fingers and liner, and the hook taking load after hands withdraw. Monitor the original ten attachments throughout, and confirm the two target eyes remain attached without measurable drift over a one-second interval.

## Simulation

This is a full-mass discretized sheet in PhysX. Each grommet and reinforced header carries eight hinged sheet panels from top to bottom. The panels have collision geometry and distributed mass. Tension-only elastic seams transfer equal and opposite forces between neighboring panels. Linear and angular drag dissipate oscillation. Gravity acts on every body through PhysX. The eyelets must take load through physical contact with their hook returns. The model omits fine wrinkles and continuous self-contact at the narrow panel seams. It approximates the hook curves with connected round collision segments using the specified steel and ball diameters. The installed rod, hooks, and walls use fixed poses. This isolates grommet threading from sliding or swiveling the hooks. The strain fault is a damage proxy. The scorer also treats more than 40 N of resultant hook load over 0.15 s as a fixture overload, a conservative trial limit that is independent of the actual hardware’s failure strength. Calibrated PEVA bend and tension data, fixture pullout, and physical reset measurements remain unverified.

The prior shower-liner submission used a 232 × 60 mm section and a 122 mm rod. The other rejected curtain submission used a 189 × 200 mm swatch on an open table. This rebuild brings back the full hanging load, coupled motion between grommets, bathroom working height, and wall-constrained access. Its independent columns, table-free floor and camera mast follow the accepted wheelchair bay, with the arm spacing retained and the mount raised for the shower. The bay code here is independently implemented on the original YAM adapter. [Prior shower-liner review](https://github.com/robocurve/benchmark-tasks/pull/3071) · [Miniature curtain review](https://github.com/robocurve/benchmark-tasks/pull/2487) · [Accepted wheelchair reference](https://github.com/robocurve/benchmark-tasks/tree/f9fa38e5d4ec45b742834d2d8dfc33fa45fef3bd/tasks/unfold-wheelchair-for-transfer)

The USD files are generated from `geometry.py` and `usd_geometry.py` for this benchmark. The product links supply dimensional references. No commercial meshes or textures are distributed here. The original YAM robot and cameras load from the installed stack.

| External dependency | Revision and terms |
| --- | --- |
| YAM Arena adapter | `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`. The upstream adapter README identifies no repository-wide license. This task calls the locally installed adapter. [Source and license notes](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1#license) |
| i2rt YAM mesh and URDF used by that adapter | `120c3c81400171174604e503943f8d1ebc891058`, MIT under the adapter's bundled `LICENSE.i2rt`. [Bundled attribution](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam) |
| Isaac Lab Arena | `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`, Apache-2.0. |
| Isaac Lab | `91a0906f93802eb859192dcb0bca879f891e92d6`, BSD-3-Clause for the runtime used here. |
| Isaac Sim | Kit `110.3.0+feature.371399.00c488ae.gl`. Use the terms supplied with the installed NVIDIA distribution. |

## Run locally

Run from this task directory in the provisioned Lambda YAM environment.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
uv pip install --python "$VIRTUAL_ENV/bin/python" --no-build-isolation --no-deps -e .
python task.py
inspect-robots run --embodiment yam_shower_bay --policy shower_scripted --scorer task_score \
  -P hold_s=2 -E task="$PWD/arena_task.py" -E headless=true -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(jq -er .instruction task.json)" --seed 0 --max-steps 900 \
  --no-prompt --store-frames --log-dir /tmp/robocurve-full-size-shower
```

The scorer awards 0.35 for each correctly seated, physically supported target eye. It awards 1.00 after all twelve attachments remain stable and the hands remain clear for one second. Wrong-hook support, loss of an original attachment, excessive seam strain, fixture overload, or a dropped liner invalidates the run. The recorded scripted run scored **1.00 at step 425**. At the final sample, each hook carried approximately 0.25 N, the hands were clear, and the liner had held for 1.7 seconds. The Dream-1 run scored **0.00 at the 900-step limit**. Both runs used seed 0, zero jitter, headless mode, and the three 320 × 176 cameras at 20 Hz. To replay the Dream-1 evaluation, replace `--policy shower_scripted -P hold_s=2` with `--policy dream`.

Use `--epochs 2 --no-store-frames` with the scripted command for two consecutive resets in one simulator process. Each reset restores the strip root states and the fold joint positions and velocities before settling the liner again. Set `SHOWER_CONTROL` to `wrong-hook`, `partial`, `release-before-attachment`, or `original-hook-displacement` before the scripted command to replay deliberate failures. The wrong-hook control slides the empty neighboring hook along the same rod so an incorrect threading fits within the liner’s available slack. The task and other controls use the documented hook spacing. Keep the same observation, physics and action settings.

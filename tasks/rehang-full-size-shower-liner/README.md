# Rehang two detached grommets on a full-size shower liner

**Category** — Societal relatability

> Two hooks on the shower liner have come loose. Rehang the liner properly without taking the whole curtain down.

## Hardware and layout

| Component | Selection and dimensions |
| --- | --- |
| Liner | iDesign Pebblz PEVA Shower Curtain, white, model 26580. 72 × 72 in (1,828.8 × 1,828.8 mm), 100% PEVA, reinforced top hem and twelve metal grommets. [Manufacturer](https://idesignhome.com/products/26580-pebblz-peva-sc-white) |
| Hooks | Twelve Moen SR2201CH double shower curtain hooks, chrome. The manufacturer specifies a 78.74 × 72.14 mm envelope, 3.56 mm steel wire and 8.89 mm end balls. For this single-liner service bay, face the deeper return toward the arms and leave the other return empty. Check approach clearance before drilling the mounts. [Moen drawing](https://assets.moen.com/shared/docs/product-specifications/sr2201sp.pdf) |
| Rod | Signature Hardware 902840 straight brass rod with wall flanges. Cut the 84 in option to fit a 1.93 m clear span, allowing for flange insertion. 25.4 mm tube diameter. [Manufacturer](https://www.signaturehardware.com/straight-solid-brass-shower-curtain-rod/902840.html) |
| Robot | Stock bimanual YAM. Mounting surfaces are 1.880 m above the liner bay floor and 610 mm apart. Both arms sit on one tabletop on the upper deck. The full liner hangs past the deck into the lower bay. See the physical validation protocol for the layout and the installation checks. |
| Overview | Independent mast with a 45 × 45 mm column and 240 × 240 mm footplate. Camera optical center at `(-1.900, -0.400, 1.970) m`, aimed at `(0.390, 0, 1.080) m`. Keep the stock overview intrinsics and wrist camera mounts. |

Use the lower bay floor as the vertical datum and the midpoint between the robot bases as the horizontal origin. Positive X points into the shower, positive Y points left as seen from the robot, and positive Z is up.

```text
Elevation from the arms                Plan viewed from above

1.978 m  ======== rod ========                      back wall X = 0.72 m
1.929 m  oooo o  o o ooooo                          return |  rod  | return
               ^  ^                           X = 0.414 m  +-------+
            loose at reset                          left YAM   right YAM
1.880 m  stock tabletop mounting plane        X = 0  Y=+.305     Y=-.305
1.1325 m upper deck, ending at X = 0.080 m
         |  full-height liner |
0.000 m  ===================== floor         X = -1.90 m   overview camera
```

The rod axis is at X = 0.414 m, Z = 1.978 m. The fixture represents a service alcove with side returns at about Y = ±0.99 m and a back clearance panel at X = 0.72 m. The hook seating line is nominally at X = 0.378 m and Z = 1.929 m. Use centered 147 mm installed hook spacing, with the outer two hooks 1.617 m apart. The sheet can gather between hooks. Measure the spacing and clear bore on the actual liner before building. The model uses 152.4 mm of material width per grommet, a nominal 16.6 mm eye opening and a cap center 16.5 mm above the seating reference. These two values are estimates to verify on the purchased parts. The curved steel profile and two caps use the manufacturer’s overall dimensions.

The estimated liner mass is 0.32 kg. A 0.10 mm PEVA film at a representative 930 kg/m³ gives approximately 0.31 kg over 3.345 m², with the balance in the reinforced header and grommets. Weigh the dry commercial liner on a scale before physical replay. The manufacturer specifications establish material and nominal dimensions. The proposed installation has no physical trial data.

## Build and reset

1. Set the 0.7475 m tabletop on the 1.1325 m upper deck and mount the two stock YAM base plates 610 mm apart. Use the lower floor as the datum. Confirm structural and attachment ratings for the actual deck and bench before commissioning a physical installation. See [Physical validation protocol](PHYSICAL_VALIDATION.md) for the required mounting and fixture records.
2. Locate the rod axis with a level. Install flanges into structural backing or a test stand of equivalent rigidity. Keep the region in front of the rod unobstructed. The shower returns sit just outside the rod ends and the back wall is behind the liner.
3. Attach all twelve hooks, set their centered pitch with a tape measure, and attach the liner in grommet order. Label hooks and grommets 1–12 from right to left as seen from the robot. Reset by slipping eyes 6 and 7 off their respective returns, leaving all ten neighbors attached. Remove hands and let the unsupported pair settle. The whole liner remains in the scene. Keep the space beneath the hem clear.
4. Record an overview before manipulation. For physical scoring, verify ring capture below the tip, visible clearance between fingers and liner, and the hook taking load after hands withdraw. Monitor the original ten attachments throughout, and confirm the two target eyes remain attached without measurable drift over a one-second interval.

See [Physical validation protocol](PHYSICAL_VALIDATION.md) for real-trial strain, fixture force and post-trial inspection checks.

## Simulation

This is a full-mass discretized sheet in PhysX. Each grommet and reinforced header carries eight hinged sheet panels from top to bottom. The panels have collision geometry and distributed mass. Tension-only elastic seams transfer equal and opposite forces between neighboring panels. Linear and angular drag dissipate oscillation. Gravity acts on every body through PhysX. The eyelets must take load through physical contact with their hook returns. The model omits fine wrinkles and continuous self-contact at the narrow panel seams. It approximates the hook curves with connected round collision segments using the specified steel and ball diameters. The installed rod, hooks, and walls use fixed poses. This isolates grommet threading from sliding or swiveling the hooks. The strain fault is a damage proxy. The scorer also treats more than 40 N of resultant hook load over 0.15 s as a fixture overload, a conservative trial limit that is independent of the actual hardware’s failure strength. Calibrated PEVA bend and tension data, fixture pullout, and physical reset measurements remain unverified.

The independent floor bay and camera mast use the architectural pattern of the wheelchair reference. This implementation uses the stock YAM adapter and independently generated shower geometry. [Wheelchair embodiment reference](https://github.com/robocurve/benchmark-tasks/tree/f9fa38e5d4ec45b742834d2d8dfc33fa45fef3bd/tasks/unfold-wheelchair-for-transfer)

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
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
inspect-robots run --embodiment yam_shower_bay --policy shower_scripted --scorer task_score \
  -P hold_s=2 -E task="$PWD/arena_task.py" -E headless=true -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')" --seed 0 --max-steps 900 \
  --no-prompt --store-frames --log-dir /tmp/robocurve-full-size-shower
```

The scorer awards 0.35 for each correctly seated, physically supported target eye. It awards 1.00 after all twelve attachments remain stable and the hands remain clear for one second. Wrong-hook support, loss of an original attachment, excessive seam strain, fixture overload, or a dropped liner invalidates the run. The scripted run in the two-level bay scored **1.00**, with all twelve hooks supported for 1.65 seconds after release at the final logged sample. The lowest sampled panel center was 0.153 m above the lower floor during the trial and 0.217 m at the final sample. Dream-1 scored **0.00** at the 900-step limit. It continued after seven retried policy requests. Both runs used seed 0, zero jitter and three 320 × 176 cameras at 20 Hz. To replay the Dream-1 evaluation, replace `--policy shower_scripted -P hold_s=2` with `--policy dream`.

Use `--epochs 2 --no-store-frames` with the scripted command for two consecutive resets in one simulator process. Each reset builds a fresh PhysX scene inside the same Kit process and restores the strip root states and fold joint positions and velocities before settling the liner. This also clears contact warm-start state from the previous episode. Set `SHOWER_CONTROL` to `wrong-hook`, `partial`, `release-before-attachment`, or `original-hook-displacement` before the scripted command to replay deliberate failures. The wrong-hook control slides the empty neighboring hook along the same rod and resets the first loose eye onto that incorrect return. The parked-arm replay observes the real contact on the wrong hook. The task and other controls use the documented hook spacing. Keep the same observation, physics and action settings.

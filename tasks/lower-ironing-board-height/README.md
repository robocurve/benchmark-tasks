# Adjust an ironing board to a lower working height

Adjust the height of a standing domestic ironing board. An arm lifts and supports the worktop while the other operates the spring-return height lever. Guide the loaded scissor frame down one setting and let its mechanical stop carry the deck before withdrawing both hands. Either arm can take either role.

The test uses the lift, release, adjust and relock sequence in the manufacturer's instructions. The size and working range follow the 110 × 30 cm Brabantia Ironing Board A. [Manufacturer instructions, illustrations 4–5](https://www.brabantia.com/be_fr/amfile/file/download/file/346/product/7813/) · [Product data sheet](https://sales.brabantia.com/Articles/218729.pdf)

## Physical fixture and reset

Use a Brabantia Ironing Board A with its standard cover, or fabricate the equivalent working-range fixture from the dimensions below. Remove the iron and anything on the deck. Mark two adjacent locked settings near 888 mm and 805 mm. Each task episode starts at the higher setting. A ruler fixed to an independent stand makes the deck height visible during the test. Install its lower datum band at 805 mm beside the board, leaving at least 40 mm of clearance.

| Component | Reproduction dimensions and materials |
| --- | --- |
| Deck and rest | 1,100 × 300 mm rounded pressed-steel platform with padded cotton cover, about 2.7 kg including a 245 × 120 mm steel and silicone rest at its right end. The deck remains supported by the crossed frame throughout. |
| Crossed base | Two rigid frames made from paired 22 mm steel tubes, each about 1.045 m from pivot to pivot. The midpoint pivot allows both frames to cross. The upper end of one frame uses a hinge fixed to the underside of the deck. The upper end of the other travels on a longitudinal guide. The free lower foot slides in the same longitudinal plane. |
| Upper guide and latch | A steel runner has a 32 × 22 × 18 mm shoulder that meets either of two steel bearing faces carried by a spring-return lever. The bearing faces are 90 mm apart along the guide. The thumb grip lies 73 mm toward the operator from the lever shaft. The open angle is about 37°. Raising the grip moves the bearing faces out of the runner's path. |
| Pedestals and landing surface | The two arm bases are at X = 0, Y = ±305 mm and Z = 750 mm in the fixture frame. X increases toward the worktop. Positive Y is the arm to the left in the reset. The center of the deck is at X = 445 mm. The standing surface is level. |

Place the grounded foot pivot at X = 445 mm, Y = 325 mm, Z = 26 mm. At reset the other foot and upper runner are at Y = −335 mm. The upper pivot axis is at Z = 837 mm and the covered worktop is approximately 888 mm above the floor. Check that the lever is down and the runner bears on the high stop. Park both arms clear. Reapply the upper setting after each trial.

In the simulation the two foot pivots remain in the floor plane. One is anchored and the other runs on a lateral guide that prevents the board sliding across the lab floor as a whole. The crossed legs, travel, lever spring and loaded catch all use passive native physics. The cover is represented by a rigid worktop surface. The fixture reproduces the active height range and operating sequence.

Export the geometry inventory and solids from this folder to reproduce the mechanical fixture.

```bash
python -c 'import isaacsim; from ironing_height.builder import create_assets; create_assets("/tmp/ironing-board-fixture")'
```

## Scoring

Measure the top of the covered deck at its longitudinal center with a ruler. The lower mark is approximately 805 mm. The permitted error is 12 mm. The lower runner seat can also be checked at Y = −425 mm, with the same tolerance. An observer looking under the deck can see the returned height lever, which lies within 12° of its rest stop when engaged. At the lower notch the bearing faces must share at least 4 mm in depth and height. Check that the runner is against its load-bearing face within 4 mm.

- **0.40** for the correct deck height with the lever still held up.
- **0.65** for the correct height and a returned lever before the hands-off check.
- **1.0** when the lower setting supports the deck for one second after both grippers withdraw. No arm or object may carry the deck through this check.
- **0** if the deck is elsewhere at the endpoint.

The released board may rise slightly while the load is taken or the lever is returned. Scoring depends on the retained endpoint and allows either arm assignment.

## Run and validate

Use the pinned Isaac Lab-Arena and YAM dependencies. Default seed is 1415158438. The action limit is 380 at 20 Hz, with native contact dynamics at 240 Hz.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/ironing-board-demo
python run.py --no-cameras --policy hold --output /tmp/ironing-board-hold
python run.py --no-cameras --policy no-release --output /tmp/ironing-board-no-release
```

The package registers Inspect Robots embodiment `ironing_height` and scripted policy `ironing_height_scripted`. The reference uses elapsed time and ordinary joint positions. Each run records a source hash, start and end state, commanded actions and score. The camera run records each 640 × 360 camera at 20 fps including the unmodified reset frame. The left wrist optical center is set 55 mm rearward on its bracket to clear the worktop edge during close support.

## Asset provenance

The floor, board, scissor geometry, stop and lever are original MIT-licensed solids. [YAM asset terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera model license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

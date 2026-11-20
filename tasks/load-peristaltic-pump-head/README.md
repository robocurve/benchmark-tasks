# Seat two silicone lines in a peristaltic pump head

Changing tubing on a two-channel head preserves the separation between two independent streams. Guide opposite sides of each loose tube across the loading gap, seat it through both retainers, then close the common cam lever. The installation follows the two-channel loading sequence and equal-tube requirement in the [L/S Easy-Load II operating manual](https://digitalassets.avantorsciences.com/adaptivemedia/rendition?clid=SAPDAM&id=a7b40be8108c9fe84c637af6655668d54eba28d7&prid=original&vid=a7b40be8108c9fe84c637af6655668d54eba28d7).

## Physical setup and reset

Use two equal 352 mm lengths of 8.0 mm OD × 4.8 mm ID silicone pump tubing, marked with different colors. Each starts connected to a separate vented water bottle with a free discharge tip. The two aluminum parking supports are 90 × 16 mm. The two marked polypropylene cups have a 22 mm mouth, 23 mm outer diameter and 33 mm height.

The custom 102 × 101 mm two-track bench head has four freely turning stainless rollers, two 18 mm pitch tracks, two end retainers 92 mm apart, two independent spring-loaded pressure shoes and an over-center handle. [Fabrication and assembly](fixture/BUILD.md) covers the printable parts, hardware list, spring setup, guarded drive and timed check. [OpenSCAD fabrication model](fixture/pump.scad) and [drive sketch](fixture/water_check/water_check.ino) are included.

Measurements below are in metres in the YAM station frame, with positive Y toward the left arm. The worktop surface is at z = 0.7475.

| Part | Starting position or orientation |
|---|---|
| Pump | Body center at x = 0.358 and y = 0.004. Retainer centers at y = −0.042 and 0.050. Roller tracks are at x = 0.367 and 0.349. The lever starts open at 70 degrees. |
| Orange feed fitting | x = 0.335, outlet y = −0.174 and z = 0.8405. Tube initially extends along +Y above that x and is clear of both tracks. |
| Blue feed fitting | x = 0.307, outlet y = −0.174 and z = 0.8405. Tube initially extends along +Y above that x and is clear of both tracks. |
| Parking supports | x = 0.345, y = −0.133 and 0.130, z = 0.8365 at the top. |
| Two empty receiving cups | x = 0.383 and 0.348, y = 0.170. |
| YAM pair | Bases at y = ±0.305, both grippers open in the fixed ready pose. |

### Bottle pickup and priming

1. Use two 24 mm outside diameter HDPE bottles with a fixed 90° barbed elbow in each cap. Inside each bottle, connect a 4 mm OD × 2.5 mm ID pickup to the cap fitting. Cut its lower end on a 45° bevel and fix it 5 mm above the bottle floor, below the marked fill line. Make a separate 1 mm air vent in each cap. Fill with 18 mL of clean water, with each free surface below its parked outlet tip. Keep the two circuits separate.
2. With the drive isolated and the cam open, attach a 10 mL syringe with a hose-barb adapter to the free discharge end. Draw water from the bottle until the full working line has no visible air and at least 2 mL of bubble-free water reaches the syringe barrel. With its free tip above the source's water level, remove the adapter and wipe off the exterior drop. Repeat on the other channel. The circuit is fully primed at episode start, with both discharge tips free.
3. Place both primed working sections across the two open rests, the same orientation as the setup image, and tared empty cups at the positions above. Check that the outlet tips remain above the bottle fill levels in both the parked and correctly seated positions. Replace any lost source water before setting the cup tare. Reset the cam to 70°, start the rotor with an axle indexed at 12 o'clock, and put both robot arms in the ready pose. Repeat the syringe step after breaking either line or when air is visible before a run.

## Score a trial

Inspect the closed head and both exits after the grippers are clear. Each loaded tube passes through both retainers and over the roller. It stays within 4.1 mm of its track center and within 5 mm of the centerline height at both retainers and between them. The cam must be within 4 degrees of its closed stop. Neither tube may cross, kink or leave the roller path. Use a ruler and a photograph for alignment. Keep hands off the active loading section and lever.

After 0.5 s with both hands clear, run the guarded drive at 24 rpm for a continuous three seconds. Weigh each tared cup immediately afterward using a scale with 0.1 g resolution. Each correctly installed channel earns 0.5 when its own cup gains at least 1.0 g of water (approximately 1.0 mL). A cup with no correctly installed line earns zero. Neither color is tied to a specific track. Switching color-to-track mapping is valid when each tube is independently routed, remains uncrossed and discharges into its corresponding cup. A dropped outlet, missed retainer, unlatched cam or stationary episode earns zero for every affected channel.

In Isaac Lab the native motion of the two pressure shoes measures loaded occlusion. A load of 1.2 to 6.5 mm at each shoe is the modeled cross-section displacement range for the bench head, using 850 N/m elastic support as a reduced-order contact response. The evaluator checks continuous positive-Y centerlines, the pose of both outlets above catch mouths, the independent occlusion loads and the released cam. Once eligible for 0.5 s, each clear channel accumulates volume using the L/S 25 nominal rate of 1.7 mL per revolution at the fixed check speed. The score requires all 60 consecutive flow steps for a channel, even if the volume threshold is reached sooner. Loss of eligibility restarts its check. The colored stream and cup level show the measured simulated outcome. The camera and fluid proxies do not change tube or cam physics.

## Run and score in Isaac Lab

Use Inspect Robots, the pinned YAM adapter and the accepted-license Isaac Lab Arena runtime. The episode has one fixed layout and seed `1415158438`, with 860 actions maximum at 20 Hz.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --output /tmp/peristaltic-demo
python run.py --device cuda:0 --no-cameras --policy hold --output /tmp/peristaltic-hold
python run.py --device cuda:0 --no-cameras --policy open-head --output /tmp/peristaltic-open
python run.py --device cuda:0 --no-cameras --policy single-tube --output /tmp/peristaltic-single
```

`receipt.json` reports the score, steps, source fingerprint and camera data. The rendered run records all initial and action frames natively in each of the three 640 × 360 views. The reference is a sequence of YAM gripper joint targets with ordinary joint-state tracking. The scorer reads native tube and cam motion after physics steps. The stationary control uses the entire horizon. The open-head control routes both tubes and skips the cam operation. The single-tube control routes the orange tube and attempts lever closure without loading the second channel.

## Assets and licenses

| Asset | Source and license |
|---|---|
| Head case and two retainers | Procedural geometry in `builder.py`, MIT (`LICENSE`). |
| Roller and flange geometry | Procedural cylinders in `builder.py`, MIT (`LICENSE`). |
| Cam, side lever and spring shoes | Procedural geometry and constraints in `builder.py`, MIT (`LICENSE`). |
| Both feed bottles, elbows and flexible tubes | Procedural geometry and constraints in `builder.py`, MIT (`LICENSE`). |
| Two external tube supports | Procedural geometry in `builder.py`, MIT (`LICENSE`). |
| Receiver cups and liquid visualization | Original geometry in `builder.py` and `environment.py`, MIT (`LICENSE`). |
| Pair of YAM models | [i2rt YAM terms](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) through the pinned adapter. |
| Worktop, legs and floor | Task-authored box geometry in `builder.py`, MIT (`LICENSE`). |
| Room lights | Task-authored light placement and intensity in `environment.py`, MIT (`LICENSE`). |
| Fabrication and drive controller | Task-authored OpenSCAD and Arduino source in `fixture/`, MIT (`LICENSE`). |
| Overview camera | Task-authored placement with [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE). |
| Left wrist camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) through the pinned adapter. |
| Right wrist camera | [MIT YAMLAb calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE) through the pinned adapter. |

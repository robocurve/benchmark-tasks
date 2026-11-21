# Scrape frost from the windshield under a spring-loaded wiper

Clear the ice that lies under the resting rubber blade. Both arms work across the width of the windshield. One arm carries the spring load of the wiper while the other pushes the actual scraper bevel forward along the inclined pane. After the swath is clear, withdraw the scraper and control the returning wiper all the way to glass contact. The grippers can exchange these roles. [Volvo S60, windshield wiper service position](https://www.volvocars.com/en-ca/support/car/s60/21w22/article/1f2a2f75c7fab5e2c0a81f6f0da34d7b_539ece7cc8f1465ec0a8cc8e1baef5a7_99cf68558e387af8c0a801e800be397a17/) · [Ford Focus, freeing blades from snow and ice](https://www.fordservicecontent.com/Ford_Content/vdirsnet/OwnerManual/Home/Content?ProcUid=G1875091&Uid=G1875090&buildtype=web%2F1000&countryCode=USA&div=f&languageCode=en&moidRef=G1750408&userMarket=USA&vFilteringEnabled=False&variantid=6056)

The windshield and blade stay on the fixture throughout. The visible cleared area measures the scraping outcome. The original attached blade stays in place. The rest of the windshield is already clear. The task covers a small section of a lower windshield at its ordinary size.

## Equipment and fixed reset

X extends forward from the midpoint of the two YAM bases, Y extends to the left, and Z is height above the floor. The YAM bases are 610 mm apart. Dimensions below are millimetres, except where noted.

| Item | Build and placement |
| --- | --- |
| Bench | Flat top 1,100 × 1,250, 748 above the floor. Fix the windshield and cowl firmly. |
| Windshield | Use an intact compact-car laminated windshield on its donor frame or a flat laminated-glass panel with protected edges. Model lower surface coordinate origin is at world (240, 0, 812). Its positive local X slopes 20° up and away, while local Y remains horizontal. The pane extends approximately from 0 to 666 along local X and from −372 to +594 along local Y. The black EPDM border covers its four edges. |
| Cowl | Use a conventional cowl with lower steel support and dark polymer vent grille. Model grille measures 104 × 997 × 30, centred at world (178, 111, 781). A blue painted hood lip 95 × 1,015 × 32 meets its near edge. |
| Wiper | Use an unpowered conventional sprung wiper arm and a similar attached 440 mm blade, as left in a service position. Disconnect the motor and lock the spindle. The lift pivot is at pane coordinates (−24, 196) and 31 normal to the glass. The steel arm grip lies 126 along the stem. The blade lip is centred at pane coordinates (226, 56) and lies parallel to pane Y, along the glass. Use an arm which returns under spring load through the first 24° of lift. If the donor has an over-centre service detent outside the working arc, keep that original hinge behaviour and use the documented partial lift. |
| Scraper | Ordinary plastic 184 × 140 mm ice scraper with a single continuous bevel, a roughly 26 mm wide grip and no metal edge. Lay flat at world (274, −494), push edge facing forward, grip top up. |
| Frost band | Use the template with pane corners (180, −145) and (276, −25). The resting blade covers an approximately 9 mm wide band near local X226. Mark all four corners outside the patch for photo alignment. |

Use a clean undamaged blade and plastic scraper. Check that the blade lifts freely and will not freeze to the window. For a repeatable physical frost layer, mark the 96 × 120 mm template boundaries externally with removable tape, then hold the blade clear while preparing the frost. Chill the laminate to −10 ± 2 °C with reusable packs on the back. Spray 1.0 ± 0.2 mL of clean water as 2–4 fine passes through the template, using a metered atomizer or weighing the bottle before and after. Allow each pass to freeze before applying the next. The nominal equivalent water depth is 0.07–0.10 mm. Remove the template, dry the blade, and take the raised-blade scoring baseline described below. Lower the blade gently without another spray. Take a separate setup photo with the blade resting on the glass. Use water frost only and wipe melted water off the glass and surrounding bench between runs.

Check the physical arrangement before the first trial. A complete pass with the smooth ABS edge standing about 25–45° to the cold pane should strip the layer in a swath, leaving a visible clean boundary. If the layer melts in under 36 seconds, extend the chilled backing or work in a colder space. If ice remains under a normal scraper push, adjust the coating preparation first. The reported force thresholds are nominal simulation settings for thin frost. Keep both YAM arms dry and the wiper motor disconnected.

The task silhouette and kinematic build can be reproduced from generated meter-scale USD solids and `geometry-inventory.json`. After installing the pinned task dependencies, export them and their hinge settings before reproducing the fixture.

```bash
python - <<'PY'
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True, device="cpu").app
try:
    from windshield_frost.builder import create_assets
    create_assets("/tmp/windshield-cad")
finally:
    app.close()
PY
```

## Physical scoring

1. **Before the trial**, take a fixed overhead or oblique baseline photo after the frost has formed, with the blade held clear of the entire marked rectangle. Confirm that all four corner marks and the frost under the resting blade's footprint are visible. Keep the camera, framing and lighting fixed for the post-trial photo.
2. **At the end of the trial**, record the blade resting against the glass. Use the side video to verify the one-second finish and controlled return. Save this evidence before moving anything.
3. **After recording the finish**, have the scorer raise the arm slowly about its hinge by the exposed stem. Lift the rubber lip off the glass without sliding it across the pane or disturbing the frost. With the blade clear of the entire rectangle, immediately take the second area photo. If the inspection lift disturbs the frost, repeat the trial from reset.

Align the two raised-blade photos using the four template corners. Divide the area frosted in the baseline and clear in the final photo by the total frosted area in the baseline, including the band beneath the resting blade. A transparent 10 × 10 grid over both aligned photos is sufficient. Estimate partly clear cells by area. The physical score is 0.8 times the fraction cleared, except the full finish described below. A scraper waving above the pane and lifting the blade without scraping both earn zero. Touching bare glass outside the rectangle does not add credit.

Award 1.0 when at least 95% of the rectangle is clear, the wiper is gently lowered until the rubber rests on the glass, the scraper is motionless on the bench, and both grippers withdraw from both objects for at least one continuous second. A snap-down disqualifies full credit. Inspect the last 20 mm of travel in a 60 fps side video when necessary. The lip should not traverse more than about 3 mm between successive frames of its last 20 mm, equivalent to the simulation check of 43°/s at the wiper pivot. Assess the finish from visible rubber-to-glass contact and motion.

In simulation the beveled ABS edge must sweep a frost cell while actually pressing against the pane with the working edge down and with forward travel. Each counted stroke segment covers the cells swept by the contacting scraper edge across consecutive samples. The scraper bevel bends on a passive PhysX joint with 6.2 N/mm of stiffness. With the edge at the glass surface, the measured deflection estimates its transmitted load. The nominal 1.2–45 N range excludes unloaded passes and crashes. The virtual coating is divided into 6 × 10 mm cells. Clearing a cell changes only the thin frost rendering and the measured fraction. The scraper and wiper poses remain governed by native physics. The plan controls the measured pose of the tool within the grasp by moving the gripper.

## Native trials and model evaluation

Run through the pinned Isaac Lab Arena and Inspect Robots versions in `pyproject.toml`. The embodiment is `windshield_frost`, with 240 Hz contact substeps and 20 Hz action updates. The fixed reference seed is 1415158438. Each trial has at most 720 actions, 36 seconds at normal control speed. Each invocation writes the setup, trace and score receipt.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/windshield-demo
python run.py --no-cameras --policy hold --output /tmp/windshield-hold
python run.py --no-cameras --policy air-pass --output /tmp/windshield-air-pass
python run.py --no-cameras --policy no-lift --output /tmp/windshield-no-lift
python run.py --no-cameras --policy snap-return --output /tmp/windshield-snap-return
```

Scene solids and the generative geometry are supplied under MIT. The robot assets and wrist cameras come from the pinned adapter. [YAM license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera asset license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)

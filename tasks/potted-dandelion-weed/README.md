# potted-dandelion-weed

**Instruction:** "Weed the pot: pull the dandelion out with its whole root and put it on the tarp. Leave the pot standing where it is."

A young dandelion has come up in an 8 in (203 mm) black plastic bulb pan of potting mix. It has a rosette of six toothed leaves round a crown at the soil surface, a yellow flower on a leaning stalk, and a 63 mm taproot down in the mix. It comes up 40 mm from the middle of the pot. A folded blue poly tarp lies on the bench to the left, where pulled weeds go.

The weed has to come out with its whole root, because a dandelion grows back from any piece of taproot left in the soil. The gardener does it like this:
- **Hold the pot down with one hand.** The root holds harder than the pot and mix weigh. Pulled with one hand, the weed comes up with the whole pot hanging from it.
- **Grip the crown itself, down at the soil, with the other hand.** Pulling on the leaves only tears them off.
- **Pull straight up and steadily.** A sideways yank, or levering the crown over, snaps the taproot at its narrow neck.

## Why it matters

Weeding pots is one of the most common garden chores. About 81% of US households took part in lawn and garden activities in 2023. The source is the National Gardening Association's *National Gardening Survey*, 2024 edition, as reported by Garden Center Magazine. Many of those households garden in pots, tubs and planters on patios, balconies and doorsteps. Weeds come up in every one of them, from wind-blown seed. Dandelions are among the commonest, and the usual way to get one out of a pot is by hand.

What makes it hard for a robot:
- **Two hands, two different jobs.** One hand pinches the pot's rim and holds it down. The other pulls the weed. The soil lets go of the root at 10 N, but the pot weighs only 7.8 N. A one-hand pull lifts the pot, and that scores 0. The holding hand has to pinch, not just press. The pull is inside the pot's footprint but the rim overhangs it, so a push down on the rim only tips the pot up on that edge.
- **Grasp at the soil line.** The crown is 22 mm wide and stands 15 mm proud of the soil. Six leaves surround it, and only two narrow gaps let the fingers past. The pad tips have to reach within a few millimetres of the soil without the gripper landing on the leaves. A higher grip catches only leaves, which tear off at 4.5 N, well below the 10 N the root needs.
- **Pull direction.** The root's neck takes 40 N in a straight pull but only 0.60 N m of bending. A sideways push or jerk of 17 N at the grip snaps it: a yank, levering the crown over, or dragging the weed sideways before it is out. A snap cannot be undone.
- **Room for two hands.** The pot is 203 mm across. The holding hand works on the near rim and the pulling hand over the crown. The holding hand has to lean outward (17 deg), or the two wrists hit each other over the pot.

## Physics (all native PhysX; nothing scripted)

| Item | Model |
|---|---|
| Pot + mix | One free rigid body, 0.80 kg: moist peat/perlite mix in a 203 x 102 mm bulb pan with a 171 mm base. Its colliders are 28 tapered wall staves, a rolled rim (a 7 mm round bead), a base, and a 10 mm soil layer. The soil surface is 18 mm below the rim, and the layer has a 16 mm square hole where the taproot goes down. |
| Lower taproot (35 mm) | A free body, bonded to the pot by a breakable PhysX fixed joint (the soil's grip): 10 N / 3 N m. |
| Crown + upper taproot (28 mm) | A free body, bonded to the lower root at the neck by a breakable fixed joint: 40 N in tension, 0.60 N m in bending. |
| Six leaves | Each a free body, bonded to the crown by a breakable fixed joint: 4.5 N / 0.10 N m. |
| Tarp | A fixed thin body on the bench. |

Nothing moves a body or releases a joint by script. Every break is PhysX's own joint break, triggered by the measured constraint load.

## Forces (numbers)

| Quantity | Value |
|---|---|
| Pot weight | 0.80 kg x 9.81 = 7.8 N |
| Root grip | 10 N, which is 2.2 N more than the pot weighs. Pulled without a hold, the pot lifts first. |
| Hold-down the second hand must give | At least 10 - 7.8 = 2.2 N down at the rim. It must also resist the tipping moment, because the pull is 40 mm off the pot's axis. |
| Pinch on the 7 mm rim | The jaws close on the rim: about 5000 N/m x 3.5 mm = 17 N per pad. With friction of about 0.7, that holds about 24 N. |
| Pinch on the crown | The pulling hand must carry the 10 N release load in friction. A pinching robot hand holds about 16-18 N, so the margin is about 60%. In the sim the jaws close fully on the 22 mm crown, about 50 N per pad at the effort cap. |
| Side load that snaps the neck (35 mm below the grip) | 0.60 / 0.035 = 17 N |

## Demo (scripted waypoints, both hands)

1. The right hand comes down on the near (-y) rim, leaning 17 deg outward, jaws across the wall, one finger inside and one outside. It pinches the rim, pad tips 9 mm below the rim top.
2. The left hand comes over the crown with its jaws part open, across it (closing along x, between the leaves). It goes down to 25 mm over the crown, then slowly down until the pad tips are 4 mm over the soil. It brings the jaws in to 2 mm round the crown, then closes on it over 1.2 s.
3. The left hand pulls straight up 95 mm in 3.5 s while the right hand holds the pot down. The squeeze alone leaves the root seated: it rises 2.4 mm at most before the pull. The soil's grip lets go during the pull, and the whole root slides out with the neck intact.
4. The right hand lets go of the rim. The left hand carries the weed over the tarp, lowers it, lets go, and goes back to rest.

Every pose was checked with the vendored IK, and the arms' URDF meshes for clearance, before the run. The two arms stay at least 40 mm apart. The first two-hand layouts failed. With both hands straight down, the two wrists collided over the pot. A tilted pulling hand, a full-speed close from wide open, or pads high on the crown knocked the crown loose or snapped it as the jaws closed.

**Result:** run `20261119-181120-a411cb` scored 1.0. Success came at step 343 (17.2 s). The root rose at most 2.4 mm under the squeeze and came out on the pull. It came out whole. The pot never rose, did not tilt, and moved 0.1 mm.

**One-hand baseline:** run `20261119-181429-17937a` (`ONE_HAND = True` in `task.py`). It used the same left-hand squeeze and pull with the right hand parked, and scored 0. The pot came up with the weed, 24 mm off the bench, and then dropped as the crown slipped from the jaws; the root stayed in the soil.

## Scoring

The scorer reads the bodies' poses at every control step (20 Hz).

| Score | Condition |
|---|---|
| 0 | Any of these, and it stays 0 once it happens: nothing has loosened; **the root snapped at any time** (the neck gap is over 3 mm or the bend is over 25 deg); **the pot was lifted more than 10 mm off the bench at any time**, which is what a one-hand pull does. |
| 0.25 | The root has loosened from the soil (risen 5 mm relative to the pot), still intact, and the pot has not been lifted. |
| 0.5 | The whole weed is out of the soil (root tip 3 mm above the soil or outside the pot), still intact, and the pot has not been lifted. |
| 1.0 | All of these: out and intact; lying on the tarp (crown and root tip inside its outline, below 6 cm); still (under 3 cm/s); let go (no finger pad within 30 mm of the crown); the pot never lifted, still upright (under 10 deg) and within 40 mm of where it started. |

**Leaf tolerance:** torn-off leaves do not change the score; the weed is judged by its crown and root. In the demo, the jaws closing between the leaves knock four of the six leaves off their 4.5 N joints (three land in the pot or beside it; one is flung about 40 cm across the bench, a sim artefact of the rigid leaf plates snapping free). A real gardener also often loses a leaf or two. A policy that pulls by the leaves simply never loosens the root, so it scores 0.

**Real trial:**
- Tape the tarp's outline and a ring round the pot's start position.
- Stand a ruler with a 10 mm mark beside the pot. A lift is judged from the video against that mark.
- Judge "root intact" and "root out" from the video. The neck is a magnet joint, so a snap shows as two separate pieces.
- Check the pot's tilt with a square and its offset with a ruler.

## Real-world counterpart (reusable, no consumables)

- **Pot:** a standard 8 in round plastic bulb pan (half-depth pot): 8 in (203 mm) top, 6.75 in (171 mm) base, 4 in (102 mm) deep. This is the size sold as an "8 in bulb pan" by orchid and nursery suppliers (in black, green or white; colour does not matter). The same shell, 2 mm wall, is in `fabrication/bulb_pan_8in.stl` for printing (a 220 x 220 mm bed or larger). A deeper 8 in azalea pot (about 150 mm tall) would put the pull height out of the arm's straight-down reach.
- **Mix:** fill the pot with about 2.0 L of lightly moist peat/perlite potting mix, leaving the soil 18 mm below the rim. Weigh the pot and add or remove mix until it weighs 0.80 kg in all.
- **Tarp:** a standard blue poly tarp, folded to about 320 x 240 mm.
- **Weed prop:** 3D-printed in PLA. Print files are `fabrication/crown_upper_root.stl` and `fabrication/lower_root.stl`, made by `fabrication/make_stl.py` from the same geometry as the sim:
  - crown: 22 mm across, 15 mm tall, brown;
  - upper root: 11 mm across tapering to 9 mm, 28 mm long, printed in one piece with the crown;
  - lower root: 9 mm across tapering to 3 mm, 35 mm long, tan;
  - leaves: six, about 85 mm long and 30 mm wide, cut from green 1 mm craft foam or taken from a silk dandelion spray;
  - flower: a silk dandelion flower pick on a 70 mm stalk, glued into the crown.
- **Magnets:** K&J Magnetics catalogue numbers. K&J's "case 1" figures are for one magnet on a thick steel plate, so each joint is set by measurement, not by the catalogue value (see the pull-off check below).
  - Neck: two D62-N52 discs (9.5 x 3.2 mm), one in each root half, face to face. A printed 6 mm spigot on the upper half sits 3 mm deep in a socket in the lower half. The magnets take the straight pull. The spigot carries bending until it levers out. Set the joint to about 40 N straight pull and about 17 N sideways at the crown (0.6 N m), checked with a spring scale. That gives a neck that is strong in tension and weak in bending.
  - Soil grip: one D42-N52 disc (6.4 x 3.2 mm; 2.85 lb = 12.7 N case 1) in the root tip. It sits on a 3 mm steel disc glued to the pot floor, at the bottom of a 16 mm plastic tube that keeps the root's hole open through the mix.
  - Leaves: each held on a D31-N52 disc (4.8 x 1.6 mm; 0.93 lb = 4.1 N case 1) against a steel pin in the crown.
- **Pull-off check (before each session):** use a 0–50 N hanging spring scale (1 N divisions) with a wire loop.
  - Soil grip: pot clamped to the bench, loop round the crown, pull straight up slowly. The root must release at 9–11 N. Too strong: add one 0.1 mm paper spacer on the steel disc; too weak: remove one.
  - Neck: hold the lower root in a vice, pull the crown straight up: release at 36–44 N. Sideways at the crown top (35 mm above the neck): release at 15–19 N.
  - Leaves: pull one leaf straight out at its base: release at 4–5 N.
  - A joint outside its window is re-spaced or its magnet replaced.
- **Reset:** re-seat the magnets, push the root back down its tube onto the steel disc, level the mix, put the pot back in its ring, and clear the tarp. Nothing is used up.

## Asset licences

Every mesh in the scene is procedural, generated by `weed.py` and `meshlib.py` and written for this task: the pot, mix, perlite, weed, leaves, flower and tarp. The scene uses flat material colours and no image textures. No third-party models, scans or textures are used. The scene assets are released with the task under the benchmark repository's licence. The YAM robot model and the station come from the challenge runtime under their own licences.

## Fidelity limits

- The potting mix is rigid. Real mix crumbles, and a little lifts with the root. The soil's grip on the root is a single break threshold rather than a gradual pull-out.
- The rim is pinched as a rigid 7 mm round bead. A real thin-walled pot flexes a little under the pinch.
- Leaves are rigid plates. Colliders are boxes and cylinders inside the visual meshes.
- The flower stalk is visual only.
- Contacts on the light weed parts have their push-out speed capped (1.5 m/s on the crown, 0.3 m/s on the root and leaves). Without the cap, the jaws closing on the 12 g crown kicked it hard enough to break its joints.
- All strengths are estimates for a young dandelion in loose mix, chosen to match the magnet prop.

## Files

- `weed.py`: geometry, parts and joints.
- `task.py`: environment, finger-pad readout and demo.
- `scoring.py`: the scorer.
- `simkit.py`: scene machinery. It adds bonds between task bodies to `Anchor`.
- `meshlib.py`: mesh helpers.
- `fabrication/`: print files for the pot and the weed prop (`*.stl`) and the script that makes them from `weed.py`.
- `runtime/`

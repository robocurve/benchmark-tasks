# Seat a flowerpot in a four-cord hanger

Place the free pot inside the hanging cradle and let go. The pot must remain upright when the hanger swings.

One hand can spread the slack cords while the other lowers the pot onto the bottom knot. The one-arm controls use the pot rim to spread the cords.

## Setup and reset

Use a 330 g flowerpot, 96 mm tall, with a 123 mm outer lip (120 mm at the rim centerline) and an 88 mm base. A four-cord hanger hangs from a fixed hook 490 mm above the table. Each 4.8 mm diameter cord runs from the hook to the free lower knot. In the YAM world frame, place the hook at x=390, y=40 mm and the upright pot base at x=300, y=−145 mm. Start both grippers open.

Load a fresh fixed scene with seed 0. The empty four-cord net hangs from the hook and the pot rests separately on the table. Confirm all four cord paths are free.

### Assemble the hanger

Bring four equal lengths of 4.8 mm braided cord from a common top ring to a compact lower knot. With the net empty, set the ring-to-knot vertical drop to 405 mm. Space the four strands around the upper ring at quarter turns. Gather the four cords with four wraps. The simulated lower knot is about 21 mm wide and 19 mm tall. The four loaded strands emerge from its top. Leaving any strand underneath changes how the pot is supported. The [station builder](tools/build_station.py) gives the centerline and attachment coordinates. The suspension hook is the only fixed support.

Tie a shorter diagonal from each main strand to its next neighbor counterclockwise when viewed from above. Use four pieces of 3.6 mm braided cord. Leave 120 mm of cord free between the ties on each piece, with extra length for the knots. In the empty setup, each diagonal starts 287 mm below the hook and ends on the next strand 366 mm below the hook. These four diagonals form the lower basket and move with the main cords.

For a physical replica, use a rigid plastic flowerpot and braided textile cord. Match the dimensions and loaded mass below. The simulation models the collision shape and mass; the physical material choices have not been bench-tested.

Use a tapered pot with a hollow upper lip. Ballast its lower portion to a total of 330 g, leaving at least the upper 30 mm clear so a finger can go inside the rim. For a physical reset, remove the pot from the net, untwist the strands, and let the empty knot come to rest before placing the pot at its marked location.

## Evaluation

Score 0.5 when the knot sits under the base and four cords surround the pot. At 50 mm above the base the four strands must each cross at a radius of 45–80 mm, with no angular gap of 180° or more. The knot must be within 44 mm of the base center and between 25 mm below and 8 mm above it. For 1.0 the base must hang over 50 mm above the tabletop, within 10° of upright, with the hands clear of pot and cords. After 0.3 s of clearance, apply 0.28 N horizontally for 0.15 s. Keep all conditions true for the rest of the 106-step window at 20 Hz. Otherwise score 0.5 only if the pot remains seated at the end, or 0.

### How to check the outcome

Mark the pot 50 mm above its base. Use synchronized views from above and from the side with a ruler in frame. At the band, check that each cord crosses once, that its centerline is 45–80 mm from the pot axis, and that the four crossing points surround the axis. A straight line through the axis must not leave all four points on one side. In the side view, locate the knot relative to the base and measure the pot's tilt against a vertical line. The four strands may move freely through the trial.

The automated test starts when the support, height, angle and hand-clearance conditions have all held for six consecutive frames at 20 Hz. For clearance, the measured hand centers must be over 130 mm from the pot reference point (47 mm vertically above its base origin) and over 55 mm from the sampled cord centerlines. The test applies a horizontal force in the station's positive x direction for the next three control frames. The total undisturbed-plus-disturbed observation window is 106 consecutive frames (5.3 s), starting with the initial six. If the pot loses any required condition after the pulse begins, it cannot earn full credit later in that trial. At the final frame, a pot seated in the four cords still earns half credit; an unseated pot earns zero.

For a physical check, place a ruler and a vertical reference beside the pot. A horizontal push gauge at the height of the loaded pot’s center of mass can apply the test pulse. Film the gauge and pot together. At 60 frames/s the 0.15 s push lasts nine frames. Withdraw the gauge fully and check the tilt and support throughout the remaining observation window.

The centered and missed-knot examples test the scoring separately. Start each robot demonstration from the empty hanger with the pot on the table.

## Run the reference

Set up the [YAM Isaac Lab–Arena station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on a Linux RTX host. The recorded runs used an L40S GPU, Inspect Robots 1.0.0 and inspect-robots-yam 0.38.0. Activate the station environment and install this task from its directory with `uv pip install --python "$(command -v python)" --no-deps .`.

The scripted reference reads the simulated object poses to place the pot and commands the two grippers and arm joints. Choose a fresh output directory and run

```sh
python -m rdb_planter.run scripted --out results/reference
```

For a one-arm control, add `--variant right_only` or `--variant left_only` to the reference command. The working arm grasps the pot by its rim, uses the opposite rim to spread the suspended cords, and lowers the base onto the bottom knot. Both controls use the same load test. A full score in either control is evidence that one arm suffices.

## Run Dream-1

Use the same station and a new output directory, with a configured Dream Labs account. The recorded run used the [Inspect Robots Dream adapter](https://github.com/kairosiann/inspect-robots-dream/tree/b5e936a201cec66c54c297e522c920d98445e339). The model receives the camera images and joint positions. At each chunk boundary the runner blends the next joint targets into the last executed target to avoid a jump.

```sh
python -m rdb_planter.run dream --out results/dream
```

Both commands use seed 0 and the same final scorer. The model receives the three standard YAM camera images and the 14 joint and gripper values. Each run writes its score to `RESULT.json` and its recordings under `media/`.

## Recorded runtime

Scripted reference took 3.4 minutes for 1,088 control steps and scored 1. Dream-1 took 3.5 minutes for 1,200 control steps and scored 0. The package includes the measured versions and hashes in [validation.json](validation.json).

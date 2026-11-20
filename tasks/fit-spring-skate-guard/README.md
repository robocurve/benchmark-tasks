# Pack a skate with its spring guard fitted

> Put the skate in the gear bag.

The bench contains a dry skate, a loose two-piece spring guard, and an open gear bag. A person reaching into the bag can cut a hand on an exposed sharpened runner. One arm holds the toe cup while the other extends the heel cup against both springs and seats it around the runner. The arms then lift the skate by its boot and release it into the bag. The two arms can trade roles. This test uses a blunt surrogate runner and ordinary safe bench handling.

[Jackson Ultima Top Notch](https://jacksonultima.com/pages/top-notch) establishes the commercial two-piece, dual-spring construction. Its [skate blade care guide](https://jacksonultima.com/blogs/news/basics-of-skating-blade-care) explains the use of guards off the ice. The scene starts with the blade dry. The guard follows the two-end capture and spring-extension procedure of the commercial design. The bench geometry is original.

## Setup and physical reconstruction

The included scene builder writes the USDA assets and a dimensional geometry inventory for the same seed-specific initial scene. All dimensions are in SI units. Place the components on an ordinary workbench with the length axes parallel to the line joining the two robot bases. Positions in `skate_guard/config.py` are measured forward and left from the midpoint of that line. The fixed screenshot shows the scene before either arm moves. The two YAM bases are 610 mm apart.

- Use a dry adult-size synthetic skate boot around 272 × 92 × 157 mm including its holder, with a blunt 243 mm runner 4.8 mm thick. The CAD uses a simplified boot profile. The runner tip and heel extend below the plastic holder far enough for the two guard roofs to overlap them. Round all surrogate edges and deburr corners before testing.
- Balance the skate blade-up on two padded wooden work blocks, supporting the toe and cuff at heights of 95 mm and 19 mm. Each block measures 88 × 66 mm in plan with a 6 mm rubber pad included in its height. The toe block is 95 mm left of the skate datum and the cuff block is 76 mm right. Leave the guard inverted, spring-closed and loose on two wooden blocks, each 38 × 65 × 42 mm. The block centers are 61 mm left and 53 mm right of the guard datum.
- Use a 31 mm-wide spring guard with a lower runner channel, one capped toe cup and one capped heel cup. At rest the inner end-stop separation is 227 mm. The heel travels along the centerline up to 54 mm to let both lips clear the 243 mm runner span. Use commercial dual-spring guard halves or make the profiles in `skate_guard/builder.py` from PETG with an overlapping guide to keep the channels aligned. Use equal extension springs along both sides, with spring anchors 32 mm apart laterally. For a printed build, choose springs to provide a combined preload of approximately 1.3 N and approximately 6.9 N at full travel, using a spring scale to measure both values. This corresponds to about 105 N/m combined stiffness. These are design targets in the YAM’s working range. Set the guide travel with stops, and verify that both end lips retain the runner under a 5 N downward hand pull before installing the equipment beside the robot. Use this check on a spare pair to confirm fit and reset the trial guard fully closed and loose on the table.
- Open a soft padded gear bag with an internal floor at least 227 × 379 mm. Prop the far side and ends to a depth of about 187 mm. Unzip and fold the front flap out so the entry lip is 39 mm high. Leave the compartment empty, with the floor in place and no straps over the opening.

Reset by taking the skate out, unclipping the guard, setting it in its spring-closed position on its blocks, replacing the boot blade-up and returning the bag and arms to the marked positions. The fit and packing remain reversible.

## Score

- **1.0** — The bag compartment contains the released skate, the runner is below the walls and fully covered between both retaining end cups, and the skate has settled for 0.5 s.
- **0.5** — The arms have let go of a settled skate with both ends retained and the runner covered. Full packing criteria are still pending.
- **0.0** — All other outcomes. Placing an exposed runner in the bag latches this score for the trial.

Visually confirm that the runner sits down in the channel across its length, both cup roofs overlap their respective ends, and the spring guard stays fitted when the skate is released. Use a ruler to check that the distance between the two interior stop faces exceeds their 227 mm rest spacing by 13–45 mm. In simulation, guard seat height is within 6 mm of the runner datum at both ends, lateral channel alignment is within 6.8 mm, and each runner end lies within 9 mm of its inner stop, with a 4 mm allowance for contact geometry at the end. The calculation checks the individual boot profiles, runner, holder and guard cup bounds against the compartment. Both grippers must be away from the released skate and guard. Neither arm identity nor the order of contact affects the score.

## Run a trial

Use an Isaac Lab environment with `inspect-robots`, `inspect-robots-yam` and the `yam-arena` adapter, then install this task from its folder with `python -m pip install -e . --no-deps`. The fixed trial seed is `1415158438`, with a maximum of 540 actions at 20 Hz (27 seconds). The simulator uses native rigid-body contacts at 240 Hz. Install FFmpeg for RGB capture.

```sh
python run.py --device cuda:0 --physics-device cpu --output runs/demo
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy hold --output runs/hold
python run.py --device cuda:0 --physics-device cpu --no-cameras --policy bare --output runs/bare
```

Read `receipt.json` for the score, `trace.jsonl` for the motion and each step's score, and `final.json` for the final state. The three cameras record the initial frame and successive action frames. Inspect discovers the `skate_guard` environment from the package entry point. `run.py` records native trials only.

The recorded reference reached 1.0 in 497 actions. The stationary control used all 540 actions and scored 0.0. The bare-runner control placed the skate in the bag with the guard left on its blocks and latched 0.0. Both reference arms perform contact work during spring opening and transport.

# Remove trapped hair from a hairbrush

Use the pin-tail comb to lift the trapped hair out of the brush. Leave the brush clear and its pins intact.

## Run and score

Use the Python 3.12 / Isaac Sim 6.1 / YAM runtime pinned in `ENVIRONMENT.json`.
An NVIDIA GPU is required for native rendering. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/clear-hair-from-a-hairbrush/run.py --mode demo --out /tmp/brush-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/clear-hair-from-a-hairbrush/run.py --mode dream --out /tmp/brush-dream
python tasks/clear-hair-from-a-hairbrush/rescore.py /tmp/brush-demo
```

Dream uses the standard RGB cameras and joint state through Inspect Robots.
Its SDK reads the credential privately. Run one episode per key at a time.
The fixed layout has no random seed or jitter. The limit is 3,000 controls,
150 seconds at 20 Hz. Both modes use identical objects, contacts and hardware.
The scripted policy commands only the robot joints and grippers.

Each run saves its source, a state trace, camera recordings and `run.json`.
Rescoring verifies the frozen source hashes, recomputes outcomes from the
recorded body poses, and checks contact, break and release histories on CPU.
It is an outcome audit, not a fresh physics simulation. Diagnostic camera
strides above 1 are excluded from submission evidence.

## Physical setup and reset

Use a 190 mm plastic pin brush with a 52 x 95 mm head and 95 mm handle.
Its 44 flexible nylon pins form four columns 16 mm apart and eleven rows
6 mm apart. Pins are 1.7 mm in diameter, 16 mm high, with 2.6 mm ball ends.
The handle is approximately 21 mm wide. The separate 200 mm comb has a
plastic toothed section and a 100 mm steel tail tapering to a fine point.
The 120 x 110 mm plastic dish has 20 mm sides. The two wooden rests hold
the comb above the table for pickup. Their dimensions are in `task.json`.

Table height is 747.5 mm. In the robot frame, X points forward and Y left.
The brush-head centre is (300, 15) mm and its handle points along +Y.
The mat centre is (300, 75) mm, comb pickup centre (430, -100) mm and dish
centre (430, 150) mm. The comb tail points along +Y. Match the setup photo.
Ordinary silicone covers are 2 x 18 x 24 mm on each finger, with 40 N
actuator limits. Neither the brush nor comb is fixed to the table.

Prepare ten equal 0.2 g synthetic-hair locks, each 75 mm long. Loosely
catch each around one pin and spread them across the head as photographed;
do not tie or glue them. Before a trial all ten locks must be caught among
the pins. Restore this layout with intact pins after each trial. The hair,
brush and comb are reusable; replace any damaged item before resetting.

## Judging

Award 1.0 for at least 90% of the original hair removed and remaining clear
for three seconds after both hands release. Award 0.5 for at least 50%,
0.25 for at least 25%, otherwise 0. Breaking or detaching a brush pin
scores 0. Only hair loosened with the comb counts. Directly picking still
trapped hair by hand earns no credit for that hair; collecting it after
loosening is allowed. The dish is convenient, not a required target.

Use the video to check tool use and undamaged pins, and a 0.1 g scale to
weigh the removed hair. Hair still caught on any pin is not removed.
The simulation counts ten equal-mass locks and checks their whole
centrelines are outside the bristle region. It records actual comb contact
at release of a snag, so touching hair earlier and later shaking it loose
is insufficient. Native hand contact excludes direct removal before loosening.

## Material approximation and assets

Each flexible lock groups fine fibres into a 1.4 mm contact envelope.
A passive 10 mN breakaway snag approximates fibres caught on a pin; that
value is an uncalibrated simulation parameter. Pins bend and can break.
The hair is free after its snag breaks. No cleared-hair pose is assigned.

Closing fingers must physically enclose a handle before a compliant
native pinch forms. It releases on opening, 8 mm slip, or loads above
18 N / 0.24 Nm. Grip reaction loads on the robot are omitted. These
approximations apply equally to the demo and Dream.

All task geometry is original procedural work under the included MIT
license. The robot, table, runtime and cameras come from the pinned YAM /
Isaac environment; no third-party task meshes, scans or textures are used.

## Recorded trials

- Demo: **1**, 1960 controls / 98 simulated seconds; 37.4 wall minutes including startup.
- Dream: **0**, 3000 controls / 150 simulated seconds; 68.8 wall minutes including startup.

Both completed recordings preserve every native frame at 20 fps. The uploaded
videos show two simultaneous judging views. CPU rescoring agrees with both
results, and the executed Python sources and demo plan match the submission.

Dream made 62 requests across 62 server session(s). The official adapter recovered expired sessions; this is not an uninterrupted single-session run.
The model used the official loopback bridge over SSH; its credential remained
on the client computer. Adapter versions and the public request transcript
are in `evidence/`. No server session IDs or credentials are published.

To audit the published traces on CPU with NumPy and SciPy:

```sh
python tasks/clear-hair-from-a-hairbrush/replay_evidence.py demo
python tasks/clear-hair-from-a-hairbrush/replay_evidence.py dream
```

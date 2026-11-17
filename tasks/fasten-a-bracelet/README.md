# Fasten a bracelet on a wrist form

Fasten the bracelet around the wrist form, then let go and leave it secure.

Helping someone fasten small jewellery requires holding a flexible band, aligning its clasp and closing it without dropping either end. This trial uses a rigid wrist form.

## Setup and reset

A 220 mm long, 54 mm diameter rigid silicone wrist form is fixed horizontally on two 25 x 80 x 25 mm rigid supports. Below it, a 195 mm exposed length, 10 x 2 mm leather bracelet has two 10 x 12 x 7 mm metal clasp housings, initially 154 mm apart, with the leather ends folded back. The male end has a 3 mm exposed, 2.2 mm square pin beyond a 3 mm long, 5 mm square stop collar. The female has a 4 mm square bore and a spring detent at 2.5 mm entry. Each housing sits freely on a fixed 6 x 10 x 10 mm acrylic rest. Both hands start open. CLASP.md specifies the equivalent mechanism and fixed reset shown in the setup image.

See the [fabrication files and parts list](fabrication/BUILD.md) for the complete spring-detent build. See [CLASP.md](CLASP.md) for the physical mechanism, all dimensions, fixed positions and repeatable reset. The setup image shows the fixed starting scene. No material is consumed unless damaged.

## Run

Use the Python 3.12 / Isaac 6.1 / YAM versions pinned in `ENVIRONMENT.json`. Native cameras require an NVIDIA GPU. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/fasten-a-bracelet/run.py --mode demo --out /tmp/fasten-a-bracelet-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/fasten-a-bracelet/run.py --mode dream --out /tmp/fasten-a-bracelet-dream
python tasks/fasten-a-bracelet/rescore.py /tmp/fasten-a-bracelet-demo
```

Fixed layout; no random seed. Limit: 1,000 control steps (50 seconds at 20 Hz). Both modes use the same scene, physics, instruction and reset. The demo uses bounded 14-element joint/gripper commands. A scripted alignment controller reads measured clasp poses and moves the right arm to align the pin with the bore; this is privileged feedback for the feasibility demonstration. It never moves or attaches a clasp directly. Dream receives no clasp poses and uses the same physical objects. Dream uses the standard three RGB cameras and robot joint observations. Run one trial per API key at a time.

Dream can use the official SDK credential store or the official BridgeServer/BridgeSession through an authenticated SSH loopback tunnel. The latter keeps its API key on the client computer. No secret belongs in trial artifacts.

## Judging

1.0 if the clasp is closed, the intact bracelet surrounds the wrist form within 5 cm of its midpoint, and both hands have released it for three seconds. 0.5 if it is closed around the form but still held. 0.25 if the pin has entered the socket by at least 1 mm but has not caught. Otherwise 0. A broken, overstretched or off-wrist bracelet scores 0. Inspect the clasp and released bracelet with photographs and a ruler.

Scores describe the final state, including the recorded hands-clear hold. In the segmented simulation, an attachment gap over 4 mm is treated as band damage and invalidates the trial; it is not credited as extra reach. Later damage invalidates earlier progress. Review all views before judging; an occluded clasp or a hand still supporting the bracelet is not evidence of success.

## Simulation

The band and both clasp ends are freely moving native rigid bodies. The leather attaches behind each housing, leaving the pin and socket unobstructed. Passive joints approximate leather bending, with a broad bending range so the folded reset can straighten around the wrist. Native position/velocity solver iterations are 128/32. Finger collisions and friction grip the small metal housings and transmit reactions to the arms. There are no gripper attachment constraints, proximity captures or pose-following clasp parts. The female housing has colliding walls and a real bore; the square pin must physically enter it. A reduced spring detent retains the pin only after at least 2.5 mm insertion, with a solid collar stopping travel at 3 mm. It allows axial twist and up to 8° swing, consistent with the loose pin and bore. Native joint-break events invalidate the trial. It does not pull the ends together. Slipping, missed insertion and catch breakage remain possible.

The segmented band, friction and spring-detent forces are engineering approximations, not measured jewellery properties. The native integration uses 240 physics steps per second. The 30 N catch break threshold is approximate. The fabrication files specify the two leaf springs, their stock source and formed profile, grooved pin, housing, fasteners and repeatable engagement check. The reduced catch omits those internal features; no physical build or force calibration is claimed.

## Evidence

`rescore.py` independently recomputes insertion, winding, joint integrity and release from the recorded native body poses. It verifies frozen-source hashes and every trace measurement. It does not re-simulate PhysX; repeat the native command to test dynamics. A CPU preview cannot demonstrate this contact-driven task.

`run.json` records result, wall time and source hashes. The trace records each ordinary applied command and measured state. Uploaded videos preserve every recorded frame at 20 fps; two judging views are placed side by side. Published compressed traces, summaries and audits are in `evidence/`, with no private server identifiers. Visual review is required in addition to the numerical audit.

For CPU-only scoring of the published evidence, install NumPy and SciPy:

```sh
python tasks/fasten-a-bracelet/replay_evidence.py demo
python tasks/fasten-a-bracelet/replay_evidence.py dream
```

## Recorded revision trials

- Demo: **1.0**, 900 actions / 45 simulated seconds; 664.4 wall seconds including startup.
- Dream: **0.0**, 1000 actions / 50 simulated seconds; 913.6 wall seconds including startup.

Dream completed 21 successful requests across 21 server sessions, with 41 total attempts including 20 expiry recoveries. The request transcript preserves timing and attempt counts. Server expiry recovery did not reset the task, but this is not an uninterrupted single-session baseline.

Both runs use identical runtime physics, policy cameras, instruction and reset. Independent scoring agrees with both results. These trials replace the earlier submission evidence.

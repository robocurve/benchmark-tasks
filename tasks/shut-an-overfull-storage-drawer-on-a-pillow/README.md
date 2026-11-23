# Shut a storage drawer on a pillow that stands higher than the drawer: hold it down, push the drawer in, get the hand out

Close an open drawer containing a pillow that rises above its opening. Keep the whole pillow inside and withdraw both hands as the drawer closes.

## Instruction

> The pillow in the open drawer stands higher than the drawer's opening. Shut the drawer completely with the whole pillow inside: hold the pillow down while you push the drawer in, and get your hand out as it closes. Then let go and move both arms away from the drawer.

The instruction above is the exact string used in the recorded model run.

## Start scene

Sterilite 2301 drawer, 432 × 365 × 175 mm outside, frame front x = 390 mm. Drawer initially open 220 mm. Therm-a-Rest Compressible Pillow Cinch Regular: free size 460 × 330 × 150 mm, 325 g. The central bulge stands 20 mm above the 152 mm opening.

Coordinates are millimetres: x forward from the line through the arm bases, y to the robot’s left, z above the table. The two arm bases are at (0, +305) and (0, −305) mm. Both arms start at the standard ready pose with grippers open. Layout and physical parameters are fixed between the demonstration and model trial.

[task.json](task.json) gives the complete start poses, dimensions and materials. [TECHNICAL.md](TECHNICAL.md) lists product sources, estimated parameters, the build procedure and asset licences.

## Successful outcome

Drawer gap ≤5 mm, whole pillow inside, no contact and grippers ≥50 mm clear for 2 s. Removing the pillow or moving the frame >10 mm gives zero.

## Coordinated operation

One hand depresses each bulging section while the other advances the drawer. The frame must take over the compression before the pressing hand withdraws. This is a fixed drawer without a self-closing spring.

The scorer evaluates the physical outcome. It does not award points for using a particular arm or for following the reference script.

## Run and inspect the result

Use Linux with an NVIDIA RTX-capable GPU and the pinned [YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1): Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Follow that runtime’s installation instructions and source its `yam.env` before running these commands from this task folder. Isaac Sim is an external dependency, not part of this package.

```sh
uv pip install 'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0'
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install 'numpy>=1.26' 'scipy>=1.11' 'pillow>=10' 'imageio[ffmpeg]'
uv pip install --no-deps .
python -m rdb_overfull_drawer.run scripted --out runs/scripted
python -m rdb_overfull_drawer.run dream --out runs/dream
```

Configure Dream Labs credentials privately on the run host before running the `dream` trial. Both commands use Inspect Robots and the same scorer in `rdb_overfull_drawer/grading.py`. The scripted controller may read simulator state; it is a feasibility reference, not a policy with the same information as Dream-1.

Outputs: `RESULT.json` (score and diagnostics), `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and `media/grader-trace.jsonl`. Read the final score in `RESULT.json`; the trace explains when each score changed. Dream-1 receives the overhead and two wrist RGB images at 320 × 176 plus standard joint/gripper position readings. The side camera is recorded for review only. Grader state and diagnostic contact forces are not model observations.

Fixed seed 0, zero object jitter, 20 Hz control and 240 Hz physics. Run limit: 1,800 steps (90 simulated seconds). Wall time includes loading and recording and is longer than robot time.

| Recorded trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| Scripted demonstration | 1.0 | 650 / 1,800 | 165.66 s |
| Dream-1 | 0.0 | 1,800 / 1,800 | 458.28 s |

These are individual recorded simulation trials. [validation.json](validation.json) records the run settings, scores and evidence scope. No hardware trial is reported.

Tests without launching Isaac Sim, using the installed YAM Python environment:

```sh
uv pip install pytest
python -m pytest tests -q
```


## Scoring

The scorer is `rdb_overfull_drawer/grading.py`; `task.json` states the same rules in words. It scores the state
at the end of the trial.

- **0** if the pillow is not lying in the drawer (its middle more than 12 mm off the drawer's middle, lifted
  more than 20 mm off the drawer floor or tipped more than 20 degrees), or if the frame was shifted more than
  10 mm on the table (the frame is fixed in the simulator, so this applies to real trials).
- **1.0**: the drawer front within 5 mm of the frame's front face, the pillow in the drawer, no part of the
  pillow outside the frame, nothing of the robot touching the unit or the pillow and both gripper centre points
  at least 50 mm from the unit, for 2 s. The trial then ends.
- **0.75**: shut, but a hand is still on or near it.
- **0.5**: open 90 mm or less (all three bulging parts are under the frame's top).
- **0.25**: open 190 mm or less (the first bulging part is under the frame's top).
- **0** otherwise.

Outcomes: nothing done, or the drawer only pushed, 0 (it stops at 214 mm); pressed, let go, then pushed, 0;
one part held down and fed under, 0.25; all fed under but the drawer not pushed home, 0.5; shut with a gripper
resting on it, 0.75; pillow lifted out and the empty drawer shut, 0; drawer shut then pulled open again, the
score for where it ends; a pad driven into the frame's top by a slam (contact giving way), not shut, at most 0.5.

### Scoring a real trial

1. Tape the frame's outline and the 220 mm start line of the drawer front on the table. Film from the front
   and above.
2. At the end, measure the gap between the frame's front face and the drawer front with a ruler.
3. Look from the front and above: the whole pillow must be inside the drawer, and with the drawer shut no
   fabric may show in the gap or outside the frame.
4. From the video, check that for the last 2 s nothing of the robot touches the unit and both grippers are
   5 cm or more from it.
5. Check the frame against its taped outline (10 mm).

## Reset

Pull the drawer out to 220 mm (a tape mark on the table under its front). Take the pillow out, shake it back to its full thickness, lay it in the drawer long side front to back and tuck its corners inside the walls. Check the frame is against its stop and clamped. Arms at their ready pose, grippers open. Nothing is consumed.

For a simulation reset, start a new run with a fresh output directory. The adapter reloads the fixed scene, resets velocities and joints, and clears the scorer’s event history. The physical preparation and replacement checks above apply before every real trial; estimated properties must be measured on the purchased objects before hardware replication.

## Simulation model

The pillow is a loose base plus 16 spring-mounted rigid pads, drawn as one fabric surface. It is not cloth or a volumetric foam simulation. Pads compress vertically; a pad partly under the frame is held down as a whole. Wrinkling, local bulging and cover snagging are absent; stiffness and friction are estimates.

See the full `fidelity` field in [task.json](task.json) and [TECHNICAL.md](TECHNICAL.md) for the collision geometry and parameter sources. The demonstration establishes solvability in this simulation; it does not measure correspondence to real hardware.

## Files and licence

| File | Purpose |
|---|---|
| `task.json` | Task definition and submission fields |
| `rdb_overfull_drawer/fixed_scene.py` | Fixed instruction, layout and run settings |
| `rdb_overfull_drawer/adapter.py` | Inspect Robots embodiment and sensor/scorer integration |
| `rdb_overfull_drawer/run.py` | Scripted and Dream-1 trial entry point |
| `rdb_overfull_drawer/scripted.py` | Feasibility controller |
| `rdb_overfull_drawer/grading.py` | Scoring implementation |
| `validation.json` | Recorded scores and settings |
| `TECHNICAL.md` | Dimensions, sources, modelling details and real-trial preparation |
| `tests/` | Existing scene, controller and scoring tests |

Task-specific source and generated geometry are provided under [MIT](LICENSE). No third-party product meshes or textures are bundled. Robot, station and simulator dependencies retain their upstream licences; see the asset list in [TECHNICAL.md](TECHNICAL.md).

The submission card uses short title/setup/fidelity fields to meet the board’s length limits. [REFERENCE-SPEC.json](REFERENCE-SPEC.json) retains the full specification used by the recorded trials; [TECHNICAL.md](TECHNICAL.md) explains its dimensions and modelling assumptions. The instruction and scoring are unchanged.

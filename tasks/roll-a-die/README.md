# Pick up and roll a die

Category: **Societal relevance**. Scorer version: **roll-a-die-v2**.

Status: the v2 scoring correction passes 33 scorer tests, 10 bridge tests and
3 recorded-evidence replay tests.
Rescoring the retained measured trajectories gives demo **1.0** and Dream **0.0**.
The demo reaches 133.207 degrees maximum tilt excursion from its verified
free-flight reference (164.434 degrees cumulative motion) and one second of
rest, completing at 9.30 simulated seconds. Dream uses its full 60-second
allowance, with 25 requests in one continuous session and no retries.

These are recorded physics runs re-evaluated by v2, **not new physics trials**.
The source scene and policy are unchanged. Two attempted VM restarts for fresh
repeat trials failed because regional GPU capacity was unavailable. Media upload
is pending. The remaining review concern is whether the visual cross-check
meets the guide's simple-tools scoring criterion; telemetry validation alone
does not establish that. No competition PR has been opened.
See [JUDGING.md](JUDGING.md) for the reviewer packet and
[VALIDATION.json](VALIDATION.json) for the versioned replay results and hashes.

The task uses a 16 mm, 5 g blue blank die surrogate on the calibrated YAM station.
It tests a small-object grasp followed by an uncontrolled tumbling release. It
does not test a numbered outcome, fairness or random-number generation.

## Environment

Use the competition's Brev simulation launchable and its Isaac Lab-Arena
environment. The development trials used these deployed versions; commit IDs
below are abbreviated:

| Component | Version or revision |
|---|---|
| YAM Arena adapter | `0059d02` |
| Isaac Lab-Arena | `c7edf3a9` |
| Isaac Lab | `91a0906` |
| Isaac Sim | `6.1.0.0` |
| Inspect Robots | `0.60.0` |
| Inspect Robots YAM | `0.37.0` |

The fixed station uses `object_jitter=0`, control at 20 Hz, PhysX at 240 Hz,
and Dream camera frames at 320 x 176 pixels. Seed: 0. The cube's centre starts at
world `(0.20, 0.16, 0.7555)` metres. These settings are shared by the scripted
demonstration and Dream-1. Record deployed revisions alongside each result;
package version numbers alone do not identify an editable checkout.

## File map

| File | Role |
|---|---|
| `task.py` | Defines the fixed scene, cube and task instruction. |
| `scoring.py` | Turns measured cube and gripper motion into pickup, release, tumble and rest credit. |
| `demo_policy.py` | Scripts joint targets for the demonstration and provides its offline motion check. |
| `run.py` | Connects a policy to the simulator, records measurements and saves evaluation artifacts. |
| `dream_bridge.py` | Batches observations over the SSH tunnel before each model call, keeping the Dream credential on the operator's computer. |
| `replay.py` | Recomputes the score from a saved trajectory without running the simulator. |
| `test_scoring.py` | Exercises the scorer with synthetic valid and invalid trajectories; these tests do not prove physical feasibility. |
| `test_bridge.py` | Checks observation ordering, retries, real HTTP transport and equivalence with the Dream client's observation window. |
| `task.json` | Holds the submission description, measured results and uploaded-media references. |
| `JUDGING.md` | Explains the review criteria, visual audit, limitations and comparison examples. |
| `VALIDATION.json` | Records v2 replay scores, input hashes and the distinction from fresh physics runs. |
| `evidence/*.jsonl.gz` | Anonymous measured trajectories, compressed without changing their contents. |
| `test_replay.py` | Independently checks the retained input hashes and three recorded outcomes. |

## Walk through one trial

1. Read `task.py` for the object and fixed starting pose, then `scoring.py` for
   the measured conditions. The cube must be picked up, released, tumble at
   least 90 degrees away from its verified free-flight reference orientation,
   and come to rest on the table. Repeated small rocking cannot add up to credit.
2. Run the tests and the offline motion check below. The latter checks reachable
   poses, joint limits and target speeds; it cannot establish that the fingers
   will grip or release the cube in simulation. The full test suite requires
   NumPy, Inspect Robots, the Dream integration and `dreamlabs-client`; the
   motion check also requires the YAM adapter. These checks need no simulator
   or Dream credentials.
3. Activate the launchable's Python environment and run a short GPU trial from
   the repository root. Give every attempt a fresh log directory; the runner
   refuses to overwrite an existing trajectory.

```bash
source ~/IsaacLab-Arena/yam.env
python -m unittest discover -s tasks/roll-a-die -p 'test_*.py' -v
python tasks/roll-a-die/demo_policy.py --check

python tasks/roll-a-die/run.py --policy demo --max-steps 200 \
  --seed 0 --log-dir artifacts/die-check-new
```

4. Read `score-summary.json` and `runtime.json`, replay the numeric audit, and
   watch the recorded camera views. Check whether the fingers grasped the cube,
   whether release was detected, and how much rotation occurred while free.
   A score of 0.25 establishes only the pickup stage.

```bash
python tasks/roll-a-die/replay.py artifacts/die-check-new/trajectory.jsonl
```

5. Adjust the demonstration's motion in `demo_policy.py`, rerun the offline
   check, and make another recorded GPU attempt. The current candidate uses a
   gentle 15 mm target jaw gap around the 16 mm cube, then an upward stroke and
   wrist rotation about world X with delayed finger opening. It only commands
   arm and gripper joints; it never moves the cube directly. Trial 004 reached
   full instrumented credit and passed visual review. Keep the scorer and scene
   fixed when comparing policies; changing the motion creates a new policy
   version to evaluate.
6. Run the longer demonstration candidate and the Dream-1 evaluation on the
   same setup. These limits are ceilings; a trial can end earlier.

```bash
# Up to 30 seconds of simulation/control time.
python tasks/roll-a-die/run.py --policy demo --max-steps 600 \
  --seed 0 --log-dir artifacts/die-demo-new

# Up to 60 seconds of simulation/control time; bridge setup is below.
python tasks/roll-a-die/run.py --policy dream --max-steps 1200 \
  --seed 0 --dream-server-url http://127.0.0.1:8210 \
  --log-dir artifacts/die-dream-new
```

With the Dream SDK credential configured securely on the VM, omit
`--dream-server-url` to use the standard direct Dream integration. The retained
clean baseline used the bridge; direct-on-VM execution remains untested.

To keep the Dream credential on the operator's computer, run this task's bridge
there from the repository root and forward the VM's loopback port over SSH.
The runner's `--dream-server-url` expects this bridge's batched endpoint:

```bash
# Once, on the operator's computer, using Python 3.12:
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install \
  'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339' \
  'dreamlabs-client==1.0.14'

# On the operator's computer, in the Dream client environment:
DREAM_LOG=off python tasks/roll-a-die/dream_bridge.py --host 127.0.0.1 --port 8210

# In a separate terminal; replace INSTANCE_SSH_ALIAS with the configured alias:
ssh -N -T -o ExitOnForwardFailure=yes \
  -R 127.0.0.1:8210:127.0.0.1:8210 INSTANCE_SSH_ALIAS
```

The bridge uses the locally configured Dream credentials, sends observations to
Dream and returns actions to the VM. Its HTTP interface has no authentication,
so keep both listeners on loopback and verify the remote binding. Keep the local
computer and tunnel available throughout the run, then close the bridge and
tunnel. The supported integration revision is
`inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339` with
`dreamlabs-client==1.0.14`.

The task bridge queues every observation and replays the complete ordered batch
on the local Dream session immediately before each model call. This reduces SSH
HTTP round trips while preserving the same nine-frame model window, measured
states, prompt, simulation timestamps and commanded joints. It does not discard
observations or alter the policy's action chunks. Its ten tests passed,
including an actual HTTP round trip and comparison with the Dream client's real
observation-window implementation.

The runner invokes Inspect Robots in-process and registers an audited YAM
embodiment plus the custom demonstration. The demonstration extends the stock
Cartesian planner to interpolate wrist rotation about world X. The task file
intentionally refuses to report success without its measured-state audit.

## What development has shown

These are development attempts, not repeated evaluations of one frozen policy.
Their changing grasp/release settings must not be treated as a success-rate
estimate.

| Trial | Change and observed result | Final score | Measured wall time | Estimated Brev runtime cost |
|---|---|---:|---:|---:|
| 001 | Full finger closure produced vibration; pickup was not established. | 0.00 | 72.533 s | $0.066 |
| 002 | A gentle 15 mm jaw gap enabled pickup; a complete roll was not achieved. | 0.25 | 48.979 s | $0.045 |
| 003 | Delayed opening produced a valid detected release; cumulative motion was 30.274 degrees and v2 maximum excursion is 29.656 degrees, below its 90-degree threshold. | 0.25 | 46.005 s | $0.042 |
| 004 | Higher upward stroke and rotation about world X produced 164.434 degrees of free tumble and continuous rest. | 1.00 | 321.206 s | $0.293 |

Trial 004 completed after 186 actions, or 9.30 seconds of simulation. Pickup was
credited at 4.60 seconds, verified release at 8.00 seconds, and the final rest
lasted one continuous second. No invalidation was recorded. Replaying the audit
matches the 1.0 score, and visual review confirms the cube enters free flight
and lands.

The displayed running rate is $3.28/hour ($3.23 compute plus $0.05 storage).
Trials 001–003 each account for roughly 4–7 cents; trial 004 accounts for about 29 cents because a parallel
simulator process encountered a shader-cache miss during startup. These are
prorated estimates from the runner's elapsed times, not billing statements.
Cold setup took approximately 40 minutes and is excluded, as are time between
attempts, storage retained after shutdown and Dream API charges.

The first Dream trial completed 1,200 steps / 60 seconds of simulation in
664.804 seconds of wall time and scored 0.0. However, its 25 action chunks
required 49 inference attempts across 25 distinct sessions, with 24 session
expiries. It is an infrastructure-affected result, not a clean model baseline.
The batched bridge addresses the per-observation transport overhead without
changing the task or model inputs. Dream rerun 002 completed in 197.996 seconds
of wall time, scoring 0.0 after the full 1,200 actions. All 25 requests succeeded
on their first attempt in one continuous session, with no action rows outside
joint limits. The model reported 7.646 seconds of inference work; this is not a
Dream billing measurement. Estimated Brev runtime cost was $0.180, excluding
Dream API charges. Use this second run as the submitted model baseline.

Control time is not GPU wall time: simulator startup, rendering and model calls
also cost time. Measure actual elapsed time and account usage instead of deriving
spend from the number of control steps. Stop the Brev instance when finished;
retained disk storage continues to be billed.

## Scorer v2 and evidence provenance

The original scorer added angular travel after release. That allowed synthetic
small back-and-forth rocking to accumulate enough credit. V2 requires the
**maximum tilt excursion from the verified free-flight reference to reach 90
degrees**. The reference is the first pose of the uninterrupted verified free
history, or the release-confirmation pose if no earlier interval qualifies.
Cumulative motion remains a diagnostic; a full revolution can still qualify
when the cube returns to its starting orientation.

Eleven new regression/boundary tests cover small rocking, partial turns and
returns, full revolutions, release anchors, yaw, pending free-flight history and
slow motion. There are 33 scorer, 10 bridge and 3 recorded-evidence tests. Offline
replay of unchanged historical measurements gives:

| Retained trial | V2 score | Maximum tilt excursion | Cumulative motion |
|---|---:|---:|---:|
| Partial demonstration 003 | 0.25 | 29.656° | 30.274° |
| Full demonstration 004 | 1.00 | 133.207° | 164.434° |
| Clean Dream baseline 002 | 0.00 | 0° | 0° |

The original logs, scores and runtime files have not been overwritten. Derived
results and input SHA-256 hashes are recorded in `VALIDATION.json`. Anonymous
measured trajectories are included in `evidence/`; detailed replay CSVs are local
artifacts. The gzip files preserve their original v1 `metrics` fields for
provenance; replay ignores those fields and recomputes v2 from measured state.
Existing recordings show the same measured
motion. The first full-score time remains 9.30 seconds, so this rescoring does
not depend on motion that would occur after v2 success termination. A fresh
validation batch would add repeatability evidence and remains pending GPU
capacity. The guide does not explicitly require fresh physics after a scorer-only
change; rescoring the final recorded evidence is already complete.

Reproduce all three recorded outcomes from a fresh clone, without Isaac Sim,
Dream credentials or GPU access (Python and NumPy are sufficient):

```bash
python tasks/roll-a-die/replay.py tasks/roll-a-die/evidence/demo.jsonl.gz
python tasks/roll-a-die/replay.py tasks/roll-a-die/evidence/dream.jsonl.gz
python tasks/roll-a-die/replay.py tasks/roll-a-die/evidence/partial-turn.jsonl.gz
python -m unittest discover -s tasks/roll-a-die -p 'test_replay.py' -v
```

## Audit and score

Each run writes the usual Inspect report and frames, plus:

- `trajectory.jsonl`: simulator time, die pose, velocities and AABB, measured
  joints, gripper TCP positions/rotations/openings, and score diagnostics at
  every control step.
- `score-summary.json`: final score, event times, tumble angle and invalidation.
- `setup.png`: the actual settled initial scene, saved before any policy action.
- `runtime.json`: package versions, settings, elapsed wall time and task-source
  SHA-256 hashes.
- `task-source/`: a copy of the Python source used by the current runner. Use
  this snapshot when replaying a trial after the working scorer changes.

The scorer awards 0 for no verified pickup, 0.25 for pickup, 0.5 for a verified
release with a maximum tilt excursion of at least 90 degrees from its verified
free-flight reference, and 1.0 when the rolled cube
also rests continuously for one second. Pickup requires the cube's bottom to
stay at least 30 mm above the table for 0.20 seconds, with a nearby closed hand
and limited cube speed. Release uses measured jaw clearance, airborne position
and acceleration consistent with gravity. Rotation counts only over verified
free intervals; rotating while held, placing the cube, or spinning it about
world vertical cannot earn roll credit.

A late invalidation erases earlier partial credit, so the task uses the final
score, never the best historical score. The final task score is also recorded
in the evaluation JSON at `samples[0].epochs[0].task_score`. An evaluation's
`status: "success"` means it finished without an evaluation error; it does not
mean the cube completed the task.

The runner retains the action bounds and per-step limiter. Its maximum changes
are derived from the published YAM joint/gripper velocity limits at 20 Hz and
applied equally to Dream-1 and the scripted demonstration. The default generic
5%-of-range limit would change the tested release trajectory.

Table containment uses the whole die AABB inside x=[-0.17, 0.89] m and
y=[-0.63, 0.63] m, giving 20 mm clearance from the table's horizontal edges.
A drop below z=0.7325 m is invalid. See `scoring.py` for all numeric predicates.
The measured TCP/opening predicates are conservative contact proxies, not direct
contact-force measurements. Inspect the videos as well as the numeric audit
before claiming a valid demonstration. The original scorer remained fixed throughout the recorded physical trials.
The v2 revision is explicit and preserves historical evidence. Preserve the
current scorer and scene for future evaluations; version later changes.

## Media and submission

Render the evaluation JSON file with
`inspect-robots video path/to/evaluation.json --out path/to/media --fps 20`.
This emits one H.264 MP4 per camera. Select or combine views that visibly show
the grasp, free tumble and final rest; keep the complete trial and its score.
Remux the final MP4 with `ffmpeg -i input.mp4 -map_metadata -1 -c copy -movflags
+faststart output.mp4` for browser playback. Each uploaded video must be at most
50 MB, and the setup image at most 10 MB.
Keep names, usernames, paths and team identity out of the task text and images.
Use the runner's actual `setup.png` after visually verifying it. A full-score
scripted demonstration is required if Dream-1 does not earn full score. Trial
004 reaches the v2 threshold when its measured trajectory is replayed; its
recording passed visual review. Completed media uploads remain pending.

Update `task.json` with measured results, verified fidelity and the media ID
returned by `tools/upload_media.py`. Submit one task folder in one pull request
to `robocurve/benchmark-tasks`. Do not use placeholder or invented scores/media.

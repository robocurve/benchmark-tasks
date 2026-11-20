# Service a manual lifejacket inflator for repacking

Prepare a dry manual lifejacket inflator for handoff. The original spent cartridge and a service reject have separate destinations. Both stock pieces can engage the socket, so an assembly that holds its own weight can still end with a red window. Full credit combines stock disposition, mechanical retention and green indication during a released one-second hold. Partial credit recognises a stable, out-of-service unit with the suspect spare quarantined. See WORKCELL.md for the exact setup, rebuild dimensions, real-world analogy and grading tolerances.

## Run in the official walkthrough environment

Use the original calibrated YAM station and the Dream adapter specified in runtime-lock.json. Activate that environment first. The official environment, robot assets and cameras come from the walkthrough. The task folder provides the bench, grader, demonstration and verification. Run from this folder and give each episode a new output directory. The verifier requires the same environment and, for video output, FFmpeg with libx264. It reads every saved physics interval and uncompressed camera frame. [Official walkthrough](https://tasks.robocurve.org/walkthrough) · [Walkthrough setup source](https://github.com/ttpierson/YAM-inspect-Isaac/blob/d825f0c51724f750f26e8c3f5620a14b5b0261b0/launchable/setup.sh)

```bash
python runtime_check.py
python run.py --mode setup --gpu auto --log-dir /tmp/pfd-setup
python verify.py /tmp/pfd-setup --media-dir /tmp/pfd-media
python run.py --mode demo --gpu auto --log-dir /tmp/pfd-scripted
python verify.py /tmp/pfd-scripted --media-dir /tmp/pfd-media
```

For an authorised Dream evaluation, configure the official SDK credential locally. A configuration-only invocation requires its own output directory. It does not open an inference session. A real invocation sends the instruction, original camera observations and robot state to Dream. Cloud recording is disabled.

```bash
python run.py --mode dream --config-only --log-dir /tmp/pfd-config
python run.py --mode dream --gpu auto --log-dir /tmp/pfd-dream
python verify.py /tmp/pfd-dream --media-dir /tmp/pfd-media
```

The station supplies three original 320 × 176 RGB images and joint feedback at 20 Hz. The scripted demonstration observes each known stock position with the right wrist, classifies the seal faces and selects one of two preplanned joint paths. The fixed evaluation layout has A intact and B pierced. The demonstration uses no simulator body poses or contact values for control. The Dream branch instantiates the official model policy, with 32 preroll actions, 48 rows per request and a total episode limit of 2,800 actions (140 s). Only the grader sees the simulator's body poses and contact stream. There is one episode, zero object jitter and no hand-guided state editing. Videos show the overview above both wrists in a shared timeline.

The dimensional geometry and demo are original to this task. The manual illustrations and manufacturer CAD are not bundled. The official runtime supplies the YAM robot and its upstream i2rt assets (MIT), IsaacLab-Arena (Apache-2.0) and Isaac Sim (NVIDIA terms). Install them under their respective upstream terms. [YAM asset attribution](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730) · [IsaacLab-Arena](https://github.com/isaac-sim/IsaacLab-Arena)

## Recorded evaluation

Dream-1 scored 0.0 in one fixed-layout episode of 2,800 actions (140 s). The scripted demonstration first reached 1.0 at action 2,688 and remained at 1.0 through action 2,724 (136.2 s). Verification replayed every action interval from recorded body poses and native contacts. All three starting images matched across episodes.

In separate development controls, exchanging the loose seal conditions changed the visual demonstration's selection and still gave 1.0. Repeating the original choice after that swap physically locked the pierced spare, left the window red and scored 0.0. Isolating only the pierced spare with the unit safely out of service scored 0.5. These controls used separate resets.

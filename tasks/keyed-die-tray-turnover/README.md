# Transfer bare dies between keyed trays for backside inspection

The fixture exposes the backs of four loose training dies while preserving an asymmetric pocket pattern. At reset the receiving tray sits on a pedestal one quarter-turn from its loading orientation. A controller can use the clipped corners and their marks to match the trays. In the reference demonstration, a wrist camera selects one of two heading corrections. A fixed sequence of ordinary YAM joint and gripper commands executes the remaining handling. No object pose, credit, contact reading or simulated attachment is given to either policy.

NXP describes matching two clipped tray corners and using a tray turner to prepare dies for backside inspection. This fixed bench uses two custom carriers on a common shaft, two removable screws holding the source tray to its carrier, a passive spring catch, rotating handles, four separate silicon coupons and two separate 50.8 mm training trays. Its movements and final pocket map are the scope of the test. [NXP application note, Figure 17](https://www.nxp.com/docs/en/application-note/AN10706.pdf#page=17)

## Run and score

Activate the pinned YAM/Arena environment from the official simulation walkthrough. From this task directory, run the commands below, choosing a new output directory for each invocation. The runner checks the installed versions and prints errors if the renderer or native contact monitor fails. Use one process and one episode per reset. Seed 0, no object jitter. Both policies use the calibrated stock YAM station, original three 320 × 176 RGB cameras and joint state at 20 Hz. PhysX integrates at 480 Hz for the thin coupons. The final trial limit of 120 seconds leaves about 57 seconds beyond the demonstrated manipulation for inspection and retries.

```bash
python run.py --mode demo --log-dir /tmp/tray-demo
python verify.py /tmp/tray-demo
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q test_grading.py
```

For an offline inspection of the formal model configuration, run the command below. The configuration-only path ends before the renderer, policy client or API is initialized. A separately authorized paid run uses the same command with a fresh output path and with the configuration-only option removed.

```bash
python run.py --mode dream --config-only --log-dir /tmp/tray-model-settings
```

The two-minute endpoint is scored by the live task file. The demonstration ends after its 1,251 commands. `physics.npz` privately archives native transforms and contact counts at the control boundaries; `replay.py` recomputes the score. The scoring window includes 21 samples over the final second. The saved initial observation accounts for the extra frame in an exported 20 fps recording.

The setup dimensions, commissioning measurements and cost basis are in the fabrication guide. The full real-world and simulation grading instructions include both keyed maps and a physical check for a perched coupon. [Fixture drawing](fabrication/fixture-drawing.pdf) · [Fabrication](fabrication/BUILD.md) · [Grading](SCORING.md) · [Source revisions](runtime-lock.json)

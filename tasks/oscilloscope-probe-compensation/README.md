# Compensate a passive oscilloscope probe

> Use the supplied trim tool to adjust the 10× probe until the oscilloscope shows a square wave with flat tops and bottoms. Then put the tool down.

**Category:** Other, laboratory research automation. Electronics labs compensate a passive probe when first pairing it with a scope input. Without this step, the measurement can show rounded edges or overshoot introduced by the probe itself, misleading a researcher debugging a prototype circuit. The robot picks up the stock trim tool, keeps its blade seated in a recessed slot, and uses the live trace to guide small turns.

## Setup

Use the calibrated bimanual YAM station. Coordinates are metres, with x forward from the arms, y left, and z above the floor. The table surface is at z = 0.7475 m. Both arms begin at READY with open grippers. The table below describes the recorded simulation. For a physical trial, the [OWON kit](#lower-cost-physical-trial) is a lower-cost alternative.

| Object | Dimensions, material, and starting position |
| --- | --- |
| Tektronix TDS2024B oscilloscope | 326.3 mm wide × 158.0 mm high × 124.2 mm deep, 2.0 kg. Molded exterior, glass LCD, metal connectors. Case centre at (0.5471, 0.154, 0.8265), with its front at x = 0.485 facing the arms. The simulation fixes its base. The 5.7 in display is represented by a 116 × 87 mm active panel. |
| Installed Tektronix P2220 probe and ground clip | 1.5 m coax lead in its standard seated CH1 BNC connector. The hooked probe tip and ground clip connect to the scope's built-in reference output and return. The compensation box is drawn with the factory side access facing upward. The stock connector housing, port, shaft and slot must retain their physical access. The simulated trimmer face centre is (0.418, 0.075, 0.816). |
| Tektronix 003-1433 tool | Slim molded insulating grip and shaft, with a metal flat tip. It lies free on the bench; its body reference position is (0.287, 0.140, 0.7525) with its metal tip pointing along −y. The prototype uses a 34 × 10 × 9.6 mm grip, a 2.5 mm diameter shaft and a 2.0 × 0.5 mm blade section. These are declared model dimensions. |

[geometry.py](geometry.py) gives the exact modeled dimensions and poses. [SOURCES.md](SOURCES.md) distinguishes manufacturer specifications from prototype choices. The simulation treats the already connected calibration circuit as fixed. It does not model manipulation of the probe cable.

## Real trial and reset

Set the instrument's CH1 and probe to 10×, with DC coupling. Connect the tip and ground clip to the scope's PROBE COMP output and return as specified in the [manufacturer procedure](https://download.tek.com/manual/071181702web.pdf). Use the built-in nominal 5 V, 1 kHz reference. Display live CH1 alone in Sample acquisition mode. Use a CH1 rising-edge trigger at the midpoint of the two plateaus, 250 μs/div, and 2 V/div with both plateaus visible. Reset by turning the trimmer until both edges are rounded and the measured error is between 27% and 35%, the band containing the simulator's 30.9% reset. Check the reset error using the exported CH1 CSV. Restore the tool to its starting pose and reset the arms. Nothing is consumed.

### Lower-cost physical trial

Use the **OWON SDS1102 benchtop kit** with one bundled switchable 1×/10× passive BNC probe, its hook and ground clip, and the supplied trim driver. On November 19, 2026, the kit was listed at **US$249** and **€219 including VAT**, respectively, before any shipping or additional taxes. The listings identify the included probe and adjustment tool. A USB flash drive is used to collect the score. [Buy from Newegg (US)](https://www.newegg.com/p/0AF-00HU-00010?Item=9SIA305BJG8038) · [Buy from Elektor (EU)](https://www.elektor.com/products/owon-sds1102-2-ch-oscilloscope-100-mhz)

The SDS1102 supplies its own nominal **5 V peak-to-peak, 1 kHz square wave**, has a **1 MΩ input with nominal 20 pF** and displays the live CH1 trace. Figure 3-7 shows the factory adjustment opening on the probe's BNC housing and the driver entering it from the side. The robot still has to seat the loose blade through that opening and keep it engaged as it turns in response to the trace. The probe and tool come with the scope, and its USB waveform export provides the trace for the same score. [OWON product specifications and kit contents](https://www.owontech.com/digital-oscilloscopes/benchtop-oscilloscopes/sds1000-series-super-economical-type-digital.html) · [OWON user manual, printed pp. 9–11, 27–31 and 57–60](https://www.owontechnology.eu/_downloads/45431c4b2958db6f367835ffcbfc183d)

1. **Layout.** Put the 301 × 152 × 70 mm scope (about 1.1 kg) on its feet, with the front facing the arms at x = 0.485 m and its lateral centre at y = 0.154 m. Lock the probe onto CH1 with its stock access opening facing up where the connector permits. Keep the opening and blade approach clear, and route the cable with slack. Hook the tip to the front-panel compensation output and clip the ground lead to its return. Place the free driver at x = 0.287 m, y = 0.140 m at the grip centre, tip along −y, and start both arms in READY. Record the actual port orientation and layout with a reset photo so they can be repeated.
2. **Acquisition.** Set the probe switch and CH1 attenuation to 10×. Use CH1 alone, DC coupling, **Acquire → Sample**, persistence off, normal CH1 rising-edge trigger at the midpoint of the two plateaus, and RUN. Use **500 μs/div** on the OWON and **2 V/div**. Centre the trace vertically so both plateaus and their transients stay visible. Keep these settings fixed during the trial.
3. **Export and score.** With a FAT32 USB drive inserted, select **Utility → Function → Save**, then **Type: Wave**, **Source: CH1**, **Next Page**, **File Format: CSV** and **Storage: External**. Save a fresh waveform after the final hold. Keep the original export. Copy the time and CH1 probe-tip voltage samples into a two-column CSV headed `time_s,voltage_v`, converting the exported units to seconds and volts as needed. If the file stores sample indices, use the actual sample interval recorded for that capture to obtain the timestamps. Preserve every sample in order. Verify that the record spans several periods with successive samples at most 26 μs apart. Run `judge_waveform.py` on that CSV and apply the same video checks and score bands below.
4. **Reset.** Before each trial turn the trimmer by hand until the edges are visibly rounded and the exported trace shows **27–35%** normalized error. Use the same settings as for scoring and return the tool and arms to their start poses.

## Scoring

Grade the CH1 trace at the fixed calibration settings. The evaluator estimates late high and low plateau levels from samples 34–46% of a period after each edge, skipping the next transition. Error is the largest deviation from the corresponding plateau over 2–34% of a period after the edge, divided by the plateau span. Use a fresh waveform with at least two rising and two falling edges, each half-cycle between 0.40 and 0.60 ms, each measured period between 0.95 and 1.05 ms, and a plateau span between 4.0 and 6.0 V. An invalid or missing trace earns zero. See [waveform.py](waveform.py) for exact sample checks.

| Normalized error after adjustment | Credit |
| --- | --- |
| >20% or adjustment absent | 0 |
| >12% to 20% | 0.25 |
| >6% to 12% | 0.50 |
| >3% to 6% | 0.75 |
| ≤3% | 1.0 when the trace and placement conditions hold together for one second |

Use the trial video to confirm that the tool adjusted the trimmer and the scope acquired a live, unclipped CH1 trace throughout. A compensated trace while the tool is still held or in the slot earns 0.75. For full credit, the tool must rest flat on the bare tabletop for one second with the grip centre at 0.01 < x < 0.68 m and |y| < 0.52 m, and both gripper centres more than 8 cm from the centre of the grip. The last partial score remains until that hold completes. Keep acquisition in RUN. After the hold, export the fresh CSV to USB using SAVE/RECALL → Save Waveform → CH1 → File on the Tektronix, or the [OWON menu above](#lower-cost-physical-trial).

```bash
python judge_waveform.py /path/to/CH1.csv
```

This command reports the waveform component. Use the trial video to score the visible tool outcome.

## Fidelity

Native PhysX models the free driver, gripper contact, physical port and slot, and a passive rotary trimmer. An RC divider model converts measured joint position to the trace shown and scored. The port, slot, grip, mass, joint friction and stops are estimates. The 3 mN·m breakaway torque is in the range specified for a comparable miniature trimmer. The fixed cable is visual geometry. The angle-to-capacitance relation remains provisional. [Comparable trimmer specification](https://www.murata.com/~/media/webrenewal/support/library/catalog/products/capacitor/trimmer/t13e.ashx?la=en-gb)

## Reproduction

Use the Python 3.12 environment with Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0, Inspect Robots 0.60.0, FFmpeg and the YAM revision in [pyproject.toml](pyproject.toml). Set `SCOPE_SIM_HOST` to the verified hostname of your simulation machine. Run these commands from the task directory.

```bash
python -m pip install --no-deps -e .
python run_demo.py /absolute/fresh/scope-demo --policy full --device cpu --seed 0 \
  --expected-hostname "$SCOPE_SIM_HOST"
```

The operator sets `OMNI_KIT_ACCEPT_EULA` after accepting applicable NVIDIA terms. Point `TMPDIR` at writable scratch space outside the task source and use a separate evidence directory. `receipt.json` records the native result, wall time and source hashes. The trial limit is 1,600 controls at 20 Hz (80 simulated seconds), with 240 Hz physics. The standard calibrated top and wrist camera videos, setup image and final displayed sample CSV are written alongside the receipt.

Run the authenticated Dream-1 policy through the supplied launcher. It selects the registered embodiment, reads the same instruction and trial limit and retains the Inspect log and calibrated camera frames.

```bash
python run_model.py /absolute/fresh/scope-dream --device cpu \
  --expected-hostname "$SCOPE_SIM_HOST"
```

The native scripted demo scored **1.0 at control 802 of 1,600 (40.1 s)** and recorded 924 controls including its retreat. The final released trace measured 0.40% error, 5.004 V plateau span and 1 kHz. A control trial on the same source inserted the blade and held it without turning. The trace stayed at its initial distortion and scored 0.0. [Validation record](validation.json) · [Recorded waveform CSV](waveform-demo.csv) · [Setup](setup.png) · [Released end state](demo-end.png)

Dream-1 (`dreamlabs/dream-1-beta`, seed 0) scored **0.0** after all 1,600 controls, with no errored trials. The script and model used the same frozen scene and scorer. The recorded wall times on the validation station were 457.51 seconds for the scripted recording and 894.68 seconds for the model trial.

## Asset licenses

Original task code and procedural USD geometry use the included [MIT license](LICENSE). Manufacturer documents were consulted as references. The pinned dependency supplies the I2RT YAM robot and gripper URDF, meshes, USD and collision geometry with its [MIT notice](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt). The task uses the dependency's calibrated station and cameras. NVIDIA simulator terms apply separately.

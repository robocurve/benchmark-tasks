# Runtime dependencies

Use the installed native `yam.env` environment. No task-specific installation,
replacement simulator, hosted-model substitute or dependency upgrade is needed.
The following versions were read from the installed environment during submission
checks. The evaluated Dream integration versions and source hashes are also
preserved in `provenance.json`:

| Component | Version |
| --- | --- |
| Python | 3.12.16 |
| Isaac Sim | 6.1.0.0 |
| Isaac Lab | 17.0.2 |
| yam-arena | 0.1.0 |
| inspect-robots | 1.0.0 |
| inspect-robots-dream | 0.1.0 |
| dreamlabs-client | 1.1.7 |
| PyTorch | 2.11.0+cu128 |
| NumPy | 2.3.1 |
| SciPy | 1.17.0 |
| Pillow | 12.3.0 |
| OpenCV (`cv2`) | 4.14.0 |
| imageio | 2.37.2 |
| imageio-ffmpeg | 0.6.0 |

These are provenance observations, not a command to overwrite an existing
environment. Native YAM geometry, gains, joint limits, station bases and gravity
compensation remain unchanged. The task builds its own furniture, sliders and
continuous floor mesh procedurally; no asset download is required.

Simulation also requires the existing cooperative scheduler, containing
`gpu-scheduler` and `isaac_binding.py`. Set `ROBOCURVE_SCHEDULER_DIR` to its
installation directory. The task always reserves ID `furniture-sliders`, validates
the reservation with `assigned_gpu()`, binds physics to masked `cuda:0`, and binds
the renderer to the assigned physical GPU with multi-GPU rendering disabled.
Numerical/controller debugging uses exclusive mode. Offline interface validation
and the recorded model episode used development mode. No unscheduled simulator
fallback is provided.

CPU tests, raw-state rescoring and saved-model action auditing do not launch a
simulator or use hosted inference. They require the native Python packages above,
but no GPU reservation. The full test suite exercises the stock kinematics through
`yam_arena.analysis` and retains the small `regression-*.json` fixtures that those
tests load.

`tools/prepare_demo.py` uses the existing `imageio-ffmpeg` binary with libx264
and libass for CPU-only media export, plus the installed DejaVu Sans font for
anonymous camera labels. The recorded evidence views are not resized. No encoder
or font installation was performed for this revision.

A newly authorized live model episode requires configured Dream credentials
through the installed client's environment or standard credential file. Keep
credentials out of the task directory. The completed model recording and score
are evidence of the actual hosted attempt; no new paid call is needed to audit or
submit them.

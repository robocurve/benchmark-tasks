# Media provenance

`task.json.media` holds the media ID of the uploaded files. `demo.mp4` and `setup.png` come from the scripted run of this source on the official Isaac Lab-Arena release/0.3.1 environment; `model_run.mp4` is from the previous revision.

- `demo.mp4`: encoded from the frames of the scripted run `LP4_a` named in [VALIDATION.json](VALIDATION.json) (every fourth control step, 5 fps, the `ffmpeg` command in README.md). Score 1.0, independent replay 1.0. It shows the flexible strap loops being pinched, pulled and carried.
- `setup.png`: that run's start-state image (top, left, right and overview cameras).
- `model_run.mp4`: the Dream-1 run listed in VALIDATION.json, on this revision's scene (strap loops) and scorer, 320 x 176 cameras, encoded from its recorded camera streams with `inspect-robots video` and combined into one video.

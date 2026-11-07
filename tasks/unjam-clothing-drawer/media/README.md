# Submission media (local staging only)

Stage these files here with `../submit_media.py`:

- `model_run.mp4`: the full-task Dream-1 run, reported score 0.0.
- `demo.mp4`: the successful v6 scripted demo, reported score 1.0.
- `setup.jpg` or `setup.png`: the initial setup image.

Video and image bytes are ignored by Git. Like accepted PR #46, this task must
upload them to the task board and record the returned media ID in `task.json`.
A local folder or GitHub comment attachment does not create that ID.

Use the existing 90-second full-task prompted run, not the handle-only diagnostic
or the successful apple control trial. Preserve the original recordings.

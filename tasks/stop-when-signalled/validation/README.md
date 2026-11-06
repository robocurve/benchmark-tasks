# Validation records

One JSON file per plan, written by `tests/run_plan.py` during the last full run of `python -m pytest -q` (the
`summary.json` of each plan, unchanged). Each holds the plan, the instruction, the score `task_score` reported, how the
trial ended, its control steps, the quantities `score()` read at the end (`final`), the versions, the GPU, the
wall-clock time, a SHA-256 over the task's code and task.json (`source_sha256`) and of the recorded video
(`video_sha256`). No credentials, session IDs or host names.

| File | Plan | Score | Steps | Wall clock |
| --- | --- | --- | --- | --- |
| `demo.json` | demo | 1 | 569 | 156 s |
| `hold.json` | hold | 0 | 61 | 18 s |
| `ignore.json` | ignore | 0 | 411 | 118 s |
| `late.json` | late | 0 | 543 | 143 s |
| `never_resume.json` | never_resume | 0.5 | 2261 | 324 s |
| `one_arm.json` | one_arm | 0 | 569 | 154 s |
| `dream.json` | dream | 0 | 2400 | 517 s |

The plans were played by `yam_scripted` (the task's `demo()`) through `inspect-robots run`, one process each, on
NVIDIA RTX PRO 6000 Blackwell Max-Q Workstation Edition; seeds are not used (fixed layout, `-E object_jitter=0`). `dream.json` is the one Dream-1 run
(`--policy dream`, the README's command): 50 requests, 63 attempts with retries; it also lists the request
settings, but no session IDs.

`source_sha256` is over the code and task.json as they were for each run; task.json's `scores` and `media` were
filled in after the runs, so they are not covered.

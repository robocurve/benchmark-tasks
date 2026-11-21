# Display-grip negative control

The patch applies to the candidate task.py SHA-256 99cb320e4c920680865a6bbb0ffbea570030d614e521db3892451e43eee4d2ae. It appends an aligned negative-controller grip and read-only contact diagnostics. The original file is an exact prefix: scene, dynamics, scoring functions and helpers remain unchanged. No policy or force measurement is substituted for the classifier. This control is not the qualifying expert demo.

From the task folder in the program launchable:

```bash
printf '99cb320e4c920680865a6bbb0ffbea570030d614e521db3892451e43eee4d2ae  task.py\n' | sha256sum -c -
laptop_control_dir="$(mktemp -d)"
cp -R . "$laptop_control_dir/"
patch -d "$laptop_control_dir" -p1 < controls/aligned-grip.patch
LAPTOP_DEMO=naive LAPTOP_LOG="$laptop_control_dir/contacts.jsonl" inspect-robots run \
  --embodiment yam_arena --policy yam_scripted -E task="$laptop_control_dir/task.py" \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Put the laptop on the laptop stand." --scorer task_score \
  --max-steps 260 --epochs 2 --store-frames --log-dir "$laptop_control_dir/log"
```

Both native control trials ran 260 steps and scored 0.0. In each, opposing fingertip contact latched the handling zero at 8.15 s while measured state remained valid. The normal first-submission naive controller also scored 0/0 but did not establish a detected grip; that result is not evidence of grip rejection. The aligned control above supplies the relevant contact check. Offline controls separately test brushes, supported rests, actual observed descents and catches; those synthetic trajectories are not physical executions.

The diagnostic copy adds contact witness counts and minimum projected opposite-face separation. It neither moves bodies nor changes the candidate classifier's original output keys. A grip is a stated geometric proxy, not measured force or display damage. The physical fixture and hardware behavior have not been validated.

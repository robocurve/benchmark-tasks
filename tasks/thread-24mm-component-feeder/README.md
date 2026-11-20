# Thread a 24 mm feeder and verify indexed pockets

The cold bench scene uses an original 24 mm carrier, three dummy packages in physical pockets at 16 mm pitch, a thin rigid leader, stationary peel-gear envelopes, and an extended button. The physical button advances the tape using a force-based motor approximation. The operational sequence starts with threading the film, followed by three independent button strokes. Flexible film peel and live sprocket engagement remain to be validated. [Opulo loading procedure](https://docs.opulo.io/feeders/6-loading-tape/loading-tape/) · [Opulo 24 mm release](https://github.com/opulo-inc/feeder/releases)

## Physical measurements

Check 20–25 mm of film advance with a side measurement by the peel nip. Record three independent index strokes. At each stop, compare the occupied pocket center with the marked pick point in a calibrated overhead image. Require ±1 mm for at least 0.20 s, no loss from any pocket, and no jam across the sequence. Verify one second released on the last pocket. The reference package is STM32H753IIK6, which CUAV specifies for the Pixhawk V6X v2. ST lists a 10 × 10 mm UFBGA body and its package-family shipping note specifies 24 mm tape and 16 mm pocket pitch. The chosen simulation pocket geometry represents that reference. Procurement availability of an individual production reel remains unchecked. [CUAV controller specification](https://doc.cuav.net/en/controller/pixhawk-v6x/) · [ST part and package](https://www.st.com/en/microcontrollers-microprocessors/stm32h753ii.html) · [ST tape and reel reference](https://www.st.com/resource/en/technical_note/tn1204-tape-and-reel-shipping-media-for-stm32-microcontrollers-in-bga-packages-stmicroelectronics.pdf)

## Run on Lambda

From this task folder, activate the environment and read the instruction and control-step limit.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
```

### Scripted demo

```bash
inspect-robots run --policy yam_scripted -P hold_s=3 --embodiment yam_arena \
  --scorer task_score -E task=arena_task.py -E headless=true -E object_jitter=0 \
  -E control_hz=20 -E cam_width=640 -E cam_height=352 --seed 0 \
  --instruction "$instruction" --max-steps "$steps" --no-prompt --store-frames --log-dir logs/demo
```

### Dream-1

```bash
inspect-robots run --policy dream --embodiment yam_arena --scorer task_score \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 --instruction "$instruction" \
  --max-steps "$steps" --no-prompt --store-frames --log-dir logs/dream
```

Read the final `task_score` under `metrics` in the saved evaluation. Run `python task.py --ik` for the synthetic scoring and reach checks.

## Remaining hardware validation

Validate the compliant peel and live sprocket engagement with the documented 24 mm carrier. Record the reel label and check the actual pocket clearance before comparing physical forces.

The scripted Lambda trial scored **1.00** in `adhoc_f82ea296.json`.

Dream-1 scored **0.0** in the saved evaluation `adhoc_a99e33a0.json`.

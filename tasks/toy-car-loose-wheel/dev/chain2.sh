#!/bin/bash
# toy-car-loose-wheel: the rest of the final runs, reordered (host slower than planned). Waits for demo-s3 first.
cd ~/benchmark-tasks/tasks/toy-car-loose-wheel || exit 1
Y=~/yam-eval; N=toy-car-loose-wheel; M=$Y/$N-media; PY=/data/IsaacLab-Arena/.venv/bin/python; S=1800
while [ ! -e $Y/$N-demo-s3/finished.txt ]; do sleep 10; done
bash dev/run.sh v-fixedguess-s0 yam_scripted 0 $S DEMO_VARIANT=fixedguess
bash dev/run.sh v-allbasket-s0 yam_scripted 0 $S DEMO_VARIANT=allbasket
bash dev/run.sh v-nothing-s0 yam_scripted 0 $S DEMO_VARIANT=nothing
bash dev/run.sh v-onearm-s6 yam_scripted 6 $S DEMO_VARIANT=onearm
bash dev/run.sh demo-s2 yam_scripted 2 $S
if curl -sf -m 10 http://127.0.0.1:8211/health; then
  echo "$(date -u +%H:%M:%S) bridge ok, waiting for ~/dream.lock" > $M/dream-state.txt
  FRAMES=1 flock ~/dream.lock bash dev/run.sh dream-s0 dream 0 $S
  (cd $M && $PY build_media.py $Y/$N-dream-s0 model_run.mp4) && rm -rf $Y/$N-dream-s0/frames
  echo "$(date -u +%H:%M:%S) dream done" >> $M/dream-state.txt
else
  echo "$(date -u +%H:%M:%S) no Dream bridge answering" > $M/dream-state.txt
fi
bash dev/run.sh v-fixedguess-s3 yam_scripted 3 $S DEMO_VARIANT=fixedguess
bash dev/run.sh v-fixedguess-s2 yam_scripted 2 $S DEMO_VARIANT=fixedguess
bash dev/run.sh v-allbox-s0 yam_scripted 0 $S DEMO_VARIANT=allbox
echo CHAIN DONE > $M/chain-done.txt

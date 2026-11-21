#!/bin/bash
# toy-car-loose-wheel: the final runs in one chain (one simulation at a time). Run from anywhere on the GPU machine.
cd ~/benchmark-tasks/tasks/toy-car-loose-wheel || exit 1
Y=~/yam-eval; N=toy-car-loose-wheel; M=$Y/$N-media; PY=/data/IsaacLab-Arena/.venv/bin/python; S=1800
mkdir -p $M && cp ~/yam-eval/cuffs-media/build_media.py $M/
FRAMES=1 bash dev/run.sh demo-s0 yam_scripted 0 $S
(cd $M && $PY build_media.py $Y/$N-demo-s0 demo.mp4 setup.png) && rm -rf $Y/$N-demo-s0/frames
bash dev/run.sh demo-s3 yam_scripted 3 $S
bash dev/run.sh demo-s2 yam_scripted 2 $S
bash dev/run.sh v-allbasket-s0 yam_scripted 0 $S DEMO_VARIANT=allbasket
if curl -sf -m 10 http://127.0.0.1:8211/health; then
  echo "$(date -u +%H:%M:%S) bridge ok, waiting for ~/dream.lock" > $M/dream-state.txt
  FRAMES=1 flock ~/dream.lock bash dev/run.sh dream-s0 dream 0 $S
  (cd $M && $PY build_media.py $Y/$N-dream-s0 model_run.mp4) && rm -rf $Y/$N-dream-s0/frames
  echo "$(date -u +%H:%M:%S) dream done" >> $M/dream-state.txt
else
  echo "$(date -u +%H:%M:%S) no Dream bridge answering" > $M/dream-state.txt
fi
for s in 0 2 3; do bash dev/run.sh v-fixedguess-s$s yam_scripted $s $S DEMO_VARIANT=fixedguess; done
bash dev/run.sh v-allbox-s0 yam_scripted 0 $S DEMO_VARIANT=allbox
bash dev/run.sh v-nothing-s0 yam_scripted 0 $S DEMO_VARIANT=nothing
bash dev/run.sh v-onearm-s6 yam_scripted 6 $S DEMO_VARIANT=onearm
echo CHAIN DONE > $M/chain-done.txt

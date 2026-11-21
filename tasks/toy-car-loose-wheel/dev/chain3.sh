#!/bin/bash
# toy-car-loose-wheel: every final run on one code version (scorer needs the pulled-out wheel; STAFF / REPAIR sign).
# Before each simulation it waits until neither priority revision session runs a simulation, or 20 GB of GPU memory is
# free. One simulation at a time. Runs not recorded use small cameras (Dream-1 and the video run keep 640 x 360).
cd ~/benchmark-tasks/tasks/toy-car-loose-wheel || exit 1
Y=~/yam-eval; N=toy-car-loose-wheel; M=$Y/$N-media; PY=/data/IsaacLab-Arena/.venv/bin/python; S=1800
mkdir -p $M
busy() {  # 0 when a priority session has an inspect-robots run in its process tree
  for s in spirit-burner-cap-first-rev spray-nozzle-off-rev; do
    for pp in $(tmux list-panes -s -t "$s" -F '#{pane_pid}' 2>/dev/null); do
      ps -eo pid=,ppid=,args= | awk -v root="$pp" '{pid[NR]=$1; par[NR]=$2; a[NR]=$0}
        END{keep[root]=1; ch=1; while(ch){ch=0; for(i=1;i<=NR;i++) if(keep[par[i]] && !keep[pid[i]]){keep[pid[i]]=1; ch=1}}
            for(i=1;i<=NR;i++) if(keep[pid[i]] && a[i] ~ /inspect-robots run/) f=1; exit !f}' && return 0
    done
  done
  return 1
}
gate() {
  while busy && [ "$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)" -lt 20000 ]; do
    echo "$(date +%H:%M) waiting: a priority revision is simulating and under 20 GB free" > $M/gate.txt; sleep 120
  done
  echo "$(date +%H:%M) go $1" > $M/gate.txt
}
run() { gate "$1"; bash dev/run.sh "$@"; }
FRAMES=1 run r2-demo-s0 yam_scripted 0 $S
(cd $M && $PY build_media.py $Y/$N-r2-demo-s0 demo.mp4 setup.png) && rm -rf $Y/$N-r2-demo-s0/frames && echo "$(date +%H:%M) demo media built" >> $M/r2-media.txt
CAM_W=160 CAM_H=90 run r2-fixedguess-s3 yam_scripted 3 $S DEMO_VARIANT=fixedguess
CAM_W=160 CAM_H=90 run r2-demo-s3 yam_scripted 3 $S
CAM_W=160 CAM_H=90 run r2-demo-s2 yam_scripted 2 $S
CAM_W=160 CAM_H=90 run r2-nothing-s0 yam_scripted 0 $S DEMO_VARIANT=nothing
CAM_W=160 CAM_H=90 run r2-allbasket-s0 yam_scripted 0 $S DEMO_VARIANT=allbasket
if curl -sf -m 10 http://127.0.0.1:8211/health; then
  echo "$(date +%H:%M) bridge ok, waiting for ~/dream.lock" > $M/dream-state.txt
  gate dream
  bash ~/dream-retry.sh ~/benchmark-tasks/tasks/$N $N $S
  D=$(cat $Y/$N-dream-ok.txt 2>/dev/null)
  [ -n "$D" ] && (cd $M && $PY build_media.py $D model_run.mp4) && rm -rf $D/frames
  echo "$(date +%H:%M) dream done" >> $M/dream-state.txt
else
  echo "$(date +%H:%M) no Dream bridge answering" > $M/dream-state.txt
fi
CAM_W=160 CAM_H=90 run r2-allbox-s0 yam_scripted 0 $S DEMO_VARIANT=allbox
CAM_W=160 CAM_H=90 run r2-fixedguess-s0 yam_scripted 0 $S DEMO_VARIANT=fixedguess
CAM_W=160 CAM_H=90 run r2-fixedguess-s2 yam_scripted 2 $S DEMO_VARIANT=fixedguess
CAM_W=160 CAM_H=90 run r2-onearm-s6 yam_scripted 6 $S DEMO_VARIANT=onearm
echo "CHAIN DONE $(date +%H:%M)" > $M/r2-chain-done.txt

"""Run a complete fresh-reset shirt trial using the published configuration."""
import argparse,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--policy',choices=('demo','dream'),required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
c=[sys.executable,str(HERE/'run.py'),'--regular-refined','--raised-support-diagnostic','--surface-contact-diagnostic','--center-peg-diagnostic','--binary-cloth-trace','--solver-iterations','128','--physics-hz','240','--functional-pegs','--external-forces-once-diagnostic','--judging-views','--out',str(a.out)]
if a.policy=='demo':c+=['--adaptive-clip','--mode','motion-probe','--plan',str(HERE/'demo-plan.json'),'--state-assisted-grasp','--continuous-pad-diagnostic','--lift-turn-diagnostic','--state-assisted-clip','--joint-feedback-from','380','--edge-patch-diagnostic','--wide-insertion-diagnostic']
else:c+=['--mode','dream','--steps','2400','--dream-rows-per-request','2']
subprocess.run(c,check=True)

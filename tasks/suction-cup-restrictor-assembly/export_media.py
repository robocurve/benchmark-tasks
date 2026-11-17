"""Export validated original camera streams, overview above wrist views."""
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('run',type=Path);p.add_argument('output',type=Path)
p.add_argument('--key-frame',type=int,default=400)
a=p.parse_args();r=a.run.resolve()
v=json.loads((r/'validation.json').read_text())
assert v.get('run_valid',v['fullscore_demo_valid']), 'Only independently validated complete runs can be exported'
reports=[f for f in r.glob('adhoc*.json') if '.live.' not in f.name]
assert len(reports)==1
report=json.loads(reports[0].read_text())
frames=Path(report['stats']['frames_dir']);steps=v['steps']
files={c:sorted(frames.glob('*_'+c+'_*.npy')) for c in ('top_cam','left_cam','right_cam')}
model=v.get('model_result',False)
plan=None if model else json.loads((r/'action-plan.json').read_text())['steps']
scores=json.loads((r/'score-timeline.json').read_text())['scores']
assert len(scores)==steps+1
a.output.mkdir(parents=True,exist_ok=False)
video=a.output/('dream-demo.mp4' if model else 'fullscore-demo.mp4')
proc=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-f','rawvideo',
    '-pix_fmt','rgb24','-s','640x576','-r','20','-i','-','-an','-c:v','libx264',
    '-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],stdin=subprocess.PIPE)
for i in range(steps+1):
    im={c:np.load(f[i]) for c,f in files.items()}
    canvas=np.full((576,640,3),245,np.uint8)
    canvas[:352]=cv2.resize(im['top_cam'],(640,352),interpolation=cv2.INTER_NEAREST)
    canvas[352:528,:320]=im['left_cam'];canvas[352:528,320:]=im['right_cam']
    cv2.putText(canvas,f'{"DREAM-1" if model else "SCRIPTED FEASIBILITY"} | SCORE {scores[i]:g} | CUSTOM PARTS',
        (8,547),cv2.FONT_HERSHEY_SIMPLEX,.45,(35,35,35),1,cv2.LINE_AA)
    label='model evaluation' if model else plan[max(0,min(i-1,len(plan)-1))]['label']
    cv2.putText(canvas,f'{i/20:.2f}s | {label}',(8,568),cv2.FONT_HERSHEY_SIMPLEX,.43,(35,35,35),1,cv2.LINE_AA)
    proc.stdin.write(canvas.tobytes())
    if i in (0,a.key_frame,steps):
        name='setup' if i==0 else 'final' if i==steps else 'assembly'
        cv2.imwrite(str(a.output/(name+'-three-views.png')),cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR))
        for c in im:cv2.imwrite(str(a.output/(name+'-'+c+'.png')),cv2.cvtColor(im[c],cv2.COLOR_RGB2BGR))
proc.stdin.close();assert proc.wait()==0
for f in ('validation.json','score-replay.json','score-timeline.json','prototype-outcome.json','mechanism-contract.json','source_manifest.json'):
    shutil.copy2(r/f,a.output/f)
manifest=dict(source_run=str(r),frames_per_camera=steps+1,duration_s=(steps+1)/20,
    layout='640x576: overview above left/right wrists',processing='layout and labels only; no frame removal, camera crop or speed change',
    known_layout=not model,dream_result=model,fullscore_validated=v['fullscore_demo_valid'],
    sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in a.output.iterdir()})
(a.output/'media-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(a.output.resolve())

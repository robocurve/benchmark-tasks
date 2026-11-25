"""Launch the scene-only still capture on an explicitly selected GPU."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import uuid

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--gpu',type=int,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
source=Path(__file__).resolve().parent
out=a.output.expanduser().resolve()
if out.exists() and any(out.iterdir()):
    p.error('Choose a new empty output directory')
out.mkdir(parents=True,exist_ok=True)
cache=out.parent/'.cache'
(cache/'tmp').mkdir(parents=True,exist_ok=True)
name=f'rc-{os.getuid()}-catheter-scene-{uuid.uuid4().hex[:6]}'
command=['docker','run','-d','--name',name,'--gpus',f'device={a.gpu}',
         '--network','none','--shm-size','8g','--user',f'{os.getuid()}:{os.getgid()}',
         '--workdir','/outputs','--label',f'robocurve.uid={os.getuid()}',
         '--label','robocurve.task=balloon-catheter-removal',
         '-v','/etc/passwd:/etc/passwd:ro','-v','/etc/group:/etc/group:ro',
         '-v',f'{source}:/task:ro','-v',f'{out}:/outputs:rw',
         '-v',f'{cache}:/tmp/task-home:rw','-e','HOME=/tmp/task-home',
         '-e','TMPDIR=/tmp/task-home/tmp','-e','NVIDIA_DRIVER_CAPABILITIES=all',
         '-e','OMNI_KIT_ACCEPT_EULA=YES','-e','ACCEPT_EULA=Y','-e','OMP_NUM_THREADS=12',
         '-e',f'USER={os.environ.get("USER","robot")}']
for group in sorted(set(os.getgroups())|{1234}):
    command+=['--group-add',str(group)]
command+=['--entrypoint','/isaac-sim/python.sh','yam-inspect-isaac:6.1.0',
          '/task/capture_scene.py','--output-dir','/outputs','--device','cuda:0',
          '--livestream','0','--viz','none','--kit_args',
          '--portable-root /tmp/task-home/kit --/renderer/multiGpu/enabled=false '
          '--/app/file/ignoreUnsavedStage=1 --/plugins/carb.tasking.plugin/threadCount=12']
receipt={'container':name,'gpu':a.gpu,'command':command,'paid_inference':False}
(out/'launch.json').write_text(json.dumps(receipt,indent=2)+'\n')
subprocess.run(command,check=True)
print(json.dumps({'container':name,'output':str(out)},indent=2))

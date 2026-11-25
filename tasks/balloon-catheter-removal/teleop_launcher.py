"""Own two-container CloudXR/YAM session; never stops unrelated resources."""
import argparse
import datetime
import json
import os
import pwd
from pathlib import Path
import shutil
import shlex
import socket
import subprocess
import sys
import time
import urllib.request
import ssl
import uuid

HERE=Path(__file__).resolve().parent
WORKSPACE=HERE.parents[1]
UID=os.getuid()
USER_NAME=pwd.getpwuid(UID).pw_name
USER_RUNS=WORKSPACE/'runs'/USER_NAME
_matching_runs=sorted(USER_RUNS.glob('[0-9][0-9]_'+HERE.name.replace('-','_')))
if len(_matching_runs)>1:
    raise RuntimeError('Multiple indexed result folders for '+HERE.name)
RUNS=_matching_runs[0] if _matching_runs else USER_RUNS/('05_'+HERE.name.replace('-','_'))
USER_CACHE=WORKSPACE/'cache'/USER_NAME/HERE.name
LAST=RUNS/f'quest-last-{UID}.json'
SIM_IMAGE='yam-inspect-isaac:6.1.0'
CXR_IMAGE='isim6-cloudxr:latest'

def call(cmd,**kwargs):
    return subprocess.run(cmd,check=True,text=True,**kwargs)

def state(name):
    r=subprocess.run(['docker','inspect','--format','{{json .State}}',name],capture_output=True,text=True)
    return json.loads(r.stdout) if r.returncode==0 else None

def private_write(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
    with os.fdopen(fd,'w') as f:json.dump(data,f,indent=2)

def select_gpu(explicit):
    rows=call(['nvidia-smi','--query-gpu=index,memory.free,utilization.gpu','--format=csv,noheader,nounits'],capture_output=True).stdout.splitlines()
    available=[tuple(map(lambda s:int(s.strip()),r.split(','))) for r in rows]
    if explicit is not None:
        found=[v for v in available if v[0]==explicit and v[1]>=16000]
        if not found:raise RuntimeError('Selected GPU needs at least 16 GB currently free')
        return found[0][0]
    available=[v for v in available if v[1]>=16000]
    if not available:raise RuntimeError('No GPU has 16 GB free')
    return min(available,key=lambda v:(v[2],-v[1]))[0]

def base_container(name,gpu):
    cmd=['docker','run','-d','--name',name,'--gpus',f'device={gpu}',
         '--init','-e','TINI_KILL_PROCESS_GROUP=1',
         '--network','host','--ipc','host','--shm-size','8g',
         '--user',f'{UID}:{os.getgid()}',
         '--label',f'robocurve.uid={UID}','--label',f'robocurve.task={HERE.name}',
         '--label','robocurve.mode=quest',
         '-v','/etc/passwd:/etc/passwd:ro','-v','/etc/group:/etc/group:ro',
         '-e','NVIDIA_DRIVER_CAPABILITIES=all','-e','OMNI_KIT_ACCEPT_EULA=YES','-e','ACCEPT_EULA=Y']
    for group in sorted(set(os.getgroups())|{1234}):cmd+=['--group-add',str(group)]
    return cmd

def wait_cloudxr(name,seconds=150):
    deadline=time.monotonic()+seconds
    ctx=ssl._create_unverified_context()  # Local CloudXR intentionally uses its generated certificate.
    while time.monotonic()<deadline:
        s=state(name)
        if not s or not s['Running']:raise RuntimeError('CloudXR exited; inspect its saved container logs')
        try:
            with urllib.request.urlopen('https://127.0.0.1:48322/',context=ctx,timeout=2) as r:
                if r.status==200:return
        except (OSError,ValueError):pass
        time.sleep(.5)
    raise RuntimeError('CloudXR did not become reachable on HTTPS 48322')

def own_stop(receipt):
    for name in reversed(receipt.get('containers',[])):
        labels=subprocess.run(['docker','inspect','--format','{{json .Config.Labels}}',name],text=True,capture_output=True)
        if labels.returncode:continue
        labels=json.loads(labels.stdout)
        if labels.get('robocurve.uid')!=str(UID) or labels.get('robocurve.task')!=HERE.name or labels.get('robocurve.mode')!='quest':
            raise RuntimeError('Refusing to stop a container with different ownership labels')
        # SIGINT asks the recorder to finalize MP4/trajectory before Docker waits.
        if state(name)['Running']:call(['docker','stop','--signal','SIGINT','--timeout','20',name],stdout=subprocess.DEVNULL)
        out=Path(receipt['output'])
        with (out/(name+'.log')).open('w') as f:
            subprocess.run(['docker','logs',name],stdout=f,stderr=subprocess.STDOUT)
        call(['docker','rm',name],stdout=subprocess.DEVNULL)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['quest','status','stop','record','detach'])
    p.add_argument('--gpu',type=int)
    p.add_argument('--host',help='Address reachable from Quest; autodetected if omitted')
    p.add_argument('--output',type=Path)
    p.add_argument('--check',action='store_true',help='Read-only preflight')
    p.add_argument('--dry-run',action='store_true')
    p.add_argument('--accept-cloudxr-eula',action='store_true')
    p.add_argument('--no-adb',action='store_true')
    p.add_argument('--smoke-seconds',type=float,default=0)
    p.add_argument('--no-xr',action='store_true',help='Only with --smoke-seconds: offline recorder test')
    p.add_argument('--probe-controller',action='store_true',help='Only with --smoke-seconds: synthetic input plumbing test')
    p.add_argument('--startup-timeout',type=float,default=600)
    a=p.parse_args()
    if a.no_xr and a.smoke_seconds<=0:p.error('--no-xr is restricted to an explicit smoke test')
    if a.probe_controller and a.smoke_seconds<=0:p.error('--probe-controller requires --smoke-seconds')
    previous=json.loads(LAST.read_text()) if LAST.exists() else None
    if a.action in ('stop','status','record','detach'):
        if not previous:print('No Quest session for this user/task');return
        if a.action=='stop':own_stop(previous);print('Stopped this task’s Quest session; recordings finalized');return
        if a.action in ('record','detach'):
            if not (state(previous['containers'][-1]) or {}).get('Running'):
                raise RuntimeError('The simulator is not running')
            target=Path(previous['output'])/'command.json'
            tmp=target.with_suffix('.tmp')
            private_write(tmp,{'id':uuid.uuid4().hex,'action':a.action})
            tmp.replace(target)
            print('Requested '+a.action+'; teleop status reports the acknowledgement');return
        status_file=Path(previous['output'])/'teleop-status.json'
        status_data=json.loads(status_file.read_text()) if status_file.exists() else {'ready':False}
        print(json.dumps({**previous,'states':{n:state(n) for n in previous['containers']},
                          'teleop_status':status_data},indent=2));return
    if previous and (state(previous['containers'][-1]) or {}).get('Running'):
        print('Quest session already running on GPU '+str(previous['gpu']))
        print('Quest browser: '+previous['quest_url'])
        print('Recordings: '+previous['output'])
        return
    if previous and not (a.check or a.dry_run) and any((state(n) or {}).get('Running') for n in previous['containers']):
        own_stop(previous)
    gpu=select_gpu(a.gpu)
    for image in (SIM_IMAGE,CXR_IMAGE):call(['docker','image','inspect','--format','{{.Id}}',image],stdout=subprocess.DEVNULL)
    ports={}
    for port in (48322,49100):
        with socket.socket() as s:ports[str(port)]=s.connect_ex(('127.0.0.1',port))==0
    candidates=[USER_CACHE/f'quest-runtime-{UID}'/'cloudxr/run/eula_accepted',
                Path.home()/'.cloudxr/run/eula_accepted',
                Path.home()/'docker/isaac-sim-2/cloudxr/run/eula_accepted']
    accepted=next((path for path in candidates if path.is_file()),candidates[0])
    if any(ports.values()):raise RuntimeError('CloudXR ports 48322/49100 are occupied; existing services were left alone')
    if not accepted.is_file() and not a.accept_cloudxr_eula:
        raise RuntimeError('CloudXR EULA acceptance absent. Review https://github.com/NVIDIA/IsaacCapture/blob/main/deps/cloudxr/CLOUDXR_LICENSE then pass --accept-cloudxr-eula if accepted.')
    if a.check:
        print(json.dumps({'preflight_passed':True,'gpu':gpu,'images':[SIM_IMAGE,CXR_IMAGE],
                          'cloudxr_ports_free':True,'existing_eula_acceptance':accepted.is_file(),
                          'headset_connection':'Requires Quest browser WebXR session; not established by preflight'},indent=2));return
    host=a.host
    if not host:
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            s.connect(('8.8.8.8',80));host=s.getsockname()[0]
    token=datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:4]
    out=(a.output or RUNS/('quest-'+token)).resolve()
    if out.exists() and any(out.iterdir()):raise RuntimeError('Choose a new empty output directory')
    private_runtime=USER_CACHE/f'quest-runtime-{UID}'
    cloud=private_runtime/'cloudxr';cache=USER_CACHE/'render'
    source=out/'source'
    names=[f'rc-{UID}-quest-cxr-{token}',f'rc-{UID}-quest-yam-{token}']
    url=f'https://{host}:48322/client/?serverIP={host}&port=48322&immersiveMode=vr&xrOffsetX=0&xrOffsetY=0&xrOffsetZ=0#/sim'
    cxr=base_container(names[0],gpu)+['-v',f'{cloud}:/isaac-sim/.cloudxr:rw','-v',f'{source}:/task:ro',
        '-e','HOME=/isaac-sim/.cloudxr','--entrypoint','bash',CXR_IMAGE,'-c',
        'source /isaac-sim/setup_python_env.sh; export PYTHONPATH=/opt/robotescape-cloudxr-python:$PYTHONPATH; exec /isaac-sim/kit/python/bin/python3 /task/cloudxr_boot.py --cloudxr-install-dir /isaac-sim/.cloudxr --host-client'+(' --accept-eula' if a.accept_cloudxr_eula else '')]
    sim=base_container(names[1],gpu)+['-v',f'{source}:/task:ro','-v',f'{out}:/outputs:rw',
        '-v',f'{cache}:/tmp/task-home:rw','-v',f'{cloud}:/isaac-sim/.cloudxr:rw',
        '-e','HOME=/tmp/task-home','-e','TMPDIR=/tmp/task-home/tmp',
        '-e','XR_RUNTIME_JSON=/isaac-sim/.cloudxr/openxr_cloudxr.json',
        '-e','NV_CXR_RUNTIME_DIR=/isaac-sim/.cloudxr/run','-e','NV_CXR_OUTPUT_DIR=/isaac-sim/.cloudxr/logs',
        '-e','XRT_NO_STDIN=true','-e','OMP_NUM_THREADS=12',
        '-e','NV_CXR_ENABLE_PUSH_DEVICES=true','-e','NV_CXR_ENABLE_TENSOR_DATA=true','-e','NV_DEVICE_PROFILE=auto-webrtc',
        '--entrypoint','/isaac-sim/python.sh',SIM_IMAGE,'/task/teleop.py',
        '--output-dir','/outputs','--device','cuda:0','--livestream','0','--viz','none',
        '--experience','/opt/IsaacLab/apps/isaaclab.python.xr.openxr.headless.kit',
        '--kit_args','--portable-root /tmp/task-home/kit --/renderer/multiGpu/enabled=false --/app/window/enabled=false --/app/file/ignoreUnsavedStage=1 --/plugins/carb.tasking.plugin/threadCount=12']
    if a.smoke_seconds:sim+=['--smoke-seconds',str(a.smoke_seconds)]
    if a.no_xr:sim+=['--no-xr']
    if a.probe_controller:sim+=['--probe-controller']
    receipt={'task':HERE.name,'uid':UID,'gpu':gpu,'containers':names,'output':str(out),'quest_url':url,
             'reference_branch':'SNU-PI/robotescape feature/quest-scene-selection','reference_sha':'e08e487f2d1c738800489b0b4eaa1734507967bc',
             'commands':[cxr,sim],'paid_inference':False,'private_cloudxr_runtime':str(cloud)}
    if a.dry_run:print(json.dumps(receipt,indent=2));return
    out.mkdir(parents=True);private_runtime.mkdir(mode=0o700,exist_ok=True)
    os.chmod(private_runtime,0o700)
    cloud.mkdir(mode=0o700,exist_ok=True);(cloud/'run').mkdir(mode=0o700,exist_ok=True)
    if accepted.is_file() and not (cloud/'run/eula_accepted').exists():shutil.copyfile(accepted,cloud/'run/eula_accepted')
    (cache/'tmp').mkdir(parents=True,exist_ok=True)
    shutil.copytree(HERE,source,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.pytest_cache'))
    private_write(out/'launch.json',receipt);private_write(LAST,receipt)
    try:
        call(cxr,stdout=subprocess.DEVNULL)
        print('Starting CloudXR; output: '+str(out),flush=True)
        wait_cloudxr(names[0])
        call(sim,stdout=subprocess.DEVNULL)
        print('Quest browser: '+url,flush=True)
        print('Waiting for Isaac XR scene initialization...',flush=True)
        deadline=time.monotonic()+a.startup_timeout
        while time.monotonic()<deadline:
            status=out/'teleop-status.json'
            if status.exists():
                result=json.loads(status.read_text())
                if result.get('ready'):break
                if result.get('error'):raise RuntimeError(result['error'])
            if not (state(names[1]) or {}).get('Running'):raise RuntimeError('Isaac XR exited; see container logs')
            time.sleep(.5)
        else:raise RuntimeError('Isaac XR initialization timed out; inspect container logs')
        print(('Recorder smoke runtime ready.' if a.no_xr else 'Isaac XR ready.')+' Squeeze: clutch arm; trigger: gripper. Right stick click: record. Left stick click: detach drained receiver.',flush=True)
        if not a.no_adb and shutil.which('adb'):
            devices=call(['adb','devices'],capture_output=True).stdout.splitlines()
            authorized=[r.split()[0] for r in devices if r.endswith('\tdevice')]
            if len(authorized)==1:
                remote_command=shlex.join(['am','start','-a','android.intent.action.VIEW','-d',url])
                subprocess.run(['adb','-s',authorized[0],'shell',remote_command],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                print('Opened client on the authorized USB headset; enter VR in its browser.')
        print('Recordings: '+str(out));print('Stop: ./run_robocurve teleop stop')
    except BaseException:
        own_stop(receipt)
        raise

if __name__=='__main__':
    try:main()
    except (RuntimeError,subprocess.CalledProcessError) as e:
        print('ERROR: '+str(e),file=sys.stderr);sys.exit(1)

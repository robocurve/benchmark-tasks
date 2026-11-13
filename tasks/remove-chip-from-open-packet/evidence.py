"""Record source identity and export a completed Inspect Robots trial."""
import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


ROOT=Path(__file__).resolve().parent


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def sources():
    return {
        p.relative_to(ROOT).as_posix():digest(p)
        for parent in (ROOT,ROOT/'runtime',ROOT/'policy')
        for p in sorted(parent.iterdir())
        if p.is_file() and p.suffix in {'.py','.toml','.sh'}
    }


def write(path,payload):
    path.write_text(json.dumps(payload,indent=2,sort_keys=True)+'\n',encoding='utf8')


def require(condition,message):
    if not condition:
        raise ValueError(message)


def export(log):
    log=log.resolve()
    run=log.parent
    mode=run.parent.name
    source=run/'sources.json'
    require(json.loads(source.read_text())==sources(),'Current source differs from run source')
    trial=json.loads(log.read_text())
    require(mode in {'demo','dream'},'Export from runs/demo or runs/dream')
    require(trial['status']=='success' and not trial['error'] and
            trial['results']['errored_trials']==0 and trial['results']['total_trials']==1,
            'Trial did not complete cleanly')
    spec=trial['eval']
    require(spec['max_steps']==2400 and spec['embodiment_info']['control_hz']==20,
            'Trial limit or frequency mismatch')
    require(spec['policy']==('yam_scripted' if mode=='demo' else 'dream_current'),'Wrong policy')
    steps=trial['stats']['total_steps']
    score=trial['results']['metrics']['task_score']
    require((mode=='demo' and score==1 and 0<steps<=2400) or
            (mode=='dream' and 0<steps<=2400 and 0<=score<=1 and (steps==2400 or score==1)),
            'Unexpected score or length')
    trace=run/'trajectory.jsonl'
    with trace.open() as stream:
        rows=[json.loads(line) for line in stream]
    require(len(rows)==steps and abs(rows[-1]['task_diagnostics']['score']-score)<1e-6,
            'Native trace does not agree with trial')
    require(all(abs(row['t']-(i+1)/20)<1e-4 for i,row in enumerate(rows)),
            'Non-contiguous 20 Hz telemetry')
    out=run/'evidence'
    out.mkdir(exist_ok=True)
    rendered=out/'native_render'
    subprocess.run(['inspect-robots','video',str(log),'--out',str(rendered)],check=True)
    assets={}
    for camera in ('top_cam','left_cam','right_cam'):
        videos=list(rendered.rglob('*'+camera+'.mp4'))
        require(len(videos)==1,'Expected one rendered stream for '+camera)
        target=out/(camera+'.mp4')
        subprocess.run(['ffmpeg','-v','error','-y','-i',str(videos[0]),'-c:v','libx264',
                        '-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(target)],check=True)
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
            '-show_entries','stream=codec_name,width,height,r_frame_rate,nb_frames','-of','json',
            str(target)]))['streams'][0]
        require((probe['width'],probe['height'],probe['codec_name'])==(1280,720,'h264') and
                Fraction(probe['r_frame_rate'])==20 and int(probe['nb_frames'])==steps+1 and
                target.stat().st_size<=50_000_000,'Unexpected camera recording: '+camera)
        assets[target.name]=digest(target)
    subprocess.run(['ffmpeg','-v','error','-y','-i',str(out/'top_cam.mp4'),'-frames:v','1',
                    str(out/'setup.png')],check=True)
    require((out/'setup.png').stat().st_size<=10_000_000,'Setup image exceeds upload limit')
    assets['setup.png']=digest(out/'setup.png')
    for item in [log,trace,source]+([run/'dream_episode.json',run/'dream_session.json'] if mode=='dream' else []):
        dest=out/('trial.json' if item==log else item.name)
        shutil.copy2(item,dest)
        assets[dest.name]=digest(dest)
    write(out/'manifest.json',dict(mode=mode,score=score,steps=steps,files=assets))
    print(out/'manifest.json')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['snapshot','export'])
    parser.add_argument('path',type=Path)
    args=parser.parse_args()
    if args.action=='snapshot':
        write(args.path,sources())
    else:
        export(args.path)

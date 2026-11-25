"""Export an uncut five-view demo; CPU encoding only, never simulation or upload."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import imageio_ffmpeg

TASK=Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def probe(path):
    reader=imageio_ffmpeg.read_frames(str(path))
    metadata=next(reader);reader.close()
    frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(path))
    return {'codec_name':metadata['codec'],'pix_fmt':metadata['pix_fmt'].split('(')[0],
            'width':metadata['size'][0],'height':metadata['size'][1],
            'fps':metadata['fps'],'nb_frames':frames,'duration':seconds}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--crf',type=int,default=20)
    args=parser.parse_args();run=args.run.resolve();out=args.out.resolve()
    result=json.loads((run/'result.json').read_text())
    seed_parser=argparse.ArgumentParser(add_help=False)
    seed_parser.add_argument('--seed',type=int,default=0)
    recorded_seed=result.get('seed',seed_parser.parse_known_args(result.get('argv',[]))[0].seed)
    if not (result['mode']=='full' and result['completed'] and result['full_task_demonstration']
            and result['scoring']['score']==1. and result['diagnostic_preinstalled_pads']==0
            and result['device']['verified'] and result['steps']<=6000):
        raise ValueError('A genuine completed full-score bare-start run is required')
    audit=json.loads(subprocess.check_output(
        [sys.executable,'-B',str(TASK/'tools/rescore_revision.py'),str(run)],text=True))
    if not (audit['rescored_score']==1. and audit['same_final_score'] and audit['bare_start']):
        raise ValueError('Independent rescoring failed')
    names=('top_cam','left_cam','right_cam','observer_cam','observer_opposite_cam')
    sources=[run/(n+'.mp4') for n in names]
    info=[probe(p) for p in sources]
    for v in info:
        if (v['nb_frames'],v['fps'],v['width'],v['height'])!=(result['steps']+1,20.,640,352):
            raise ValueError('All five streams must contain the complete synchronized trial at 640x352/20 Hz')
    out.mkdir(parents=True,exist_ok=False)
    labels=('Overview','Left wrist','Right wrist','Evidence A - recording only','Evidence B - recording only')
    subtitles='''[Script Info]
ScriptType: v4.00+
PlayResX: 1280
PlayResY: 704
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Label,DejaVu Sans,18,&H00FFFFFF,&H00FFFFFF,&H80000000,&H80000000,0,0,0,0,100,100,0,0,3,3,0,7,8,8,8,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
'''
    for label,(x,y) in zip(labels,((8,8),(434,8),(860,8),(8,360),(648,360))):
        subtitles+=f'Dialogue: 0,0:00:00.00,0:59:59.99,Label,,0,0,0,,{{\\an7\\pos({x},{y})}}{label}\n'
    for i,label in enumerate((f"Scripted demonstration | Final score: 1.0 | {result['steps']} / 6000 actions | Complete trial - normal speed",
                               'Evidence views below are recording only and retain their full 640 x 352 pixels.')):
        subtitles+=f'Dialogue: 0,0:00:00.00,0:59:59.99,Label,,0,0,0,,{{\\an7\\pos(12,{266+34*i})}}{label}\n'
    (out/'labels.ass').write_text(subtitles)
    command=[imageio_ffmpeg.get_ffmpeg_exe(),'-hide_banner','-loglevel','error','-nostdin',
             '-filter_complex_threads','2']
    for source in sources:command+=['-threads','1','-i',str(source)]
    graph=('[0:v]scale=426:234,setsar=1[n0];[1:v]scale=426:234,setsar=1[n1];'
           '[2:v]scale=426:234,setsar=1[n2];'
           '[n0][n1][n2][3:v][4:v]xstack=inputs=5:'
           'layout=0_0|426_0|852_0|0_352|640_352:fill=black[grid];'
           '[grid]ass=labels.ass:fontsdir=/usr/share/fonts/truetype/dejavu[video]')
    command+=['-filter_complex',graph,'-map','[video]','-an','-map_metadata','-1',
              '-c:v','libx264','-preset','medium','-crf',str(args.crf),'-threads','4',
              '-pix_fmt','yuv420p','-movflags','+faststart',str(out/'demo.mp4')]
    subprocess.run(command,check=True,cwd=out)
    final=probe(out/'demo.mp4')
    if (final['codec_name'],final['pix_fmt'],final['nb_frames'],final['fps'])!=('h264','yuv420p',result['steps']+1,20.):
        raise ValueError('Export lost frames or has the wrong encoding')
    if (out/'demo.mp4').stat().st_size>=50_000_000:
        raise ValueError('Video exceeds 50 MB; preserve it and encode into a new output with a higher --crf')
    manifest={'run':run.name,'score':1.,'actions':result['steps'],'seed':recorded_seed,
              'uncut':True,'speed':1.,'encoding':final,'crf':args.crf,
              'export_view_sizes':{'native_views':[426,234],'evidence_views':[640,352]},
              'source_recordings_sha256':{p.name:sha(p) for p in sources},
              'source_result_sha256':sha(run/'result.json'),
              'source_measurements_sha256':sha(run/'measurements.jsonl'),
              'source_bundle_sha256':result['bundle_sha256'],
              'evidence_cameras':result['evidence_cameras'],'independent_rescore':audit,
              'file':{'bytes':(out/'demo.mp4').stat().st_size,'sha256':sha(out/'demo.mp4')}}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'run':run.name,'encoding':final,'file':manifest['file']},indent=2))


if __name__=='__main__':main()

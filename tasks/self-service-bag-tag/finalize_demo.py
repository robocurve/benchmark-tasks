"""Verify the continuous rollout before labeling or encoding it as a success."""
import json,sys,subprocess,hashlib
from pathlib import Path
import numpy as np
import imageio_ffmpeg
from yam_arena.constants import READY
from yam_arena import ik,station
run=Path(sys.argv[1]).resolve();assert not (run/'STAGED_NOT_DEMO.json').exists(), 'Staged diagnostic cannot be a full demo'
data=json.loads((run/'result.json').read_text());trace=data['trace'];last=trace[-1];c=last['closure']
assert data.get('stage')=='continuous_full_demo_complete' and data.get('terminal_success'), 'No complete terminal result'
assert any(r['closure'].get('bond_press_n',0.)>=1. for r in trace), 'No physical pressure evidence'
errors={}
for side in ('left','right'):
 actual=ik.tcp_pose(np.array(last['robots'][side]['joints'])[:6],station.BASES[side])[0]
 target=ik.tcp_pose(READY,station.BASES[side])[0]
 errors[side]=float(np.linalg.norm(actual-target))
passed=(c['score']==1. and c['stable_s']>=2. and c['bonded'] and not c['broken'] and c['hands_clear'] and c['hands_open'] and max(errors.values())<.02)
review={'passed':passed,'closure':c,'ready_error_m':errors,'continuous_simulated_seconds':last['step']/20,'source_sha256':{f:hashlib.sha256((Path(__file__).parent/f).read_bytes()).hexdigest() for f in ('task.py','tag_closure.py','press_contacts.py','direct_thread_transport_probe.py','partkit.py')}}
launch=run/'source_at_launch.json'
if launch.exists():
 recorded=json.loads(launch.read_text())
 for name,digest in recorded.items():
  if name in review['source_sha256']: assert review['source_sha256'][name]==digest, 'Source changed since launch: '+name
loaded=run/'tag_closure.loaded.py'
if loaded.exists():
 review['source_sha256']['tag_closure.py']=hashlib.sha256(loaded.read_bytes()).hexdigest()
 review['scoring_note']='GPU run used saved tag_closure.loaded.py. Revised reward validated separately in revised-score-audit.json.'
 audit=run/'revised-score-audit.json'
 if audit.exists():
  review['revised_offline_score']=json.loads(audit.read_text())['final_score']
(run/'final-review.json').write_text(json.dumps(review,indent=2)+'\n')
assert passed, json.dumps(review)
frames=sorted((run/'frames').glob('top_*.png'));steps=[int(f.stem.split('_')[-1]) for f in frames]
assert all(b-a==4 for a,b in zip(steps,steps[1:])), 'Noncontinuous video frames'
manifest=run/'video-frames.txt';manifest.write_text(''.join("file '"+str(p)+"'\nduration 0.2\n" for p in frames))
video=run/'successful-demo.mp4'
subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error','-f','concat','-safe','0','-i',str(manifest),'-r','5','-c:v','libx264','-crf','24','-maxrate','2M','-bufsize','4M','-map_metadata','-1','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],check=True)
import imageio.v2 as imageio
reader=imageio_ffmpeg.read_frames(str(video),pix_fmt='rgb24');metadata=next(reader)
count=0;decoded=None
for pixels in reader:
    count+=1
    decoded=np.frombuffer(pixels,np.uint8).reshape(metadata['size'][1],metadata['size'][0],3)
assert count==len(frames), (count,len(frames))
error=float(np.mean(np.abs(decoded.astype(float)-imageio.imread(frames[-1]).astype(float))))
assert error<8., 'Encoded terminal frame differs from actual terminal simulation frame'
imageio.imwrite(run/'encoded-final-frame.png',decoded)
(run/'encoded-video-audit.json').write_text(json.dumps(dict(decoded_frames=count,
    expected_frames=len(frames),duration_seconds=count/metadata['fps'],
    terminal_frame_mean_pixel_error=error,sha256=hashlib.sha256(video.read_bytes()).hexdigest()),indent=2))
print(video)

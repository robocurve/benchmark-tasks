from pathlib import Path
import json,sys,subprocess
from PIL import Image,ImageDraw,ImageFont
src=Path(sys.argv[1]);dst=Path(sys.argv[2]);dst.mkdir(parents=True,exist_ok=True)
rows=json.loads((src/'trace.json').read_text());font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
is_dream=(src/'dream_transcript.json').exists()
video_name='model_run.mp4' if is_dream else 'demo_native.mp4'
command=['ffmpeg','-v','error','-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','960x200','-r','4','-i','-','-an','-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(dst/video_name)]
p=subprocess.Popen(command,stdin=subprocess.PIPE)
latched=[i for i,r in enumerate(rows) if r['attached']];chosen={0:'initial',len(rows)-1:'final'}
if latched:
 chosen[max(0,latched[0]-1)]='insertion';chosen[latched[0]]='latched';chosen[min(len(rows)-1,latched[0]+20)]='released'
for i,row in enumerate(rows):
 im=Image.new('RGB',(960,200),'#122132');d=ImageDraw.Draw(im)
 for j,(name,title) in enumerate([('top_cam','Top camera'),('left_cam','Left wrist camera — live'),('right_cam','Right wrist camera')]):
  d.text((j*320+8,3),title,font=font,fill='white');im.paste(Image.open(src/f'{name}_{row["step"]:04d}.png'),(j*320,24))
 p.stdin.write(im.tobytes())
 if i in chosen:
  im.save(dst/(chosen[i]+'.png'))
  Image.open(src/f'overview_cam_{row["step"]:04d}.png').save(dst/(chosen[i]+'_mechanism.png'))
p.stdin.close();assert p.wait()==0
subprocess.run(['ffmpeg','-v','error','-y','-framerate','4','-pattern_type','glob','-i',str(src/'overview_cam_*.png'),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(dst/'demo_mechanism.mp4')],check=True)
(dst/'result.json').write_text((src/'result.json').read_text())
print(dst)

Image.open(src/f'top_cam_{rows[0]["step"]:04d}.png').save(dst/'setup.png')

from pathlib import Path
import sys,subprocess
from PIL import Image
src=Path(sys.argv[1]);out=src/('model_run.mp4' if 'dream' in src.name else 'demo.mp4')
p=subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1920x352','-r','4','-i','-','-an','-c:v','libx264','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE)
for f in sorted(src.glob('top_cam_*.png')):
 k=int(f.stem.split('_')[-1]);im=Image.new('RGB',(1920,352))
 for i,cam in enumerate(['top_cam','left_cam','right_cam']):im.paste(Image.open(src/f'{cam}_{k:04d}.png').resize((640,352)),(i*640,0))
 p.stdin.write(im.tobytes())
p.stdin.close();assert p.wait()==0;print(out)

"""Export the three recorded native cameras side by side at simulated real time."""
from pathlib import Path
import argparse,subprocess
from PIL import Image
p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('--out',required=True);a=p.parse_args();src=Path(a.source);out=Path(a.out);out.parent.mkdir(parents=True,exist_ok=True)
frames=sorted(src.glob('top_cam_*.png'))
if not frames:raise RuntimeError('No recorded observations found')
w,h=Image.open(frames[0]).size
pipe=subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{3*w}x{h}','-r','4','-i','-','-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE)
for frame in frames:
    step=int(frame.stem.split('_')[-1]);canvas=Image.new('RGB',(3*w,h))
    for i,name in enumerate(['top_cam','left_cam','right_cam']):canvas.paste(Image.open(src/f'{name}_{step:04d}.png').convert('RGB'),(i*w,0))
    pipe.stdin.write(canvas.tobytes())
pipe.stdin.close()
if pipe.wait()!=0:raise RuntimeError('Video encoding failed')
print(out)

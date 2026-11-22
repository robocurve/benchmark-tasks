"""Export a full-scale torso surface and one continuous printable tape face."""
from pathlib import Path
import numpy as np
from geometry import points,faces,WIDTH,DX,N
from printed_scale import digits
out=Path(__file__).with_name('physical-templates');out.mkdir(exist_ok=True)
lines=['solid torso_mm']
for face in faces:
 for j in range(1,len(face)-1):
  tri=np.array([points[face[0]],points[face[j]],points[face[j+1]]])*1000
  n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);n/=max(np.linalg.norm(n),1e-12)
  lines.append('facet normal '+' '.join(map(str,n)));lines.append('outer loop')
  lines += ['vertex '+' '.join(map(str,p)) for p in tri]
  lines += ['endloop','endfacet']
lines.append('endsolid torso_mm');(out/'torso-mm.stl').write_text('\n'.join(lines)+'\n')
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1080mm" height="20mm" viewBox="0 0 1080 20">','<rect width="1080" height="20" fill="#faf2cc"/>']
for cm in range(108):
 svg.append(f'<rect x="{cm*10-.225}" y="6.5" width=".45" height="7" fill="#1f1a14"/>')
 if cm%5==0:
  for cx,cy,wx,wy in digits(str(cm)):
   # Match the panel's local x=width, y=length axes.
   svg.append(f'<rect x="{cm*10+(cy-wy/2)*1000}" y="{10+(cx-wx/2)*1000}" width="{wy*1000}" height="{wx*1000}" fill="#111"/>')
svg.append('</svg>');(out/'paper-tape-face.svg').write_text('\n'.join(svg)+'\n')
print(out)

"""CPU-only full twelve-panel OBB certificate, not native dynamics evidence."""
import argparse,json
from pathlib import Path
import numpy as np
from collapse_geometry import audit


def check():
    angles={n:0. for n in ('north','south','east','west')};failures=[];count=0
    for name in ('south','north','east','west'):
        for angle in np.linspace(0.,270.,136):
            angles[name]=float(angle);result=audit(90.,angles);count+=1
            if result['penetrations']:failures.append(dict(phase='opening',flap=name,angle_deg=float(angle),**result))
    for theta in np.linspace(90.,1.1,179):
        result=audit(float(theta));count+=1
        if result['penetrations']:failures.append(dict(phase='shear',theta_deg=float(theta),**result))
    return dict(kind='cpu_geometry_only',configurations=count,passed=not failures,failures=failures,
                final_at_270_deg=audit(1.1),final_at_268_5_deg=audit(1.1,{n:268.5 for n in angles}),native_dynamics_validated=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path);a=p.parse_args();result=check();text=json.dumps(result,indent=2)+'\n'
    if a.out:a.out.write_text(text)
    print(text);raise SystemExit(0 if result['passed'] else 1)

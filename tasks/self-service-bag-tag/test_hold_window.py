"""Test the actual runtime hold block with synthetic corners, without Isaac."""
import ast,json
from pathlib import Path
import numpy as np
import tag_closure as s
root=Path(__file__).parent
module=ast.parse((root/'tag_closure.py').read_text())
step=next(n for n in module.body if isinstance(n,ast.FunctionDef) and n.name=='step')
def target(n,name):
 return isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets)
a=next(i for i,n in enumerate(step.body) if target(n,'corners'))
b=next(i for i,n in enumerate(step.body) if target(n,'released_stable'))
fn=ast.parse('def hold(xyz,R,final,dt):\n pass').body[0]
fn.body=step.body[a:b+1]
code=ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[]));ns=dict(s.__dict__);exec(compile(code,'runtime-hold-block','exec'),ns)
s.reset();x=np.zeros((10,3));R=np.tile(np.eye(3),(10,1,1));hold=ns['hold']
hold(x,R,True,.05)
for _ in range(41):hold(x,R,True,.05)
assert s.STATE['stable_s']>=2.
hold(x+np.array([.006,0,0]),R,True,.05);assert s.STATE['stable_s']==0.
hold(x,R,True,.05);assert s.STATE['stable_s']==0.,'returning to start must not restore old credit'
for _ in range(10):hold(x,R,True,.05)
hold(x,R,False,.05);assert s.STATE['stable_s']==0.
hold(x,R,True,.05);assert s.STATE['stable_s']==0.
# Unchanged centres with a large rotation must also reset the hold.
for _ in range(10):hold(x,R,True,.05)
from scipy.spatial.transform import Rotation
Q=R.copy();Q[0]=Rotation.from_euler('x',30,degrees=True).as_matrix()
hold(x,Q,True,.05);assert s.STATE['stable_s']==0.
(root/'runs').mkdir(exist_ok=True)
(root/'runs/hold-window-regression.json').write_text(json.dumps({'passed':True,'kind':'actual runtime block tested on synthetic poses','checks':['2s valid hold','6mm excursion resets','return does not restore credit','invalid prerequisite resets','rotation resets']},indent=2))
print('5 runtime hold-window tests passed')

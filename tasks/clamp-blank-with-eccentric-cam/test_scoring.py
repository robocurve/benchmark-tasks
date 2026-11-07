from pathlib import Path
from types import SimpleNamespace
import sys,numpy as np,math
root=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'runtime'),str(root)]
import task
s=SimpleNamespace(pos=task.GOAL.copy(),quat_xyzw=np.array([0.,0.,0.,1.]),lin_vel=np.zeros(3),ang_vel=np.zeros(3),resting=lambda *args:True)
task.reset_history();task._loaded=True;task._q=math.radians(95);task._open=True;task._released=True
assert task.score({'blank':s})==.5,'A closed cam without measured preload is not a completed clamp'
task._stop=.001
assert task.score({'blank':s})==1.
task._released=False
assert task.score({'blank':s})==.75,'Robot contact must prevent released full score'
task._released=True;s.pos[1]+=.0025
assert task.score({'blank':s})==.5,'A registered but separated blank must not count as clamped'
print('PASS released clamp proof')

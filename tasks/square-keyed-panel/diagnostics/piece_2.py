"""Physical single-placement diagnostic. All four parts start in the original layout.

This cannot demonstrate full assembly; only one selected piece is manipulated.
"""
import importlib.util
from pathlib import Path
root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('square_first_source',root/'task.py')
task=importlib.util.module_from_spec(spec);spec.loader.exec_module(task)
SCENE,OBJECTS,score,success,distance=task.SCENE,task.OBJECTS,task.score,task.success,task.distance

def __getattr__(name):return getattr(task,name)

def demo():
    return {'left':[w for w in task.demo()['left'] if w.label.startswith('2:') or w.label=='park left clear']}

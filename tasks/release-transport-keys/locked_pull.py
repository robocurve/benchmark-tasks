"""Counterfactual validation only. Attempt carrier extraction while both keys remain engaged."""
import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('transport_reference',Path(__file__).with_name('task.py'))
reference=importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)
SCENE=reference.SCENE
OBJECTS=reference.OBJECTS
SUCCESS_HOLD_S=reference.SUCCESS_HOLD_S
SCORE_OVER=reference.SCORE_OVER
success=reference.success
score=reference.score
distance=reference.distance
observe=reference.observe
reset_history=reference.reset_history

def demo():
    return {'left':reference.demo()['left'][-8:]}

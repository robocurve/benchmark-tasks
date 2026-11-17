"""Offline model-path and canonical-goal checks; never calls Dream."""
import ast,json,sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from launch import cli_args
from assembly.instruction import INSTRUCTION

def test_same_canonical_goal():
    assert ast.get_docstring(ast.parse((ROOT/'task.py').read_text()))==INSTRUCTION
    assert json.loads((ROOT/'task.json').read_text())['instruction']==INSTRUCTION
    assert cli_args('dream',2400,Path('/tmp/unused'))[cli_args('dream',2400,Path('/tmp/unused')).index('--instruction')+1]==INSTRUCTION

def test_model_protocol_without_script_delay():
    args=cli_args('dream',2400,Path('/tmp/unused'))
    assert args[args.index('--policy')+1]=='dream'
    for setting in ('record=false','dry_run=false','rows_per_request=48','preroll_steps=32','object_jitter=0','control_hz=20','cam_width=320','cam_height=176'):
        assert setting in args
    assert not any('hold_s=' in s for s in args)

@pytest.mark.parametrize('n',[0,2401,-1,20.5,True])
def test_bad_budgets_rejected(n):
    with pytest.raises(ValueError):cli_args('dream',n,Path('/tmp/unused'))

def test_dream_has_no_demo_plan_loading():
    tree=ast.parse((ROOT/'launch.py').read_text())
    # The entire plan-read/controller installation is guarded by the demo mode.
    for node in ast.walk(tree):
        if isinstance(node,ast.If) and ast.unparse(node.test)=="a.mode == 'demo'":
            block=ast.unparse(node)
            if 'install_controller' in block:break
    else:raise AssertionError('Missing exclusive demo-controller branch')
    assert "allow_demo_evidence=a.mode=='demo'" in (ROOT/'launch.py').read_text()

"""No network or model clients: actual parser and isolated Dream-branch checks."""
import json,sys
from types import ModuleType
import pytest
import launch

@pytest.mark.parametrize('mode',['demo','dream','stability'])
def test_original_contract(mode,tmp_path):
    from inspect_robots.cli import build_parser,_parse_kvs
    args=build_parser().parse_args(launch.cli_args(mode,2400,tmp_path)[1:])
    env=_parse_kvs(args.embodiment_args)
    assert args.instruction==json.loads((launch.ROOT/'task.json').read_text())['instruction']
    assert args.epochs==1 and args.max_steps==2400 and args.embodiment=='yam_arena'
    assert env['scene']=='station' and env['object_jitter']==0 and env['control_hz']==20
    assert (env['cam_width'],env['cam_height'])==(320,176)
    if mode=='dream':
        assert args.policy=='dream'
        assert _parse_kvs(args.policy_args)==dict(model=launch.DREAM_MODEL,record=False,dry_run=False,rows_per_request=48,preroll_steps=32,timeout_s=300)

def test_dream_never_installs_scripted_controller(monkeypatch,tmp_path):
    import guide_task.image_health
    monkeypatch.setattr(guide_task.image_health,'install',lambda output:lambda:None)
    calls=[];runtime=ModuleType('guide_task.runtime');cli=ModuleType('inspect_robots.cli')
    def install(output,record_until_s=None):
        assert record_until_s is None
        calls.append('runtime');return lambda:calls.append('saved')
    runtime.install=install
    cli.main=lambda:calls.append('cli')
    monkeypatch.setitem(sys.modules,'guide_task.runtime',runtime)
    monkeypatch.setitem(sys.modules,'guide_task.controller',None)
    monkeypatch.setitem(sys.modules,'inspect_robots.cli',cli)
    monkeypatch.setattr(sys,'argv',['launch','--mode','dream','--log-dir',str(tmp_path)])
    launch.main();assert calls==['runtime','cli','saved']
    assert not (tmp_path/'controller_trace.json').exists()

def test_config_only_cannot_start_physics(monkeypatch,tmp_path):
    monkeypatch.setitem(sys.modules,'guide_task.runtime',None)
    monkeypatch.setattr(sys,'argv',['launch','--mode','dream','--config-only','--log-dir',str(tmp_path)])
    assert launch.main()==0
    assert json.loads((tmp_path/'configuration.json').read_text())['config_only']

@pytest.mark.parametrize('steps',[0,2401,-1])
def test_budget_limits(steps,tmp_path):
    with pytest.raises(ValueError):launch.cli_args('dream',steps,tmp_path)

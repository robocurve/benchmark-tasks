"""Model branch must not silently inherit scripted scene/action shortcuts."""
import importlib.util,sys
from pathlib import Path
import pytest,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from launch import cli_args
from carton_probe.instruction import INSTRUCTION

def test_dream_inputs_and_budget():
    args=cli_args('dream',2400,Path('/tmp/unused'))
    assert args[args.index('--policy')+1]=='dream'
    assert args[args.index('--instruction')+1]==INSTRUCTION
    assert 'record=false' in args and 'dry_run=false' in args
    assert 'rows_per_request=48' in args and 'preroll_steps=32' in args
    assert 'object_jitter=0' in args and 'control_hz=20' in args
    assert not any('hold_s=' in x for x in args)

@pytest.mark.parametrize('n',[0,2401,-1,20.5,True])
def test_invalid_budget(n):
    with pytest.raises(ValueError):cli_args('dream',n,Path('/tmp/unused'))

def test_layout_preserves_every_pixel():
    from export_media import compose_frame
    rng=np.random.default_rng(23)
    images={c:rng.integers(0,256,(176,320,3),dtype=np.uint8) for c in ('top_cam','left_cam','right_cam')}
    out=compose_frame(images,42)
    assert np.array_equal(out[24:376:2,::2],images['top_cam'])
    assert np.array_equal(out[400:,:320],images['left_cam'])
    assert np.array_equal(out[400:,320:],images['right_cam'])


def test_empty_recording_is_not_valid_rendering():
    from carton_probe.image_health import ImageMonitor
    assert not ImageMonitor().report()['valid']

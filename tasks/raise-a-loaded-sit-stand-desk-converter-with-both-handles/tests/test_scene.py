"""The USD asset is what tools/build_assets.py makes from layout.py, and has the structure the adapter expects."""

import importlib.util
import math
import re
from pathlib import Path

from rdb_desk_converter import fixed_scene as F
from rdb_desk_converter import layout as L

ROOT = Path(__file__).resolve().parents[1]
TEXT = F.USD.read_text()


def _builder():
    spec = importlib.util.spec_from_file_location("build_assets", ROOT / "tools/build_assets.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_asset_is_up_to_date():
    assert _builder().usd() == TEXT


def test_one_articulation_tree_with_expected_joints():
    assert TEXT.count("PhysicsArticulationRootAPI") == 1
    assert "physxArticulation:enabledSelfCollisions = 1" in TEXT
    for name in F.JOINTS:
        assert re.search(rf'def Physics\w+Joint "{name}"', TEXT)
    assert 'def PhysicsFixedJoint "fix_base"' in TEXT
    assert TEXT.count("PhysxMimicJointAPI:rotX") == 2
    assert "physxMimicJoint:rotX:gearing = 1\n" in TEXT and "physxMimicJoint:rotX:gearing = -1\n" in TEXT


def test_spring_and_latch_drives_match_layout():
    k = re.search(r'drive:angular:physics:stiffness = ([\d.e-]+)', TEXT).group(1)
    assert float(k) == __import__("pytest").approx(L.SPRING_K * math.pi / 180, rel=1e-5)
    assert f"drive:linear:physics:stiffness = {F.LATCH_K:.6g}" in TEXT
    assert f"drive:linear:physics:targetPosition = {-F.LATCH_PRELOAD:.6g}" in TEXT


def test_colliders_hidden_display_visible():
    # every collider cube is invisible except the lock teeth, pins and the card (which are what they look like)
    for block in re.findall(r'def Cube "(\w+)" \(\n\s+prepend apiSchemas = \[[^\]]*PhysicsCollisionAPI[^\]]*\]\n\s+\)\n\s+\{([^}]*)\}', TEXT):
        name, body = block
        visible = 'visibility = "invisible"' not in body
        assert visible == (name.startswith("tooth_") or name in ("pin", "card", "card_foot")), name


def test_loads_are_free_bodies_outside_the_articulation():
    loads = TEXT[TEXT.index('def Scope "Loads"'):TEXT.index('def Scope "Marks"')]
    assert 'def Xform "monitor"' in loads and 'def Xform "keyboard"' in loads
    assert "ArticulationRootAPI" not in loads

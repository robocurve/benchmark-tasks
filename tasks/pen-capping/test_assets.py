"""Offline checks for original, self-contained pen asset regeneration."""
from pathlib import Path

import pytest
from pxr import Usd, UsdGeom, UsdPhysics

import build_assets


@pytest.mark.parametrize('color', ['red', 'blue'])
def test_barrel_is_visible_original_geometry_with_four_unchanged_colliders(color):
    stage = Usd.Stage.Open(str(Path(__file__).parent / 'assets' / f'pen-{color}.usdc'))
    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert {p.GetName() for p in colliders} == {'Barrel', 'Shoulder', 'Nib', 'End'}
    assert all(p.IsA(UsdGeom.Cylinder) for p in colliders)
    assert not any(p.IsA(UsdGeom.Mesh) for p in stage.Traverse())
    assert all(UsdGeom.Imageable(p).ComputeVisibility() != 'invisible' for p in colliders)
    assert not stage.GetRootLayer().subLayerPaths
    assert not stage.GetRootLayer().GetExternalReferences()
    assert UsdPhysics.MassAPI(stage.GetDefaultPrim()).GetMassAttr().Get() == pytest.approx(.035)


@pytest.mark.parametrize('color', ['red', 'blue'])
def test_pens_regenerate_without_any_source_mesh_and_match_checked_in_layers(tmp_path, monkeypatch, color):
    reference = Usd.Stage.Open(str(Path(__file__).parent / 'assets' / f'pen-{color}.usdc'))
    monkeypatch.setattr(build_assets, 'HERE', tmp_path)
    build_assets.build_pens()
    regenerated = Usd.Stage.Open(str(tmp_path / 'assets' / f'pen-{color}.usdc'))
    assert regenerated.GetRootLayer().ExportToString() == reference.GetRootLayer().ExportToString()

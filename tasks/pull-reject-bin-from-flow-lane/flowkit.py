"""flowkit: a yamkit Prop whose colliders carry their own physics material (friction and combine mode).

A carton-flow lane's skate wheels let a bin roll down a 3-degree slope with almost no resistance along the lane. The
simulator has no wheels here: the lane's bed is one flat collider with a low friction (0.03) and the combine mode
"min", so the bin's own material (yam_arena gives every prop the grippy 1.2/1.0 rubber-like material, combine
"average") does not raise it. PhysX takes the higher-priority combine mode of the pair (average < min < multiply <
max), so "min" wins over "average" and the bin meets 0.03 on the bed, below tan(3 degrees) = 0.052: it runs down.

    import flowkit as fk
    LANE = fk.FrictionProp("lane", pos=..., meshes=..., colliders=..., friction=0.03, combine="min")

How: yamkit writes each prop's USD text in `_usda`; this module wraps that function. For a FrictionProp it adds a
physics material under the prop's root and binds it to every collider (a direct binding on the collider prim, which
wins over a weaker binding on the root). Other props are written unchanged. Plain Python.

The spawner must not bind a material of its own: yamkit gives every Prop yam_arena's grippy material, and Isaac Lab's
spawner binds it to each collider prim with "stronger than descendants", replacing the binding above (on the GPU the
lane then had friction 1.2/1.0 and the bins never ran down). So this module also wraps yamkit's scene hook: a
FrictionProp is spawned with no physics material, keeping its own.
"""
from __future__ import annotations

from dataclasses import dataclass

import sys

import yamkit as kit


@dataclass(frozen=True)
class FrictionProp(kit.Prop):
    """A Prop whose colliders all get one physics material: static = dynamic = `friction`, no bounce, `combine` mode
    ("average", "min", "multiply" or "max")."""

    friction: float = 0.5
    combine: str = "average"


def _material(prop: FrictionProp) -> str:
    return ('    def Material "PhysMat" (\n        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            '    )\n    {\n'
            f'        float physics:staticFriction = {prop.friction}\n'
            f'        float physics:dynamicFriction = {prop.friction}\n'
            '        float physics:restitution = 0\n'
            f'        uniform token physxMaterial:frictionCombineMode = "{prop.combine}"\n'
            f'        uniform token physxMaterial:restitutionCombineMode = "min"\n    }}\n')


def usda(prop) -> str:
    """yamkit's USD text, plus the physics material for a FrictionProp."""
    text = _ORIGINAL(prop)
    if not isinstance(prop, FrictionProp):
        return text
    head = '    def Scope "Looks"\n'
    assert head in text, "yamkit's USD layout changed: no Looks scope"
    text = text.replace(head, _material(prop) + head, 1)
    old = '        prepend apiSchemas = ["PhysicsCollisionAPI"]\n    )\n    {\n'
    new = ('        prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]\n    )\n    {\n'
           '        rel material:binding:physics = </Root/PhysMat>\n')
    assert old in text, "yamkit's USD layout changed: no collider schema line"
    return text.replace(old, new)


_ORIGINAL = getattr(kit._usda, "_flowkit_original", kit._usda)
usda._flowkit_original = _ORIGINAL
kit._usda = usda


def _wrap_scene(scene) -> None:
    """Spawn FrictionProps with no physics material of the spawner's (yamkit's hook would bind the grippy one)."""
    if getattr(scene, "_flowkit", False) or not getattr(scene, "_yamkit", False):
        return
    scene._flowkit = True
    task_object, grippy = scene.task_object, scene._grippy

    def wrapped(spec):
        if not isinstance(spec, FrictionProp):
            return task_object(spec)
        scene._grippy = lambda: None
        try:
            return task_object(spec)
        finally:
            scene._grippy = grippy

    scene.task_object = wrapped


_PATCH = getattr(kit._patch, "_flowkit_original", kit._patch)


def _patch(scene) -> None:
    _PATCH(scene)
    _wrap_scene(scene)


_patch._flowkit_original = _PATCH
kit._patch = _patch
if "yam_arena.scene" in sys.modules:
    _wrap_scene(sys.modules["yam_arena.scene"])

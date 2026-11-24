"""Run from task package cwd using the shipped standalone PhysX environment.

From the package root: python -m validation.cushion.ablate reference filter results/run/filter

The only scene edit is a body-pair exclusion between the sole and all arm bodies.
Save pair targets, contact observations, and endpoint from fresh simulation.
"""

import argparse
import gzip
import json
from pathlib import Path
from hashlib import sha256
from unittest.mock import patch

from pxr import Usd, UsdPhysics
from isaac import standalone


def execute(mode, treatment, out):
    out.mkdir(parents=True, exist_ok=True)
    original_make = standalone.make_scene
    original_bank = standalone.ContactMatrix
    bank_inputs = {}
    summary = dict(treatment=treatment, mode=mode)

    def make_scene(path, fixture):
        arms, shapes = original_make(path, fixture)
        stage = Usd.Stage.Open(str(path))
        sole = stage.GetPrimAtPath("/Bedside/cushion")
        arm_paths = sorted(arms.values())
        assert sole.HasAPI(UsdPhysics.RigidBodyAPI)
        assert all(
            stage.GetPrimAtPath(p).HasAPI(UsdPhysics.RigidBodyAPI) for p in arm_paths
        )
        assert [row[2] for row in shapes if row[1] == "cushion"] == ["sole"]
        pad_bodies = {f"foam{i}" for i in range(6)}
        pads = [r for r in shapes if r[1] in pad_bodies]
        assert sorted(r[2] for r in pads) == [f"pad{i}" for i in range(6)]
        assert len(set(arm_paths)) == len(arm_paths) == 18
        if treatment == "filter":
            pairs = UsdPhysics.FilteredPairsAPI.Apply(sole)
            pairs.CreateFilteredPairsRel().SetTargets(arm_paths)
            stage.GetRootLayer().Save()
            del pairs
            stage = Usd.Stage.Open(str(path))
        targets = (
            UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath("/Bedside/cushion"))
            .GetFilteredPairsRel()
            .GetTargets()
        )
        assert sorted(map(str, targets)) == (arm_paths if treatment == "filter" else [])
        summary["body_targets"] = sorted(map(str, targets))
        summary["sole_colliders"] = [r[0] for r in shapes if r[1] == "cushion"]
        bank_inputs["sensors"] = [r for r in shapes if r[1] == "cushion"] + pads
        bank_inputs["filters"] = [(p, p, "") for p in arm_paths]
        return arms, shapes

    class AuditedBank(original_bank):
        def __init__(self, eng, sensors, filters):
            super().__init__(eng, sensors, filters)
            self.contacts = original_bank(
                eng, bank_inputs["sensors"], bank_inputs["filters"]
            )
            self.steps_read = 0
            self.steps = {"sole": 0, "foam": 0}
            self.peaks = {"sole": 0.0, "foam": 0.0}

        def read(self):
            heel = super().read()
            magnitudes = dict(sole=0.0, foam=0.0)
            for body, _, _, _, magnitude in self.contacts.read():
                magnitudes["sole" if body == "cushion" else "foam"] += magnitude
            for surface, m in magnitudes.items():
                self.steps[surface] += int(m > 1e-6)
                self.peaks[surface] = max(self.peaks[surface], m)
            self.steps_read += 1
            if self.steps_read == 3360:
                summary["physics_steps"] = self.steps_read
                summary["robot_contact_steps"] = self.steps.copy()
                summary["peak_reported_contact_magnitude"] = self.peaks.copy()
                self.contacts.b.destroy()
            return heel

    with (
        patch.object(standalone, "make_scene", make_scene),
        patch.object(standalone, "ContactMatrix", AuditedBank),
    ):
        standalone.simulate(mode, out)
    record = json.loads((out / (mode + ".json")).read_text())
    with gzip.open(out / (mode + ".jsonl.gz"), "rt") as f:
        initial = json.loads(next(f))["initial"]["poses"]["cushion"][0]
        for row in f:
            endpoint = json.loads(row)
    summary["cushion_displacement_x_mm"] = (
        endpoint["raw"]["poses"]["cushion"][0] - initial
    ) * 1000
    task_root = Path(standalone.__file__).resolve().parents[1]
    summary["sources_sha256"] = {
        name: sha256((task_root / name).read_bytes()).hexdigest()
        for name in (
            "isaac/standalone.py",
            "isaac/readout.py",
            "task/scoring.py",
            "task/witness.py",
        )
    }
    summary["fixture_sha256"] = record["fixture_sha256"]
    summary["score"] = record["score"]
    summary["reason"] = record["reason"]
    summary["actions_sha256"] = record["actions_sha256"]
    for field in ("heel_channel_n", "heel_clear_mm"):
        summary[field] = record["final"]["witness"][field]
    (out / (mode + "-contact.json")).write_text(json.dumps(summary, indent=2) + "\n")
    assert summary["physics_steps"] == 3360
    if treatment == "filter":
        assert summary["robot_contact_steps"]["sole"] == 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["reference", "fixed_heel_on_foam", "idle"])
    parser.add_argument("treatment", choices=["stock", "filter"])
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    execute(args.mode, args.treatment, args.output.resolve())

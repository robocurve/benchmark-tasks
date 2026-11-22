"""One loose-stock repair setup for every policy."""
import repair_scene

BODIES, FIXED = repair_scene.BODIES, repair_scene.FIXED
CONTACT_CAPACITY = 2048


def build(stage):
    repair_scene.build('repair',stage)


def after_contact_batch(poses, forces, paths, filters, sample, hz):
    repair_scene.CURRENT.update(poses[-1], forces, paths, filters, sample, hz)
    repair_scene.CURRENT.measure_park(poses, forces, paths, filters, hz)


def debug_metrics():
    repair = repair_scene.CURRENT
    return {} if repair is None else {**repair.diagnostics, **repair.last_metrics}


def install_proof_load(sim):
    repair_scene.CURRENT.install_load(sim)


def save_probe(out):
    if repair_scene.CURRENT is not None:
        repair_scene.CURRENT.save(out)

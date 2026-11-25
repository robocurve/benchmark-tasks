"""Start layouts of the two liquids (bead centres stacked in each beaker's bore, station frame) for
tasks/dilute-sulfuric-acid: python make_beads.py <task folder>. Rerun after moving a beaker."""
import sys
from pathlib import Path

import numpy as np

task_dir = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(task_dir))
import task  # noqa: E402

for name, xy, r_in, floor, n, seed, base in (("acid", task.A_XY, task.A_RIN, task.A_FLOOR, task.N_ACID, 1, task.TOP),
                                             ("water", task.W_XY, task.W_RIN, task.W_FLOOR, task.N_WATER, 2, task.W_Z)):
    p, r = task.bead_layout(xy, r_in, floor, n, seed, base)
    np.savez(task_dir / "assets" / f"{name}_start.npz", pos=p.astype(np.float32), radius=r.astype(np.float32))
    print(name, len(p), "beads, top", round((p[:, 2].max() - base) * 1000), "mm above its beaker's foot")

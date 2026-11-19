"""Read-only liquid containment scoring; native execution is validated separately."""
import numpy as np

TARGET_ML = 400.0
INNER_RADIUS = .036
OUTER_RADIUS = .040
BASE = .008
RIM = .131
REST_OFFSET = .0015
# Calibration candidate for coupled 16/4 solver at 480 Hz in this glass.
# Native 100/420 mL bulk estimates fit 0.01868278 mL/sample; held-out checks pending.
QUANTUM_ML = .0187


class BeerScore:
    def __init__(self):
        self.entered = np.zeros(0, dtype=bool)
        self.spilled = np.zeros(0, dtype=bool)
        self.retained = 0

    def update(self, positions, glass_position, glass_rotation):
        """Consume consecutive, unsmoothed physics positions in world metres.

        Particle order is append-only. Pending inflow is neither retained nor
        spilled until it reaches the glass's rim plane or escapes the workspace.
        """
        p = np.asarray(positions, dtype=float).reshape(-1, 3)
        center = np.asarray(glass_position, dtype=float)
        rotation = np.asarray(glass_rotation, dtype=float)
        if (not np.isfinite(p).all() or center.shape != (3,)
                or rotation.shape != (3, 3) or not np.isfinite(center).all()
                or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
                or np.linalg.det(rotation) < .999):
            raise ValueError('Invalid native liquid or glass state')
        if len(p) < len(self.entered):
            raise ValueError('Native particles disappeared; evaluation incomplete')
        added = len(p) - len(self.entered)
        self.entered = np.pad(self.entered, (0, added))
        self.spilled = np.pad(self.spilled, (0, added))
        local = (p - center) @ rotation
        radial = np.linalg.norm(local[:, :2], axis=1)
        # Include the resolved boundary shell; do not reject particles merely
        # because their finite-radius collision response keeps them off the wall.
        inside = (radial <= INNER_RADIUS) & (local[:, 2] >= BASE) & (local[:, 2] <= RIM)
        # A central splash above the opening is airborne, not served or spilled.
        # Escaping the outer footprint or missing below the rim is a spill.
        # Soft contact may leave a centre in the physical wall/base thickness.
        # That band earns no volume credit and is not an exterior escape.
        envelope = (radial <= OUTER_RADIUS) & (local[:, 2] >= 0.) & (local[:, 2] <= RIM)
        escaped = self.entered & (radial > OUTER_RADIUS)
        if rotation[2,2]>1e-6:
            above=self.entered & (local[:,2]>RIM)
            # Airborne footprint is the rim's horizontal projection, rather
            # than an extension of the tilted sidewall above the opening.
            height=(RIM-local[above,2])/rotation[2,2]
            mouth_xy=local[above,:2]+height[:,None]*rotation[2,:2]
            escaped[above]=np.linalg.norm(mouth_xy,axis=1)>OUTER_RADIUS
        missed = (~self.entered & ~envelope & (local[:, 2] < RIM)
                  & (radial > OUTER_RADIUS))
        lowest_rim = center[2] + RIM*rotation[2,2] - OUTER_RADIUS*np.linalg.norm(rotation[2,:2])
        # A tilted or inverted glass must not leave tabletop puddles classified
        # as airborne inflow merely because its local rim plane points sideways.
        missed |= ~envelope & ((p[:,2] < lowest_rim) | (p[:,2] <= .7475 + 2*REST_OFFSET))
        below_base = local[:, 2] < 0.
        out_of_workspace = ~envelope & ((np.abs(p[:, 0]) > 1.5) | (np.abs(p[:, 1]) > 1.) | (p[:, 2] < .70))
        self.spilled |= escaped | missed | below_base | out_of_workspace
        self.entered |= inside
        self.retained = int(np.count_nonzero(inside))
        spill_ml = float(np.count_nonzero(self.spilled) * QUANTUM_ML)
        volume = float(self.retained * QUANTUM_ML)
        return dict(score=0. if spill_ml else min(volume / TARGET_ML, 1.),
                    retained_ml=volume, spilled_ml=spill_ml,
                    emitted_ml=float(len(p) * QUANTUM_ML),
                    pending_particles=int(np.count_nonzero(~inside & ~self.spilled)),
                    spill_latched=bool(spill_ml))

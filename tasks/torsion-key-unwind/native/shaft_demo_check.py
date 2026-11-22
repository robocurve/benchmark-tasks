"""Pre-pull qualification for the two scripted demonstrations."""
if __package__:
    from .shaft_contract import PRELOAD, LIMIT
else:
    from shaft_contract import PRELOAD, LIMIT

class HubGrasp:
    """Count each new degree of unloading once, only across two opposed samples."""
    def __init__(self):
        self.furthest = 0.0
        self.turn = 0.0
        self.ticks = 0

    def observe(self, angle, both):
        progress = min(PRELOAD, max(self.furthest, -angle))
        if both and self.ticks:
            self.turn += progress - self.furthest
        self.ticks = self.ticks+1 if both else 0
        self.furthest = progress

def ready(anchor, rotor, key_mm, opposed_key_contact, failed):
    return (not failed and abs(PRELOAD+anchor)<3 and abs(PRELOAD+anchor-rotor)<LIMIT
            and key_mm<3 and opposed_key_contact)


def flank_trim(anchor_deg):
    """Degrees of additional flank motion after the initial 34-degree tool path."""
    import math
    if not math.isfinite(anchor_deg):
        raise ValueError("Invalid input hub angle")
    return min(8.0,max(0.0,PRELOAD+anchor_deg))

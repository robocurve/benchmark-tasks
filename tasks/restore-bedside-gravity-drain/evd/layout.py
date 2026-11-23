from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATUM = 0.850
LAYOUTS = {
    "raised": (210, 90),
    "lowered": (90, 210),
    "aligned": (210, 210),
    "shifted": (234, 114),
}
INSTRUCTION = "The bed has finished moving. Return the bedside drain to service at the posted setting and leave it operating."
LINKS = (
    "base",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "gripper",
    "tip_left",
    "tip_right",
)
NAMES = ("mast", "carriage", "brake", "tap", "patient")
MODES = {
    "reference": ("raised", "guide"),
    "lowered": ("lowered", "guide"),
    "aligned": ("aligned", "guide"),
    "alternate": ("shifted", "guide"),
    "low_open": ("raised", "open"),
    "high_open": ("lowered", "open"),
    "idle": ("raised", "idle"),
    "paused": ("raised", "pause"),
    "held": ("raised", "held"),
    "blocked": ("raised", "blocked"),
    "dream": ("raised", "dream"),
    "no_lift_support": ("raised", "guide"),
    "no_brake_release": ("raised", "guide"),
    "framing": ("raised", "idle"),
}

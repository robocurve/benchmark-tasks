"""Stand-in for the two Inspect Robots classes the scripted reference emits, for dry runs without the runtime."""

from dataclasses import dataclass, field


@dataclass
class Action:
    data: object
    meta: dict = field(default_factory=dict)


@dataclass
class ActionChunk:
    actions: list
    control_hz: float = 20.

"""Collect colliding actor and shape paths during each measured physics substep."""

import math
from dataclasses import dataclass, field


@dataclass
class ContactPairs:
    impulses: dict = field(default_factory=dict)
    pairs: set = field(default_factory=set)
    collider_pairs: set = field(default_factory=set)


class PairMonitor:
    def __init__(self, decode):
        self.decode = decode
        self.current = ContactPairs()
        self.error = None

    def on_contacts(self, headers, points):
        try:
            for h in headers:
                if h.num_contact_data <= 0:
                    continue
                actors = tuple(
                    sorted(
                        str(self.decode(getattr(h, k))) for k in ("actor0", "actor1")
                    )
                )
                vertical, total = self.current.impulses.get(actors,(0.,0.))
                for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                    impulse = points[i].impulse
                    vertical += abs(impulse[2])
                    total += math.sqrt(sum(impulse[k]**2 for k in range(3)))
                self.current.impulses[actors] = (vertical,total)
                shapes = tuple(
                    sorted(
                        str(self.decode(getattr(h, k)))
                        for k in ("collider0", "collider1")
                    )
                )
                self.current.pairs.add(actors)
                self.current.collider_pairs.add(shapes)
        except (ValueError, TypeError, AttributeError) as exc:
            self.error = exc

    def drain(self):
        if self.error is not None:
            raise RuntimeError(
                "Native contact data could not be decoded"
            ) from self.error
        result = self.current
        self.current = ContactPairs()
        return result

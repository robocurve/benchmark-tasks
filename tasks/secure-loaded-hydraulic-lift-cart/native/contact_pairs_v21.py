"""Collect colliding actor and shape paths during each measured physics substep."""

from dataclasses import dataclass, field


@dataclass
class ContactPairs:
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

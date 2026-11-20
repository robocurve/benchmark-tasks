"""Released retention and even fit at the two opposed hems."""

class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable = 0

    def evaluate(self, state):
        seated = sum(state["seated"].values())
        self.stable = self.stable + 1 if state["cover_flat"] and state["hands_clear"] and seated == 2 else 0
        return {"score": 1.0 if self.stable >= 20 else .5 if seated and state["hands_clear"] else 0.,
                "success": self.stable >= 20, "settled_actions": self.stable, **state}

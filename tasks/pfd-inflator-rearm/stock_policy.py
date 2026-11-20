"""Two wrist observations select between two known-layout manipulation paths."""
import json
import numpy as np
def inspect(rgb):
    if rgb is None or rgb.shape != (176,320,3) or rgb.dtype != np.uint8:
        return {"reading":"unknown"}
    gray = rgb[72:103,141:173].mean(axis=2)
    y,x = np.ogrid[72:103,141:173]
    radius = (x-156.5)**2 + (y-87)**2
    face = float(np.median(gray[(radius>=25)&(radius<65)]))
    contrast = max(12., .14*face)
    rim = float(np.mean(gray[(radius>=110)&(radius<200)] < face-contrast))
    spots = int(np.count_nonzero(gray[11:20,12:20] < face-max(12.,.10*face)))
    label = "unknown"
    if 40 < face < 248 and rim > .55:
        if spots == 0:
            label = "intact"
        elif spots >= 3:
            label = "recessed"
    return dict(reading=label,face=face,rim_fraction=rim,dark_pixels=spots)



class StockInspection:
    def __init__(self):
        self.samples = {"A": [], "B": []}
        self.choice = None

    def command(self, entry, images, action):
        slot = entry.get("stock_view")
        if slot:
            self.samples[slot].append(inspect(images.get("right_cam")))
        if "alternate_action" not in entry:
            return action
        if self.choice is None:
            observed = []
            for readings in self.samples.values():
                last = [r["reading"] for r in readings[-8:]]
                observed.append(last[0] if len(last)==8 and len(set(last))==1 else "unknown")
            self.choice = {("intact","recessed"): "A", ("recessed","intact"): "B"}.get(tuple(observed),"hold")
        if self.choice == "hold":
            from yam_arena.constants import READY
            return np.r_[READY,1.,READY,1.]
        return np.asarray(entry["alternate_action"],float) if self.choice=="B" else action

    def save(self, path):
        (path/"stock-decision.json").write_text(json.dumps(dict(intact_slot=self.choice, samples=self.samples),indent=2)+"\n")

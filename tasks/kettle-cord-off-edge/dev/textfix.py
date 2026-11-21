"""Text-only fixes (switch condition, strip not on mains, the ability sentence, estimated cord length and lamp colour).
Usage: python dev/textfix.py   (edits task.json and dev/texts.py in place, prints the lengths)"""
import json, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
pairs = [
    ("filled with 1.0 L, lid shut (a real trial: unplugged, cold, 1.0 kg of dry ballast in a sealed bag instead of water).",
     "filled with 1.0 L, lid shut (real trial: unplugged, cold, 1.0 kg of dry ballast in a sealed bag, strip not connected to mains)."),
    ("and the blue power lamp on the handle's back lights.",
     "and a power lamp on the handle's back lights (blue here; colour estimated)."),
    ("Cord: 76 cm, 7 mm, black.", "Cord: 76 cm, 7 mm, black (estimated; the manual gives no length)."),
    ("switch down at its stop (lamp lit).", "switch down at its stop (in simulation the lamp also lights)."),
    ("draw the cord back onto the counter, and only then switch the kettle on.",
     "draw the cord back onto the counter, and only then switch the kettle on. The ability this adds to the benchmark: "
     "noticing a cord that hangs over the counter edge and drawing it back onto the counter before switching the "
     "appliance on."),
]
tp = root / "task.json"
t = json.loads(tp.read_text())
for a, b in pairs:
    hit = [k for k in ("setup", "scoring", "context") if a in t[k]]
    assert len(hit) == 1, a
    t[hit[0]] = t[hit[0]].replace(a, b)
tp.write_text(json.dumps(t, indent=2, ensure_ascii=False) + "\n")
xp = root / "dev" / "texts.py"
src = xp.read_text()
for k in ("SETUP", "SCORING"):  # keep dev/texts.py in step: rewrite its two strings from task.json
    start = src.index(f"{k} = (")
    end = src.index("\n)\n", start) + 3
    src = src[:start] + f"{k} = (\n    {json.dumps(t[k.lower()], ensure_ascii=False)}\n)\n" + src[end:]
xp.write_text(src)
for k in ("setup", "scoring", "context", "fidelity"):
    print(k, len(t[k]))

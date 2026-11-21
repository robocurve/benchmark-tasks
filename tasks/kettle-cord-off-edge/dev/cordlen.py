"""Loop depth for a few cord lengths, and the cord length from the pinch to the plug and to the base (no simulation)."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import numpy as np
import layout as L
for cl in (0.70, 0.76, 0.80):
    L.CORD_LEN = cl
    n, d = L.cord_nodes()
    print(cl, "depth", round(d, 3), len(n) - 1, "links")
L.CORD_LEN = float(sys.argv[1]) if len(sys.argv) > 1 else 0.76
L.N_LINKS = round(L.CORD_LEN / L.LINK)
n, d = L.cord_nodes()
P = np.array(n)
cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
ks = [k for k in range(len(P)) if k < len(P) // 2 and P[k, 1] < 0.58 and P[k, 2] > L.TOP - 0.002]
k = max(ks, key=lambda k: P[k, 1])
print("pinch node", k, P[k].round(3), "to plug", round(cum[-1] - cum[k], 3), "to base", round(cum[k], 3))
for q in P[::3]:
    print(q.round(3))

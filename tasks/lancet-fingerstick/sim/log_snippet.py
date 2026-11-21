

# -- appended by sim/mk.sh to the sim copies only: a per-step trace of the lancet, the tab, the hand and the model -------
import os as _os  # noqa: E402

_TRACE = {"f": None, "n": 0}


@on_step
def _trace(env):
    try:
        _TRACE["n"] += 1
        if _TRACE["n"] % 2:
            return
        if _TRACE["f"] is None:
            # WHY in ../out and named by the trial's seed: the run scripts pull sim/out only, and runs of one copy on
            # several seeds must not overwrite each other's trace
            out = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "out")
            _os.makedirs(out, exist_ok=True)
            path = _os.path.join(out, f"trace_{_os.path.basename(__file__)[:-3]}_{run_seed()}.csv")
            _TRACE["f"] = open(path, "w")
            _TRACE["f"].write("step,q_mm,force,tab_deg,ring_q_mm,drop_mm,tab_off,out,positioned,fired,fire,free_t,"
                              "rx,ry,rz,rf,lx,ly,lz,lf,stage,fail,model_err,"
                              "Lx,Ly,Lz,Lax,Lay,Laz,Hx,Hy,Hz,hx,hy,hz,fpx,fpy,fpz,site,sd_mm,"
                              "j1,j2,j3,j4,j5,j6,t1,t2,t3,t4,t5,t6,gyaw,gax,gay,gaz\n")
        base = env.unwrapped
        lan, hand = base.scene["lancet"], base.scene["hand"]
        origin = _np(base.scene.env_origins)[0]
        pl = _np(lan.data.root_link_pose_w)[0]
        pb = _np(hand.data.root_link_pose_w)[0]
        ph = _np(hand.data.body_link_pose_w)[0][list(hand.body_names).index("ring")]
        lp, ax = pl[:3] - origin, _rot3(pl[3:7])[:, 2]
        face = face_w(lp, pl[3:7], _M["q"])
        pm = _rot3(ph[3:7]).T @ (face - (ph[:3] - origin))
        site = site_of(pm)
        m, mem = _M, _MEM
        st = mem["now"]["stage"] if mem.get("now") else -1
        r = m["right"] if m["right"] else (np.full(3, np.nan), float("nan"))
        lh = m["left"] if m["left"] else (np.full(3, np.nan), float("nan"))
        r4 = lambda v: round(float(v), 4)  # noqa: E731
        row = [_TRACE["n"], round(1000 * m["q"], 3), round(shield_force(m["q"]), 2), round(math.degrees(m["tab"]), 2),
               round(1000 * m["ring_q"], 3), round(1000 * ring_drop(m["ring_q"]), 3), int(m["tab_off"]),
               int(m["out"]), int(m["positioned"]), int(m["fired"]), str(m["fire"]).replace(",", ";"),
               round(m["free_t"], 2), *[r4(v) for v in r[0]], round(r[1], 5), *[r4(v) for v in lh[0]],
               round(lh[1], 5), st, str(mem.get("fail")).replace(",", ";"), str(m["error"]).replace(",", ";"),
               *[r4(v) for v in lp], *[r4(v) for v in ax], *[r4(v) for v in pb[:3] - origin],
               *[r4(v) for v in ph[:3] - origin], *[r4(v) for v in pm],
               (site[0].replace(",", ";") if site else "none"), (round(1000 * site[4], 2) if site else "")]
        rob = base.scene["right_robot"]
        jn = list(rob.joint_names)
        ji = [jn.index(f"joint{i}") for i in range(1, 7)]
        jq = _np(rob.data.joint_pos)[0]
        try:
            jt = _np(rob.data.joint_pos_target)[0]
        except Exception:  # noqa: BLE001
            jt = np.full(len(jn), np.nan)
        g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
        gR = _rot3(g[3:7])
        row += [r4(jq[i]) for i in ji] + [r4(jt[i]) for i in ji]
        row += [round(math.degrees(math.atan2(gR[1, 0], gR[0, 0])), 2), *[r4(v) for v in gR[:, 2]]]
        _TRACE["f"].write(",".join(str(v) for v in row) + "\n")
        _TRACE["f"].flush()
    except Exception as e:  # noqa: BLE001
        if _TRACE["f"] is not None:
            _TRACE["f"].write(f"# trace error {e!r}\n")

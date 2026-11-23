"""Retain real PhysX contact evidence; identify allowed contacts by the fixture collision shape."""


class ContactBook:
    def __init__(self, decode):
        self.decode = decode
        self.pending = set()
        self.observed = {}
        self.unsafe = {}
        self.streak = {}
        self.bilateral = {}
        self.error = None
        self.interarm_intervals = []

    def event(self, headers, points):
        try:
            for h in headers:
                if h.num_contact_data <= 0:
                    continue
                self.pending.add(
                    tuple(
                        str(self.decode(getattr(h, k)))
                        for k in ("collider0", "collider1")
                    )
                )
        except Exception as exc:
            self.error = exc

    def advance(self, tick):
        if self.error:
            raise RuntimeError("PhysX contact decoding failed") from self.error
        seen = set()
        interarm = False
        for a, b in self.pending:

            def robot(p):
                return (
                    "left"
                    if "/LeftArm/" in p
                    else "right"
                    if "/RightArm/" in p
                    else None
                )

            if not robot(a):
                a, b = b, a
            side = robot(a)
            if not side or robot(b) == side:
                continue
            if robot(b):
                interarm = True
            link = next(
                (s for s in ("tip_left", "tip_right") if "/" + s + "/" in a),
                "other_link",
            )
            # Keep the original body/link for non-pad touches, including a camera housing
            # if the installed robot provides one as a collision shape.
            robot_collider = a.split("/env_0/")[-1]
            obj = b.rsplit("/", 1)[-1]
            recognized = {
                "liftNeck": "carriage",
                "pressTab": "brake",
                "knob": "tap",
            }.get(obj)
            key = side + "/" + link + "::" + b.split("/env_0/")[-1]
            if link == "other_link" or robot(b):
                key = robot_collider + "::" + b.split("/env_0/")[-1]
            if key not in self.observed:
                self.observed[key] = [tick, tick, 0]
            self.observed[key][1:] = [tick, self.observed[key][2] + 1]
            if recognized and link != "other_link":
                seen.add((side, link, recognized))
            else:
                self.unsafe.setdefault(key, tick)
        for side, obj in [("left", "carriage"), ("right", "brake"), ("right", "tap")]:
            k = side + "_" + obj
            present = all((side, tip, obj) in seen for tip in ("tip_left", "tip_right"))
            self.streak[k] = self.streak.get(k, 0) + 1 if present else 0
            self.bilateral[k] = max(self.bilateral.get(k, 0), self.streak[k])
        if interarm:
            if self.interarm_intervals and self.interarm_intervals[-1][1] == tick - 1:
                self.interarm_intervals[-1][1] = tick
            else:
                self.interarm_intervals.append([tick, tick])
        self.pending.clear()

    def summary(self):
        return dict(
            contact_ranges=self.observed,
            forbidden=self.unsafe,
            interarm_intervals=self.interarm_intervals,
            bilateral_max_ticks=self.bilateral,
        )

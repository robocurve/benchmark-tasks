"""Fixed grading rules evaluated against measured tabletop-frame quantities."""
import math

def satisfies(values, requirements):
    return all(math.isfinite(values[r["metric"]]) and r["lo"] <= values[r["metric"]] <= r["hi"]
               for r in requirements)


class FirstExcursion:
    """Remember the pre-motion state and commit the first significant directed excursion."""
    def __init__(self,spec):
        self.spec=spec
        self.origin=None
        self.armed=False
        self.started=False
        self.continuous=False
        self.outcome=None

    def update(self,values,previous):
        if self.outcome is not None:
            return
        s=self.spec
        x=values[s["metric"]]*(1 if s["direction"]=="increase" else -1)
        # A fresh directional extreme, or a return to the unresolved origin, permits
        # the operator to set up the grasp again. Subthreshold outward jitter never
        # becomes an irreversible rejection. Away from the origin a late grasp cannot
        # rewrite the last pre-departure frame.
        if self.origin is None or x<=self.origin+s["noise"]:
            self.origin=x if self.origin is None else min(self.origin,x)
            self.armed=set(s["needs"])<=previous and any(satisfies(values,g) for g in s["onset_any"])
            self.started=False
            self.continuous=True
            return
        self.started=True
        self.continuous &= any(satisfies(values,g) for g in s["during_any"]) if s["during_any"] else True
        if x-self.origin>=s["travel"]-1e-12:
            self.outcome=bool(self.armed and self.continuous)


class Grade:
    def __init__(self, fixture):
        self.fixture = fixture
        self.progress = [0] * len(fixture["milestones"])
        self.seen = set()
        self.holds = [0] * len(fixture["tiers"])
        self.invalid = False
        self.excursions={x["name"]:FirstExcursion(x) for x in fixture.get("excursions",[])}

    def prime(self,measurements):
        """Set pre-action origins at the real settled reset, without counting a timestep."""
        for watch in self.excursions.values():
            watch.update(measurements,set())

    def update(self, measurements):
        if not all(math.isfinite(v) for v in measurements.values()):
            raise ValueError("Non-finite simulator measurement")
        previous=set(self.seen)
        for group in self.fixture["invalidate"]:
            self.invalid |= satisfies(measurements, group)
        for guard in self.fixture.get("violations",[]):
            if (set(guard["needs"])<=previous and not (set(guard["until_seen"]) & previous)
                    and satisfies(measurements,guard["all"])):
                self.invalid=True
        for name,watch in self.excursions.items():
            watch.update(measurements,previous)
            if watch.outcome is True:self.seen.add(name)
            if watch.outcome is False and watch.spec["reject_on_failure"]:self.invalid=True
        for i, milestone in enumerate(self.fixture["milestones"]):
            matches = set(milestone.get("needs",[]))<=previous and satisfies(measurements, milestone["all"])
            self.progress[i] = self.progress[i] + 1 if matches else 0
            if self.progress[i] >= milestone["ticks"]:
                self.seen.add(milestone["name"])
        score = 0.0
        for i, tier in enumerate(self.fixture["tiers"]):
            matches = (set(tier["needs"]) <= self.seen and satisfies(measurements, tier["all"]))
            self.holds[i] = self.holds[i] + 1 if matches else 0
            if self.holds[i] >= tier["ticks"] and not self.invalid:
                score = max(score, tier["score"])
        detail={"score":score,"measurements":dict(measurements),"invalid":self.invalid,
                "milestones":sorted(self.seen),"hold_ticks":max(self.holds,default=0)}
        if self.excursions:
            detail["excursions"]={name:{"origin":w.origin,"armed_at_origin":w.armed,"departed":w.started,
                                        "condition_held":w.continuous,"outcome":w.outcome}
                                   for name,w in self.excursions.items()}
        return detail


class TaskScorer:
    name = "physical_task_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score
        grade = record.steps[-1].result.info["grade"] if record.steps else {"score": 0.0}
        return Score(value=grade["score"], explanation=str(grade))


def make_scorer():
    return TaskScorer()

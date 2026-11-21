"""Qualification of scripted rail transfers using native contact evidence."""
class GripGate:
    def __init__(self, plans):
        self.segments = []
        arm = None
        for left,right in zip(plans["left"], plans["right"], strict=True):
            assert left.label == right.label and left.duration == right.duration
            if left.label == "hover open":
                arm = "left" if left.pos[1] > 0 and left.pos[1] < .1 else "right"
            self.segments.append((left.duration,left.label,arm))
            if left.label == "retreat":
                arm = None
        self.streak = 0
        self.legs = []
        self.label = ""
        self.arm = None
        self.y = None
        self.rest = None
        self.lift = None

    def begin(self, time):
        label,arm="done",None
        for duration,phase,hand in self.segments:
            if time < duration:
                label,arm=phase,hand
                break
            time -= duration
        self.begin_phase(label,arm)

    def begin_phase(self,label,arm):
        if label == "lift vertically" and label != self.label and self.streak < 24:
            raise ValueError("Lift blocked: opposed grasp unconfirmed")
        if label == "carry pickup" and self.label != label and self.streak < 24:
            raise ValueError(f"Carry blocked before command: {self.streak}/24 consecutive opposed-contact substeps")
        if label == "open at mark" and self.label != label:
            if self.y is None or abs(self.y - self.goal(arm)) > 6 or self.streak < 24:
                raise ValueError(f"Release blocked: carriage at {self.y} mm")
        if label == "clear pickup" and self.label != label:
            if self.streak < 12:
                raise ValueError("Retraction blocked: release unconfirmed")
            self.legs.append(arm)
        if label != self.label:
            self.streak = 0
        self.label,self.arm = label,arm

    @staticmethod
    def goal(arm):
        return -66 if arm == "left" else 66

    def sample(self,row):
        if not self.arm:
            return
        poses = row["poses"]
        self.y = 1000*(poses["sled"][1]-poses["frame"][1])
        height=1000*(poses["block"][2]-poses["sled"][2])
        if self.label=="hover open":
            self.rest=height
        if self.rest is None:
            raise ValueError("Missing measured resting height")
        self.lift=height-self.rest
        sides = {}
        for body,contacts in row["pairs"].items():
            for link in contacts:
                tip = link.rsplit("/",1)[-1]
                if body != "block" or f"/{self.arm.title()}Arm/" not in link or tip not in {"tip_left","tip_right"}:
                    raise ValueError(f"Unexpected {self.arm} transfer contact: {body} {link}")
                sides[tip] = set(row["pickup_faces"].get(link,[]))
        opposed = any(sign in sides.get("tip_left",()) and -sign in sides.get("tip_right",()) for sign in (-1,1))
        if self.label in {"hover open","straddle pickup","close on pickup","verify hold","lift vertically","verify lift"} and abs(self.y+self.goal(self.arm)) > 2:
            raise ValueError(f"Carriage moved before grasp qualification: {self.y:.2f} mm")
        if self.label == "verify hold":
            self.streak = self.streak+1 if opposed else 0
        if self.label=="verify lift":
            self.streak=self.streak+1 if opposed and self.lift>=8 else 0
        if self.label=="verify seated":
            self.streak=self.streak+1 if opposed and abs(self.lift)<=2 and abs(self.y-self.goal(self.arm))<=6 else 0
        if self.label in {"carry pickup","settle held"} and self.lift<8:
            raise ValueError(f"Pickup dropped to {self.lift:.2f} mm above rest")
        if self.label in {"lift vertically","verify lift","carry pickup","settle held","lower at mark","verify seated"} and not opposed:
            raise ValueError("Opposed pickup contacts lost during travel or settle")
        if self.label == "open at mark":
            self.streak = self.streak+1 if not sides else 0
        if self.label in {"clear pickup","retreat"} and (sides or abs(self.y-self.goal(self.arm))>6 or abs(self.lift)>2):
            raise ValueError("Block moved or remained in contact after release")

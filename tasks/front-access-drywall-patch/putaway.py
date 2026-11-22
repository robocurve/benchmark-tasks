"""Detect a controlled touchdown using the ordinary trigger, table and gripper contacts."""


class Touchdown:
    def __init__(self):
        self.clear_samples=0
        self.armed=False
        self.seen=False
        self.previous_off=False
        self.event=None

    def update(self,table_load,grip,off,hz,start):
        for i,(load,held,released) in enumerate(zip(table_load,grip,off,strict=True)):
            if load<=.15:
                self.clear_samples+=1
                if self.clear_samples>=hz//10:
                    self.armed=True
                    self.seen=False
            else:
                if self.armed:
                    self.seen=bool(held and released and self.previous_off)
                    self.event=dict(sample=int(start+i),under_gripper_control=bool(held),
                                    trigger_released_before_contact=bool(released and self.previous_off))
                    self.armed=False
                self.clear_samples=0
            self.previous_off=bool(released)

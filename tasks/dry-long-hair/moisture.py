"""Energy-limited moisture prototype for measured hair-surface patches.

This models local state only, not hair dynamics, occlusion or a functioning
robot scene. The native adapter must derive exposed area/airflow from geometry.
No drying from a scripted visit flag or elapsed time alone.
"""
from dataclasses import dataclass
import math


@dataclass
class Patch:
    area: float
    dry_heat_capacity: float
    water: float
    temperature: float=20.

    def step(self,dt,air_temperature,heat_transfer,exposed_fraction,ambient=20.):
        if not all(math.isfinite(v) for v in (dt,air_temperature,heat_transfer,exposed_fraction,self.water,self.temperature)):
            raise ValueError('nonfinite thermal state')
        if not (0<dt<=.05 and self.area>0 and self.dry_heat_capacity>0 and
                heat_transfer>=0 and 0<=exposed_fraction<=1 and self.water>=0):
            raise ValueError('invalid material or airflow sample')
        # Candidate evaporation onset is a model parameter, not a clinical
        # or real-world hair/scalp safety recommendation.
        incoming=max(0.,heat_transfer*self.area*exposed_fraction*(air_temperature-self.temperature)*dt)
        capacity=self.dry_heat_capacity+self.water*4180.
        warming=max(0.,35.-self.temperature)*capacity
        used=min(warming,incoming)
        self.temperature+=used/capacity
        evaporative_energy=max(0.,incoming-used)
        removed=min(self.water,evaporative_energy/2_260_000.)
        self.water-=removed
        leftover=evaporative_energy-removed*2_260_000.
        self.temperature+=leftover/(self.dry_heat_capacity+self.water*4180.)
        self.temperature+=(ambient-self.temperature)*min(1.,.02*dt)
        return removed

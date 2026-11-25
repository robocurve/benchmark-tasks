// The heat-set rule of task.py, for the no-heat emulator plate. Pure C++ (no Arduino calls), so it can be tested on
// a computer: g++ -O2 -o test_rule test_rule.cpp && ./test_rule
#pragma once
#include <math.h>

namespace heatset {

const float T_AMB = 22.0f, T_IRON = 225.0f;  // C: the room; the iron's set temperature
const float T_FLOW = 160.0f;                 // C: PLA melts; above this an insert may move
const float TAU_HEAT = 2.0f;                 // s: towards T_IRON while the iron's tip touches the insert
const float TAU_COOL = 15.0f;                // s: towards T_AMB once it has lifted
const float TAU_BLOCK = 4.0f;                // s: the extra cooling while the steel block touches it

struct Insert {
  float temp = T_AMB;
  bool soft() const { return temp >= T_FLOW; }
};

// One step of dt seconds. iron / block: whether the iron's tip / the block touches this insert now.
// Returns whether the insert is soft (the brake may release).
inline bool step(Insert &ins, float dt, bool iron, bool block) {
  if (iron) {
    ins.temp += (T_IRON - ins.temp) * (1.0f - expf(-dt / TAU_HEAT));
  } else {
    float rate = 1.0f / TAU_COOL + (block ? 1.0f / TAU_BLOCK : 0.0f);
    ins.temp += (T_AMB - ins.temp) * (1.0f - expf(-dt * rate));
  }
  return ins.soft();
}

}  // namespace heatset

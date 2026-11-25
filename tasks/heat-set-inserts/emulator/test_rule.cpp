// Checks the emulator's rule against the cases task.py's self_test uses:  g++ -O2 -o test_rule test_rule.cpp && ./test_rule
#include <assert.h>
#include <stdio.h>
#include "rule.h"
using namespace heatset;

float run(Insert &i, float secs, bool iron, bool block, float dt = 0.01f) {
  for (int n = 0; n < (int)(secs / dt + 0.5f); n++) step(i, dt, iron, block);
  return i.temp;
}

int main() {
  Insert a;  // from cold, about 2.3 s of contact makes it soft
  run(a, 2.0f, true, false);
  assert(!a.soft());
  run(a, 0.4f, true, false);
  assert(a.soft());
  Insert b;  // a brief touch (0.2 s) does nothing the block could use
  run(b, 0.2f, true, false);
  run(b, 5.0f, false, true);
  assert(!b.soft() && b.temp < T_AMB + 30);
  Insert c;  // short touches with pauses never make it soft
  float peak = T_AMB;
  for (int k = 0; k < 5; k++) { run(c, 0.4f, true, false); if (c.temp > peak) peak = c.temp; run(c, 2.0f, false, false); }
  assert(peak < T_FLOW);
  Insert d;  // the demo's way: 3 s + 3.9 s pressed; soft for about 5 s on its own, about 1.5 s under the block
  run(d, 6.9f, true, false);
  float t = 0; Insert e = d;
  while (e.soft()) { step(e, 0.01f, false, false); t += 0.01f; }
  assert(t > 4.0f && t < 6.5f);
  t = 0; e = d;
  while (e.soft()) { step(e, 0.01f, false, true); t += 0.01f; }
  assert(t > 1.0f && t < 2.0f);
  printf("emulator rule: all cases pass (after 6.9 s pressed: %.0f C; soft for %.1f s under the block)\n", d.temp, t);
  return 0;
}

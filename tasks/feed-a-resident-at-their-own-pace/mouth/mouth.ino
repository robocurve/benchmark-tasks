// Mouth controller for the feeding manikin (Arduino Nano). The same rules as task.py:
//   rest: mouth closed. When something is within 6 cm of the upper lip (IR sensor), the mouth is armed (it has seen
//   nothing within 8 cm since it last closed) and 1.5 s have passed since it closed, and the tongue is idle:
//   open over 0.5 s, hold T_open, close over 0.4 s, whatever is in it.
//   A bite landing on the tongue (a steady load rise of 2.5 g or more) is held for the next T_hold of the lot; at the first
//   moment after that with the mouth at rest, the tongue tips 52 deg over 0.4 s, holds 0.6 s and returns over 0.4 s,
//   dropping whatever lies on it into the cup.
// Lot: send 'A', 'B' or 'C' over serial (115200 baud) before the trial, then 'R' (or press the reset button), which
// closes the mouth, levels the tongue and zeroes the load cell. Log at 50 Hz: t_ms, mouth state, jaw deg, IR cm,
// tongue g, event.
//
// Wiring: jaw servo (MG996R, through the slip clutch) D9; tongue servo (MG90S) D10; Sharp GP2Y0A41SK0F IR sensor
// (4-30 cm) A0, behind a 4 mm hole in the upper lip; HX711 with a 100 g load cell under the tongue: DOUT D4, SCK D5;
// reset button D2 to GND. Servos on their own 6 V supply, grounds joined. Libraries: Servo, HX711 (bogde).

#include <HX711.h>
#include <Servo.h>

const int PIN_JAW = 9, PIN_TONGUE = 10, PIN_IR = A0, PIN_DOUT = 4, PIN_SCK = 5, PIN_RESET = 2;

// calibration (set once per build; see README "Real rig")
const float JAW_CLOSED_DEG = 90.0, JAW_OPEN_DEG = 111.5;      // 21.5 deg: 30 mm between the incisor edges
const float TONGUE_LEVEL_DEG = 90.0, TONGUE_TIP_DEG = 141.6;  // 51.6 deg (0.90 rad)
float LOAD_SCALE = 2280.0;                                     // HX711 counts per gram
const float IR_A = 12.08, IR_B = -1.058;                       // cm = IR_A * volts^IR_B (GP2Y0A41SK0F datasheet fit)

const float NEAR_CM = 6.0, AWAY_CM = 8.0, COOLDOWN_S = 1.5;
const float T_OPENING = 0.5, T_CLOSING = 0.4, T_TIP = 0.4, T_TIPPED = 0.6, T_UNTIP = 0.4;
const float LAND_G = 2.5;          // a rise at least this big over the baseline, steady (within 0.5 g) for 0.3 s,
const float STEADY_G = 0.5, STEADY_S = 0.3;   // with the spoon beyond 6 cm of the lip, is a bite landing

struct Lot { float t_open; float hold[3]; };
const Lot LOTS[3] = {{5.0, {50.0, 4.0, 5.0}}, {4.5, {5.0, 50.0, 4.0}}, {4.0, {4.0, 6.0, 5.0}}};

Servo jaw, tongue;
HX711 scale;
int lot = 0;
enum Mouth { REST, OPENING, OPEN, CLOSING } mouth = REST;
bool armed = true, tipping = false;
float since = 0, cool_until = 0, tongue_since = 0, base_g = 0, ref_g = 0, ref_since = 0;
int landings = 0, held = 0;
float due[8];

float now_s() { return millis() / 1000.0; }

float ir_cm() {
  float v = analogRead(PIN_IR) * 5.0 / 1023.0;
  return v < 0.25 ? 99.0 : IR_A * pow(v, IR_B);
}

void reset_trial() {
  mouth = REST; armed = true; tipping = false; cool_until = 0; landings = 0; held = 0;
  jaw.write(JAW_CLOSED_DEG); tongue.write(TONGUE_LEVEL_DEG);
  delay(800);
  scale.tare(20); base_g = 0; ref_g = 0; ref_since = now_s();
  Serial.print("reset lot "); Serial.println((char)('A' + lot));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RESET, INPUT_PULLUP);
  jaw.attach(PIN_JAW); tongue.attach(PIN_TONGUE);
  scale.begin(PIN_DOUT, PIN_SCK); scale.set_scale(LOAD_SCALE);
  reset_trial();
}

void loop() {
  static unsigned long next = 0;
  if (millis() < next) return;
  next = millis() + 20;                                        // 50 Hz
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= 'A' && c <= 'C') lot = c - 'A';
    if (c == 'R') reset_trial();
  }
  if (digitalRead(PIN_RESET) == LOW) reset_trial();

  float t = now_s(), d = ir_cm(), g = scale.is_ready() ? scale.get_units(1) : base_g;
  const char *event = "";
  if (d > AWAY_CM) armed = true;

  // tongue: landings and swallows
  if (fabs(g - ref_g) > STEADY_G) { ref_g = g; ref_since = t; }
  if (!tipping && g - base_g >= LAND_G && t - ref_since >= STEADY_S && d > NEAR_CM && held < 8) {   // spoon out
    due[held++] = t + LOTS[lot].hold[min(landings, 2)];
    landings++; event = "landed"; base_g = g;
  }
  if (!tipping && g < base_g - STEADY_G && t - ref_since >= STEADY_S) { base_g = g; event = "load drop"; }
  bool any_due = false;
  for (int i = 0; i < held; i++) any_due |= t >= due[i];
  if (!tipping && held > 0 && any_due && mouth == REST) { tipping = true; tongue_since = t; event = "swallow"; }
  if (tipping) {
    float dt = t - tongue_since, a;
    if (dt < T_TIP) a = dt / T_TIP;
    else if (dt < T_TIP + T_TIPPED) a = 1.0;
    else if (dt < T_TIP + T_TIPPED + T_UNTIP) a = 1.0 - (dt - T_TIP - T_TIPPED) / T_UNTIP;
    else { a = 0.0; tipping = false; held = 0; base_g = g; }
    tongue.write(TONGUE_LEVEL_DEG + a * (TONGUE_TIP_DEG - TONGUE_LEVEL_DEG));
  }

  // mouth
  float dt = t - since, a = 0.0;
  switch (mouth) {
    case REST:
      if (d < NEAR_CM && armed && t >= cool_until && !tipping) { mouth = OPENING; since = t; event = "opens"; }
      break;
    case OPENING:
      a = min(1.0, dt / T_OPENING);
      if (dt >= T_OPENING) { mouth = OPEN; since = t; }
      break;
    case OPEN:
      a = 1.0;
      if (dt >= LOTS[lot].t_open) { mouth = CLOSING; since = t; }
      break;
    case CLOSING:
      a = max(0.0, 1.0 - dt / T_CLOSING);
      if (dt >= T_CLOSING + 0.1) { mouth = REST; since = t; armed = false; cool_until = t + COOLDOWN_S; event = "closed"; }
      break;
  }
  jaw.write(JAW_CLOSED_DEG + a * (JAW_OPEN_DEG - JAW_CLOSED_DEG));

  Serial.print(millis()); Serial.print(',');
  Serial.print("ROCX"[mouth]); Serial.print(',');
  Serial.print(a * (JAW_OPEN_DEG - JAW_CLOSED_DEG), 1); Serial.print(',');
  Serial.print(d, 1); Serial.print(',');
  Serial.print(g, 2); Serial.print(',');
  Serial.println(event);
}

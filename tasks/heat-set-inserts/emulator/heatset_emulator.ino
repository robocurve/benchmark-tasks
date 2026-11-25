// No-heat emulator plate for the heat-set-inserts task (Arduino Nano or Uno). It runs the simulation's heating rule
// (rule.h) for each insert and releases that insert's brake only while the rule says it is soft; a red LED by each
// hole shows it. Contacts are sensed through 5 V and the inserts' internal pull-ups: the cold iron's tip and the steel
// block are each wired back to the board (thin flexible wire), and each insert to an input.
//
//   D2, D3   insert a, insert b            INPUT_PULLUP (read LOW when the tool being tested touches it)
//   D11      iron tip wire                 driven LOW while the iron is tested, high impedance otherwise
//   D12      block wire                    driven LOW while the block is tested, high impedance otherwise
//   D7, D8   brake a, brake b              HIGH = release (logic-level MOSFET, 5 V solenoid, flyback diode)
//   D9, D10  LED a, LED b                  HIGH = soft
//   Serial   115200 baud, one line every 0.5 s: t, each insert's temperature and contacts (keep it as the trial log)
#include "rule.h"

const uint8_t INS[2] = {2, 3}, BRAKE[2] = {7, 8}, LED[2] = {9, 10};
const uint8_t IRON = 11, BLOCK = 12;
const unsigned long STEP_MS = 10;  // 100 Hz
heatset::Insert ins[2];
unsigned long last = 0, lastLog = 0;

bool touches(uint8_t tool, uint8_t other, uint8_t insertPin) {
  pinMode(other, INPUT);  // high impedance
  pinMode(tool, OUTPUT);
  digitalWrite(tool, LOW);
  delayMicroseconds(50);
  bool t = digitalRead(insertPin) == LOW;
  pinMode(tool, INPUT);
  return t;
}

void setup() {
  for (int k = 0; k < 2; k++) {
    pinMode(INS[k], INPUT_PULLUP);
    pinMode(BRAKE[k], OUTPUT);
    pinMode(LED[k], OUTPUT);
    digitalWrite(BRAKE[k], LOW);  // locked (the solenoid's spring holds the pin in): power off = locked
  }
  pinMode(IRON, INPUT);
  pinMode(BLOCK, INPUT);
  Serial.begin(115200);
  last = lastLog = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - last < STEP_MS) return;
  float dt = (now - last) / 1000.0f;
  last = now;
  bool iron[2], block[2];
  for (int k = 0; k < 2; k++) {
    iron[k] = touches(IRON, BLOCK, INS[k]);
    block[k] = touches(BLOCK, IRON, INS[k]);
    bool soft = heatset::step(ins[k], dt, iron[k], block[k]);
    digitalWrite(BRAKE[k], soft ? HIGH : LOW);
    digitalWrite(LED[k], soft ? HIGH : LOW);
  }
  if (now - lastLog >= 500) {
    lastLog = now;
    Serial.print(now / 1000.0f, 1);
    for (int k = 0; k < 2; k++) {
      Serial.print(k ? " b " : " a ");
      Serial.print(ins[k].temp, 0);
      Serial.print(iron[k] ? " iron" : " -");
      Serial.print(block[k] ? " block" : " -");
    }
    Serial.println();
  }
}

// Arduino-compatible 200-step/rev motor with a STEP/DIR driver in full-step mode.
// A closed guard switch and a pressed start button each connect their input to GND.
constexpr byte GuardPin = 6;
constexpr byte StartPin = 7;
constexpr byte DirPin = 8;
constexpr byte StepPin = 9;
constexpr byte EnablePin = 10;  // The DRV8825 enable input is active low.
constexpr byte PumpDirection = LOW;
constexpr unsigned long StepPeriodUs = 12500;  // 200 steps/rev at 24 rpm.
constexpr unsigned int CheckSteps = 240;      // One 3.0 s check.

bool guardClosed() {
    return digitalRead(GuardPin) == LOW;
}

void setup() {
    digitalWrite(EnablePin, HIGH);
    pinMode(EnablePin, OUTPUT);
    pinMode(StepPin, OUTPUT);
    pinMode(DirPin, OUTPUT);
    pinMode(GuardPin, INPUT_PULLUP);
    pinMode(StartPin, INPUT_PULLUP);
    digitalWrite(DirPin, PumpDirection);
}

void loop() {
    if (!guardClosed() || digitalRead(StartPin) != LOW) {
        return;
    }
    delay(25);
    if (!guardClosed() || digitalRead(StartPin) != LOW) {
        return;
    }
    digitalWrite(EnablePin, LOW);
    delayMicroseconds(20);
    unsigned long next = micros();
    for (unsigned int step = 0; step < CheckSteps && guardClosed(); ++step) {
        digitalWrite(StepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(StepPin, LOW);
        next += StepPeriodUs;
        while (guardClosed() && static_cast<long>(micros() - next) < 0) {
        }
    }
    digitalWrite(EnablePin, HIGH);
    while (digitalRead(StartPin) == LOW) {
        delay(10);
    }
}

// DM556Y speed and direction control with potentiometer

const int STEP_PIN = 3;
const int DIR_PIN  = 4;
const int POT_PIN  = A0;

const int STEPS_PER_REV = 1600;
const float MAX_RPM = 700.0;

// Stop zone centered around 512
const int POT_CENTER = 512;
const int STOP_HALF_WIDTH = 150;
const int STOP_MIN = POT_CENTER - STOP_HALF_WIDTH; // 362
const int STOP_MAX = POT_CENTER + STOP_HALF_WIDTH; // 662

// DM556Y minimum pulse width
const unsigned int STEP_HIGH_US = 5;

// Potentiometer update interval
const unsigned long POT_UPDATE_INTERVAL_US = 5000; // 5 ms

// Light exponential smoothing
const float ALPHA = 0.25;

float filteredPot = 512.0;
float targetRPM = 0.0;
bool targetDir = LOW;
bool motorEnabled = false;

unsigned long lastPotReadTime = 0;
unsigned long lastStepToggleTime = 0;
unsigned long stepIntervalUs = 0;
bool stepState = false;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateMotionCommand() {
  int rawPot = analogRead(POT_PIN);
  filteredPot = ALPHA * rawPot + (1.0 - ALPHA) * filteredPot;

  float rpm = 0.0;
  bool dir = LOW;
  bool enable = false;

  if (filteredPot < STOP_MIN) {
    dir = LOW;
    rpm = mapFloat(filteredPot, 0, STOP_MIN, MAX_RPM, 0.0);
    enable = (rpm > 0.5);
  }
  else if (filteredPot > STOP_MAX) {
    dir = HIGH;
    rpm = mapFloat(filteredPot, STOP_MAX, 1023, 0.0, MAX_RPM);
    enable = (rpm > 0.5);
  }
  else {
    rpm = 0.0;
    enable = false;
  }

  targetRPM = rpm;
  targetDir = dir;
  motorEnabled = enable;

  digitalWrite(DIR_PIN, targetDir);

  if (motorEnabled) {
    float stepsPerSecond = (targetRPM * STEPS_PER_REV) / 60.0;
    if (stepsPerSecond >= 1.0) {
      stepIntervalUs = (unsigned long)(1000000.0 / stepsPerSecond);
    } else {
      motorEnabled = false;
    }
  }
}

void updateStepper() {
  if (!motorEnabled) {
    digitalWrite(STEP_PIN, LOW);
    stepState = false;
    return;
  }

  unsigned long now = micros();

  if (!stepState) {
    if (now - lastStepToggleTime >= stepIntervalUs) {
      digitalWrite(STEP_PIN, HIGH);
      stepState = true;
      lastStepToggleTime = now;
    }
  } else {
    if (now - lastStepToggleTime >= STEP_HIGH_US) {
      digitalWrite(STEP_PIN, LOW);
      stepState = false;
    }
  }
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  filteredPot = analogRead(POT_PIN);
  updateMotionCommand();
}

void loop() {
  unsigned long now = micros();

  if (now - lastPotReadTime >= POT_UPDATE_INTERVAL_US) {
    lastPotReadTime = now;
    updateMotionCommand();
  }

  updateStepper();
}
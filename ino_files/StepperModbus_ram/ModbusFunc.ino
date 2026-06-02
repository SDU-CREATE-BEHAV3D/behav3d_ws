// --- queue coils ---
enum q_coils {
  QUEUE_PUSH = 13,
  CANCEL_ALL = 14
};

enum ext_coils {
  EXTRUDE_MAN = 10,
  PRG_STATE = 11,
  EXTRUDE_PRG = 12,
};

// --- Input Register map for telemetry (read-only via Modbus) ---
#define IR_STEPS_LO 0
#define IR_STEPS_HI 1

// --- RAM extruder mechanics ---
const uint8_t LIMIT_SWITCH_PIN = CONTROLLINO_DI0;
const bool LIMIT_SWITCH_ACTIVE_STATE = LOW; // NC switch: open/tripped -> LOW

const bool DIR_AWAY_FROM_LIMIT_STATE = HIGH;
const bool DIR_TOWARD_LIMIT_STATE = LOW;

const unsigned long LIMIT_DEBOUNCE_MS = 10;

const uint32_t STEPS_PER_REV = 1600UL;
const float SCREW_LEAD_MM_PER_REV = 4.0f;
const float GEAR_REDUCTION = 16.0f;
const uint32_t STEPS_PER_MM = 6400UL;        // 1600 * 16 / 4
const uint32_t LIMIT_BACKOFF_MM = 4UL;
const uint32_t SOFTLIMIT_TRAVEL_MM = 305UL;
const uint32_t LIMIT_BACKOFF_STEPS = LIMIT_BACKOFF_MM * STEPS_PER_MM;
const uint32_t SOFTLIMIT_MAX_STEPS = SOFTLIMIT_TRAVEL_MM * STEPS_PER_MM;

// steps_accum and motion state are defined in InterruptSetup.ino
extern volatile uint32_t steps_accum;
extern volatile uint32_t soft_position_steps;
extern volatile bool soft_zero_known;
extern volatile uint32_t auto_backoff_steps_remaining;
extern volatile bool auto_backoff_active;
extern volatile bool auto_backoff_done;
extern volatile bool continuous_mode_active;
extern volatile int8_t motion_direction;

// Publish steps_accum into input registers IR0/IR1
static inline void publish_steps_accum() {
  uint32_t v;
  noInterrupts();
  v = steps_accum;
  interrupts();
  modbusTCPServer.inputRegisterWrite(IR_STEPS_LO, (uint16_t)(v & 0xFFFF));
  modbusTCPServer.inputRegisterWrite(IR_STEPS_HI, (uint16_t)((v >> 16) & 0xFFFF));
}

static bool queue_used_since_on = false;
static bool prev_queue_push = false;

static bool limit_raw_state = false;
static bool limit_debounced_state = false;
static bool prev_limit_debounced_state = false;
static unsigned long limit_last_change_ms = 0;

// step_rate_hz (steps/s). This is the base pulse frequency for STEP on D07 (Timer3).
unsigned long extruderFreq = 1;

// cfsMultiplier (%) scales the legacy Timer4 STEP on D06 (not wired today).
unsigned long cfsMultiplier = 1;

uint32_t read_steps_32_from_HR34() {
  uint16_t lo = (uint16_t)modbusTCPServer.holdingRegisterRead(3);
  uint16_t hi = (uint16_t)modbusTCPServer.holdingRegisterRead(4);
  return ((uint32_t)hi << 16) | (uint32_t)lo;
}

void add_steps_accum(uint32_t v) {
  if (v == 0) return;
  noInterrupts();
  uint32_t current = steps_accum;
  uint32_t space = 0xFFFFFFFFu - current;
  steps_accum = current + ((v > space) ? space : v);
  interrupts();
}

void clear_steps_accum() {
  noInterrupts();
  steps_accum = 0;
  interrupts();
}

bool isLimitSwitchActiveRaw() {
  return digitalRead(LIMIT_SWITCH_PIN) == LIMIT_SWITCH_ACTIVE_STATE;
}

void setupLimitSwitch() {
  pinMode(LIMIT_SWITCH_PIN, INPUT);
  bool active = isLimitSwitchActiveRaw();
  limit_raw_state = active;
  limit_debounced_state = active;
  prev_limit_debounced_state = active;
  limit_last_change_ms = millis();
}

void updateLimitSwitchDebounce() {
  bool raw = isLimitSwitchActiveRaw();
  unsigned long now = millis();

  if (raw != limit_raw_state) {
    limit_raw_state = raw;
    limit_last_change_ms = now;
  }

  if ((now - limit_last_change_ms) >= LIMIT_DEBOUNCE_MS) {
    limit_debounced_state = limit_raw_state;
  }
}

void finalizeAutoBackoffIfReady() {
  bool done = false;
  noInterrupts();
  done = auto_backoff_done;
  interrupts();

  if (!done) return;

  noInterrupts();
  soft_position_steps = 0;
  soft_zero_known = true;
  auto_backoff_done = false;
  interrupts();

  if (limit_debounced_state) {
    Serial.println("Limit backoff finished while switch is still active.");
  } else {
    Serial.println("Limit backoff finished. Soft zero set.");
  }

  // Keep the default ROS/manual direction aligned away from the limit unless ROS overrides coil 4.
  modbusTCPServer.coilWrite(4, DIR_AWAY_FROM_LIMIT_STATE);
}

void forceDirectionAwayFromLimit() {
  digitalWrite(CONTROLLINO_D4, DIR_AWAY_FROM_LIMIT_STATE);
  noInterrupts();
  motion_direction = +1;
  interrupts();
}

void startLimitBackoff(unsigned long* lastExtrude) {
  forceDirectionAwayFromLimit();
  PORTH &= ~(1 << 4); // ENA LOW -> driver enabled

  noInterrupts();
  steps_accum = LIMIT_BACKOFF_STEPS;
  auto_backoff_steps_remaining = LIMIT_BACKOFF_STEPS;
  auto_backoff_active = true;
  auto_backoff_done = false;
  continuous_mode_active = false;
  soft_zero_known = false;
  interrupts();

  *lastExtrude = millis();
  enableTimer3(true);

  Serial.println("Limit switch tripped. Starting 4 mm backoff.");
}

void cancelAllMotion(bool disableDriver) {
  enableTimer3(false);
  enableTimer4(false);

  noInterrupts();
  steps_accum = 0;
  auto_backoff_steps_remaining = 0;
  auto_backoff_active = false;
  auto_backoff_done = false;
  continuous_mode_active = false;
  motion_direction = 0;
  interrupts();

  queue_used_since_on = false;

  if (disableDriver) {
    PORTH |= (1 << 4); // ENA HIGH
  }
}

bool motionAllowed(bool movingAwayFromLimit) {
  bool zeroKnown;
  uint32_t pos;

  noInterrupts();
  zeroKnown = soft_zero_known;
  pos = soft_position_steps;
  interrupts();

  if (!zeroKnown) {
    return true;
  }

  if (movingAwayFromLimit) {
    return pos < SOFTLIMIT_MAX_STEPS;
  }

  return pos > 0;
}

void syncModbus(unsigned long* lastExtrude) {
  bool extruding;
  bool movingAwayFromLimit;
  bool backoffActive;

  // 1) Mirror coils 0..4 -> D00..D04
  for (int i = 0; i < 5; i++) {
    digitalWrite(CONTROLLINO_D0 + i, modbusTCPServer.coilRead(i));
  }

  // 2) Limit switch debounce and homing finalization
  updateLimitSwitchDebounce();
  finalizeAutoBackoffIfReady();

  // 3) Trigger backoff on debounced active edge
  if (limit_debounced_state && !prev_limit_debounced_state) {
    startLimitBackoff(lastExtrude);
  }
  prev_limit_debounced_state = limit_debounced_state;

  noInterrupts();
  backoffActive = auto_backoff_active;
  interrupts();

  if (backoffActive) {
    forceDirectionAwayFromLimit();
  }

  // 4) Extrusion gate: unchanged Modbus contract
  extruding = modbusTCPServer.coilRead(EXTRUDE_MAN) ||
              (modbusTCPServer.coilRead(EXTRUDE_PRG) && modbusTCPServer.coilRead(PRG_STATE));

  if (!backoffActive) {
    if (!extruding) {
      cancelAllMotion(true);
    } else {
      PORTH &= ~(1 << 4); // ENA LOW
      *lastExtrude = millis();
    }
  } else {
    PORTH &= ~(1 << 4); // keep driver enabled while backing off
  }

  // 5) Frequency updates
  if (syncHoldingReg(1, &extruderFreq) || syncHoldingReg(2, &cfsMultiplier)) {
    setTimer3Freq(extruderFreq);
    // Timer4 legacy intentionally left disabled
  }

  // 6) QUEUE_PUSH rising edge -> enqueue HR3|HR4 steps
  bool now_push = modbusTCPServer.coilRead(QUEUE_PUSH);
  if (now_push && !prev_queue_push) {
    if (!backoffActive) {
      uint32_t req = read_steps_32_from_HR34();
      if (req > 0) {
        add_steps_accum(req);
        queue_used_since_on = true;
        if (extruding) enableTimer3(true);
      }
    } else {
      Serial.println("Ignoring QUEUE_PUSH while limit backoff is active.");
    }
  }
  prev_queue_push = now_push;

  // 7) CANCEL_ALL
  if (modbusTCPServer.coilRead(CANCEL_ALL)) {
    cancelAllMotion(true);
    modbusTCPServer.coilWrite(CANCEL_ALL, 0);
  }

  // 8) Scheduler
  noInterrupts();
  backoffActive = auto_backoff_active;
  interrupts();

  if (backoffActive) {
    forceDirectionAwayFromLimit();
    enableTimer3(true);
  } else if (extruding) {
    movingAwayFromLimit = (modbusTCPServer.coilRead(4) == DIR_AWAY_FROM_LIMIT_STATE);

    noInterrupts();
    motion_direction = movingAwayFromLimit ? +1 : -1;
    interrupts();

    if (queue_used_since_on) {
      noInterrupts();
      continuous_mode_active = false;
      bool hasQueuedSteps = (steps_accum > 0);
      interrupts();

      if (hasQueuedSteps && motionAllowed(movingAwayFromLimit)) {
        enableTimer3(true);
      } else {
        enableTimer3(false);
      }
    } else {
      bool allowed = motionAllowed(movingAwayFromLimit);
      noInterrupts();
      continuous_mode_active = allowed;
      interrupts();
      enableTimer3(allowed);
    }
  } else {
    noInterrupts();
    continuous_mode_active = false;
    motion_direction = 0;
    interrupts();
    enableTimer3(false);
  }

  // 9) Idle sleep safety
  if (millis() - *lastExtrude > 60000UL) {
    PORTH |= (1 << 4); // ENA HIGH
  }

  // 10) Telemetry: publish queued steps remaining
  publish_steps_accum();
}

// stepperDriverOn/Off pulse Relay 8 (R8). This is separate from ENA (D05).
void stepperDriverOn() {
  digitalWrite(CONTROLLINO_R8, HIGH);
  delay(200);
  digitalWrite(CONTROLLINO_R8, LOW);
}

void stepperDriverOff() {
  digitalWrite(CONTROLLINO_D0, HIGH);
  delay(200);
  digitalWrite(CONTROLLINO_D0, LOW);
}

int syncHoldingReg(int regIndex, unsigned long* currentFreq) {
  long hrVal = modbusTCPServer.holdingRegisterRead(regIndex);
  if (hrVal < 0) {
    Serial.print("Failed to read holding register ");
    Serial.print(regIndex);
    Serial.println(" !");
    return 0;
  }

  if (hrVal == 0) return 0;
  if (hrVal == *currentFreq) return 0;

  *currentFreq = (unsigned long)hrVal;
  return 1;
}

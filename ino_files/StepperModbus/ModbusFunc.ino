// --- queue coils ---
enum q_coils {
  QUEUE_PUSH = 13,   // enqueue step request from HR3|HR4 (rising edge)
  CANCEL_ALL = 14    // cancel current job and clear FIFO
};
// --- Input Register map for telemetry (read-only via Modbus) ---
#define IR_STEPS_LO 0  // lower 16 bits of steps_accum
#define IR_STEPS_HI 1  // upper 16 bits of steps_accum

// steps_accum is defined in InterruptSetup.ino
extern volatile uint32_t steps_accum;

// Publish steps_accum into input registers IR0/IR1
static inline void publish_steps_accum() {
  uint32_t v;
  noInterrupts();
  v = steps_accum;      // atomic snapshot
  interrupts();
  modbusTCPServer.inputRegisterWrite(IR_STEPS_LO, (uint16_t)(v & 0xFFFF));
  modbusTCPServer.inputRegisterWrite(IR_STEPS_HI, (uint16_t)((v >> 16) & 0xFFFF));
}

// Tracks if the queue has been used since extruding went ON (for legacy continuous fallback)
static bool queue_used_since_on = false;

// Edge detection for QUEUE_PUSH
bool prev_queue_push = false;

// Read 32-bit step count from HR3|HR4
uint32_t read_steps_32_from_HR34() {
  uint16_t lo = (uint16_t)modbusTCPServer.holdingRegisterRead(3); // HR3 = low 16
  uint16_t hi = (uint16_t)modbusTCPServer.holdingRegisterRead(4); // HR4 = high 16
  return ((uint32_t)hi << 16) | (uint32_t)lo;
}

// Add steps to accumulator with saturation
void add_steps_accum(uint32_t v) {
  if (v == 0) return;
  noInterrupts();
  uint32_t current = steps_accum;
  uint32_t space   = 0xFFFFFFFFu - current;
  steps_accum = current + ((v > space) ? space : v);
  interrupts();
}

enum ext_coils {
  EXTRUDE_MAN = 10,
  PRG_STATE = 11,
  EXTRUDE_PRG = 12,
};

// step_rate_hz (steps/s). This is the base pulse frequency for STEP on D07 (Timer3).
unsigned long extruderFreq = 1;

// cfsMultiplier (%) scales the legacy Timer4 STEP on D06 (not wired today).
// 100 = same as extruderFreq, 50 = half, etc.
unsigned long cfsMultiplier = 1;


void syncModbus(unsigned long* lastExtrude) {
  // 1) Mirror coils 0..4 -> D00..D04 (DIR on D04)
  for (int i = 0; i < 5; i++) {
    digitalWrite(CONTROLLINO_D0 + i, modbusTCPServer.coilRead(i));
  }

  // 2) Extrusion gate: ON enables ENA and allows jobs to run. OFF cancels everything.
  int extruding = modbusTCPServer.coilRead(EXTRUDE_MAN) ||
                  (modbusTCPServer.coilRead(EXTRUDE_PRG) && modbusTCPServer.coilRead(PRG_STATE));

  if (!extruding) {
    // OFF: cancel and sleep
    enableTimer3(false);
    noInterrupts();
    steps_accum = 0;
    interrupts();
    PORTH |= (1 << 4); // ENA HIGH
    queue_used_since_on = false;
  } else {
    PORTH &= ~(1 << 4); // ENA LOW
    *lastExtrude = millis();
  }


  // 3) Frequency updates (speed). Timer3 drives STEP on D07.
  if (syncHoldingReg(1, &extruderFreq) || syncHoldingReg(2, &cfsMultiplier)) {
    setTimer3Freq(extruderFreq);
    // Timer4 legacy intentionally left disabled
    // setTimer4Freq(extruderFreq * (cfsMultiplier / 100.0f));
  }

  // 4) QUEUE_PUSH (coil 13) rising edge -> enqueue HR3|HR4 steps
  bool now_push = modbusTCPServer.coilRead(QUEUE_PUSH);
  if (now_push && !prev_queue_push) {
    uint32_t req = read_steps_32_from_HR34();
    if (req > 0) {
      add_steps_accum(req);
      queue_used_since_on = true;
      // If extruding and timer is currently off, start it immediately
      if (extruding) enableTimer3(true);
    }
  }
  prev_queue_push = now_push;



  // 5) CANCEL_ALL (coil 14)
  if (modbusTCPServer.coilRead(CANCEL_ALL)) {
    modbusTCPServer.coilWrite(CANCEL_ALL, 0); // auto-clear (momentary behavior)
  }

  // 6) Scheduler:

  // If extruding and no queued-steps usage in this ON session,
  // allow legacy continuous (free-run). Otherwise honor accumulator.
  if (extruding) {
    if (queue_used_since_on) {
      // Queue used: run only while steps_accum > 0
      if (steps_accum > 0) {
        enableTimer3(true);
      } else {
        enableTimer3(false);
      }
    } else {
      // Legacy continuous mode
      enableTimer3(true);
    }
  } else {
    enableTimer3(false);
  }



  // 7) Idle sleep safety (same as before)
  if (millis() - *lastExtrude > 60000) {
    PORTH |= (1 << 4); // ENA HIGH
  }
  // 8) Telemetry: publish steps remaining
  publish_steps_accum();

} // <--- make sure this is the closing brace of syncModbus(...)


// stepperDriverOn/Off pulse Relay 8 (R8). Used to power-cycle or latch driver supply.
// This is separate from ENA (D05), which gates the driver logic.

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

  *currentFreq = hrVal;
  return 1;
}

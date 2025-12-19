// Nidec 24H404H070 - Adaptive Bang-Bang Control
// Learns the motor's behavior and adapts pulse sizes
//
// Motor pins (8..1 on the PCB header):
// 8: +12V (VM)  -> Bench PSU +12V  (NOT to Arduino)
// 7: GND        -> Bench PSU GND + Arduino GND (common ground)
// 6: BRAKE (active LOW)        -> D6
// 5: RUN (PWM, active LOW)     -> D5
// 4: DIR (direction)           -> D4
// 3: +5V encoder Vcc           -> 5V
// 2: Encoder B (CHB)           -> D3 (unused here)
// 1: Encoder A (CHA)           -> D2 (+10k pullup to 5V strongly recommended)

const uint8_t PIN_DIR   = 4;
const uint8_t PIN_RUN   = 5;  // PWM, ACTIVE-LOW
const uint8_t PIN_BRAKE = 6;
const uint8_t PIN_ENC_A = 2;  // CHA only

// ---------- Encoder state ----------
volatile long encoderCount = 0;
volatile bool dirForward   = true;

const int   CPR           = 200;
const float DEG_PER_COUNT = 360.0 / CPR;

// ---------- Motion tuning ----------
const int   STOP_TOL_COUNTS = 2;  // ±3.6° tolerance
const unsigned long MOVE_TIMEOUT_MS = 20000;

// Single power level - we'll only vary pulse duration
const uint8_t MOVE_POWER = 30;

// Adaptive pulse timing (will learn and adjust)
int basePulseTime = 25;  // Start with 25ms pulses
const int minPulse = 15;
const int maxPulse = 50;

// =====================================================
//                      ENCODER ISR
// =====================================================
void encoderISR() {
  if (dirForward) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// =====================================================
//                    MOTOR HELPERS
// =====================================================
void motorBrake(bool on) {
  digitalWrite(PIN_BRAKE, on ? LOW : HIGH);
}

void motorSetDirection(bool forward) {
  dirForward = forward;
  digitalWrite(PIN_DIR, forward ? HIGH : LOW);
}

void motorSetPower(uint8_t power) {
  analogWrite(PIN_RUN, 255 - power);
}

void motorStop() {
  motorSetPower(0);
}

long getEncoderCount() {
  noInterrupts();
  long c = encoderCount;
  interrupts();
  return c;
}

// Execute a single movement pulse and return counts moved
int executePulse(int pulseTimeMs) {
  long startCount = getEncoderCount();
  
  motorSetPower(MOVE_POWER);
  delay(pulseTimeMs);
  
  motorStop();
  motorBrake(true);
  delay(100);  // Hard brake
  motorBrake(false);
  delay(150);  // Settle time
  
  long endCount = getEncoderCount();
  return abs(endCount - startCount);
}

// =====================================================
//                  MOVE TO TARGET ANGLE
// =====================================================
void moveToAngle(float targetDeg) {
  long targetCount = lround(targetDeg * CPR / 360.0);

  motorBrake(false);
  delay(150);

  long current = getEncoderCount();
  long err = targetCount - current;

  if (abs(err) <= STOP_TOL_COUNTS) {
    Serial.println("[Move] Already within tolerance.");
    motorStop();
    motorBrake(true);
    return;
  }

  Serial.print("\n[Move] Target = ");
  Serial.print(targetDeg, 1);
  Serial.print("° (");
  Serial.print(targetCount);
  Serial.print(" counts), Current = ");
  Serial.print(current * DEG_PER_COUNT, 1);
  Serial.print("° (");
  Serial.print(current);
  Serial.println(" counts)");

  bool needForward = (targetCount > current);
  motorSetDirection(needForward);
  delay(100);

  unsigned long moveStart = millis();
  int moveCount = 0;
  int overshootCount = 0;

  while (true) {
    long currentCount = getEncoderCount();
    long eCount = targetCount - currentCount;
    long absErr = abs(eCount);

    float currentDeg = currentCount * DEG_PER_COUNT;

    // Check if reached
    if (absErr <= STOP_TOL_COUNTS) {
      Serial.println("[Move] Target reached!");
      break;
    }

    // Check for overshoot
    bool shouldBeForward = (eCount > 0);
    if (shouldBeForward != needForward) {
      overshootCount++;
      if (overshootCount > 2) {
        Serial.println("[Move] Oscillating - stopping here");
        break;
      }
      
      Serial.println("[Overshoot - reversing]");
      
      // Reduce pulse time when we overshoot
      basePulseTime = max(minPulse, basePulseTime - 3);
      Serial.print("Reducing pulse time to ");
      Serial.println(basePulseTime);
      
      motorStop();
      motorBrake(true);
      delay(250);
      motorBrake(false);
      delay(100);
      
      needForward = shouldBeForward;
      motorSetDirection(needForward);
      delay(100);
    }

    // Calculate pulse duration based on distance
    int pulseTime;
    if (absErr <= 2) {
      pulseTime = minPulse;  // Minimum pulse for last count
    } else if (absErr <= 5) {
      pulseTime = basePulseTime;  // Adaptive base pulse
    } else if (absErr <= 15) {
      pulseTime = basePulseTime + 10;
    } else {
      pulseTime = basePulseTime + 20;
    }
    
    pulseTime = constrain(pulseTime, minPulse, maxPulse);

    Serial.print("Pos: ");
    Serial.print(currentDeg, 1);
    Serial.print("° | Err: ");
    Serial.print(absErr);
    Serial.print(" counts | Pulse: ");
    Serial.print(pulseTime);
    Serial.println("ms");

    // Execute the pulse
    int countsMoved = executePulse(pulseTime);
    moveCount++;

    // Adaptive learning - if we moved too far, reduce pulse time
    if (absErr <= 5 && countsMoved > absErr) {
      basePulseTime = max(minPulse, basePulseTime - 2);
      Serial.print("Moved too far - reducing base pulse to ");
      Serial.println(basePulseTime);
    }
    // If we barely moved, increase pulse time
    else if (absErr > 3 && countsMoved < 1) {
      basePulseTime = min(maxPulse, basePulseTime + 2);
      Serial.print("Moved too little - increasing base pulse to ");
      Serial.println(basePulseTime);
    }

    // Safety timeout
    if (millis() - moveStart > MOVE_TIMEOUT_MS) {
      Serial.println("[Move] TIMEOUT");
      break;
    }

    // Give up if too many moves without progress
    if (moveCount > 50) {
      Serial.println("[Move] Too many iterations - stopping");
      break;
    }
  }

  motorStop();
  motorBrake(true);
  delay(150);

  long finalCount = getEncoderCount();
  float finalDeg = finalCount * DEG_PER_COUNT;
  float finalError = finalDeg - targetDeg;

  Serial.print("[Final] Position = ");
  Serial.print(finalDeg, 1);
  Serial.print("° (");
  Serial.print(finalCount);
  Serial.print(" counts) | Error = ");
  Serial.print(finalError, 1);
  Serial.print("° | Learned pulse time: ");
  Serial.print(basePulseTime);
  Serial.println("ms");
  
  if (abs(finalError) <= 5.4) {
    Serial.println("Status: ACCEPTABLE");
  } else {
    Serial.println("Status: OUT OF SPEC");
  }
}

// =====================================================
//                      SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_DIR,   OUTPUT);
  pinMode(PIN_RUN,   OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);

  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_RUN, HIGH);
  digitalWrite(PIN_BRAKE, LOW);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);

  encoderCount = 0;

  Serial.println("=== Nidec 24H Adaptive Bang-Bang Control (8V) ===");
  Serial.println("CPR: 200, Resolution: 1.8°, Tolerance: ±3.6°");
  Serial.println("The system will learn optimal pulse duration");
  Serial.println("\nCommands:");
  Serial.println("  <angle> - Move to angle");
  Serial.println("  z - Zero position");
  Serial.println();
}

// =====================================================
//                       LOOP
// =====================================================
void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line.equalsIgnoreCase("z") || line.equalsIgnoreCase("zero")) {
      noInterrupts();
      encoderCount = 0;
      interrupts();
      Serial.println("[Zero] Position reset");
      return;
    }

    float angle = line.toFloat();
    if (!line.equals("0") && angle == 0.0) {
      Serial.println("[Error] Invalid angle");
      return;
    }

    moveToAngle(angle);
    Serial.println();
  }
}
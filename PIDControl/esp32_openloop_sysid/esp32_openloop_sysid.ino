/*
  ESP32 Open-Loop System Identification (with PID Safety Net)
  -------------------------------------------------------------
  Purpose : Drive the DC motor OPEN LOOP with a staircase PWM profile
            (max speed -> min speed) to collect step-response data for
            system identification in MATLAB.
  Safety  : The actuator has finite mechanical travel. If the measured
            position leaves a safe band near either end stop, this sketch
            temporarily takes control away from the open-loop profile and
            runs the closed-loop PID (saturation + anti-windup, same as
            esp32_pid_position_control.ino) to pull the position back to
            a safe region. Once recovered, the open-loop profile resumes.
  Logging : Every 10 ms sample prints one row: time_ms, pwm_cmd,
            position_deg, mode. Plain tab-separated numeric rows after a
            single header line, so the Serial Monitor output can be
            copy-pasted into a .txt file and loaded in MATLAB with
            readmatrix('log.txt') (it auto-skips the text header row).

  Wiring (same as esp32_pid_position_control.ino, PWM driven directly on
  the two direction pins - sign-magnitude drive, EN tied high in hardware):
    Potentiometer wiper -> GPIO35 (ADC1_CH7)
    L293D 1A (pin 2)    -> GPIO13
    L293D 2A (pin 7)    -> GPIO14
    L293D EN            -> tied to +5V (always enabled)

  Serial commands (type in Serial Monitor, Enter to send):
    r  -> (re)start the staircase profile from the first step
    x  -> stop immediately (motor off, profile halted)
*/

#include <Arduino.h>

// ---------- Pin definitions ----------
const int PIN_POT       = 35;  // ADC input (potentiometer wiper)
const int PIN_MOTOR_IN1 = 13;  // L293D direction pin 1A (PWM)
const int PIN_MOTOR_IN2 = 14;  // L293D direction pin 2A (PWM)

// ---------- PWM (LEDC) configuration ----------
const int PWM_FREQ_HZ    = 500;
const int PWM_RESOLUTION = 8;                         // 8-bit duty
const int PWM_MAX        = (1 << PWM_RESOLUTION) - 1; // 255

// ---------- Potentiometer / mechanical calibration ----------
const int   ADC_MIN       = 0;
const int   ADC_MAX       = 4095;
const float ANGLE_MIN_DEG = 0.0f;
const float ANGLE_MAX_DEG = 300.0f;

// ---------- Safety PID gains (used only near the travel limits) ----------
float Kp = 1.0f;
float Ki = 0.1f;
float Kd = 0.1f;
const float KAW = 1.0f;

// ---------- Safety band ----------
const float SAFETY_MARGIN_DEG = 15.0f;  // engage PID within this margin of an end stop
const float SAFETY_HYST_DEG   = 5.0f;   // must recover this far past the margin to hand back control
const float LOWER_LIMIT_DEG   = ANGLE_MIN_DEG + SAFETY_MARGIN_DEG;
const float UPPER_LIMIT_DEG   = ANGLE_MAX_DEG - SAFETY_MARGIN_DEG;

// ---------- Open-loop staircase profile (system ID input) ----------
// Sequential steps from maximum forward speed to maximum reverse speed.
const int stepValues[] = {
  PWM_MAX,               // 100% forward
  (PWM_MAX * 3) / 4,     //  75% forward
  PWM_MAX / 2,           //  50% forward
  PWM_MAX / 4,           //  25% forward
  0,                     //   0%
  -(PWM_MAX / 4),        //  25% reverse
  -(PWM_MAX / 2),        //  50% reverse
  -((PWM_MAX * 3) / 4),  //  75% reverse
  -PWM_MAX               // 100% reverse
};
const int NUM_STEPS = sizeof(stepValues) / sizeof(stepValues[0]);
const uint32_t STEP_DURATION_MS = 3000; // hold each step for 3 s, tune to your dynamics

// ---------- Control loop timing ----------
const float    Ts_S  = 0.01f;                    // 10 ms sample time (100 Hz)
const uint32_t Ts_MS = (uint32_t)(Ts_S * 1000.0f);

// ---------- Modes ----------
enum ControlMode { MODE_OPEN_LOOP = 0, MODE_SAFETY = 1, MODE_STOPPED = 2 };
ControlMode mode = MODE_OPEN_LOOP;

// ---------- Open-loop profile state ----------
int      currentStep      = 0;
uint32_t stepStartMillis  = 0;

// ---------- Safety PID state ----------
float integralTerm    = 0.0f;
float prevMeasurement = 0.0f;
bool  firstSample      = true;
float safetySetpoint  = 0.0f;

uint32_t lastSampleMillis = 0;
String   serialBuffer;

// ---------- Helpers ----------
float readPositionDeg() {
  int raw = analogRead(PIN_POT);
  float deg = (float)(raw - ADC_MIN) * (ANGLE_MAX_DEG - ANGLE_MIN_DEG)
              / (float)(ADC_MAX - ADC_MIN) + ANGLE_MIN_DEG;
  return deg;
}

void setMotor(float cmd) {
  int duty = (int)fabsf(cmd);
  duty = constrain(duty, 0, PWM_MAX);

  const float DEADBAND = 0.5f;
  if (cmd > DEADBAND) {
    ledcWrite(PIN_MOTOR_IN1, 0);
    ledcWrite(PIN_MOTOR_IN2, duty);
  } else if (cmd < -DEADBAND) {
    ledcWrite(PIN_MOTOR_IN1, duty);
    ledcWrite(PIN_MOTOR_IN2, 0);
  } else {
    ledcWrite(PIN_MOTOR_IN1, 0);
    ledcWrite(PIN_MOTOR_IN2, 0);
  }
}

void restartProfile(uint32_t now) {
  currentStep     = 0;
  stepStartMillis = now;
  mode            = MODE_OPEN_LOOP;
}

void handleSerialInput(uint32_t now) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        char cmd = serialBuffer.charAt(0);
        if (cmd == 'r' || cmd == 'R') {
          restartProfile(now);
        } else if (cmd == 'x' || cmd == 'X') {
          mode = MODE_STOPPED;
        }
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  ledcAttach(PIN_MOTOR_IN1, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttach(PIN_MOTOR_IN2, PWM_FREQ_HZ, PWM_RESOLUTION);
  setMotor(0);

  lastSampleMillis = millis();
  restartProfile(lastSampleMillis);

  Serial.println(F("# Open-loop staircase system ID with PID safety net"));
  Serial.println(F("# Commands: r = (re)start profile, x = stop"));
  Serial.println(F("# mode: 0=open-loop step, 1=safety PID, 2=stopped"));
  Serial.println(F("time_ms\tpwm_cmd\tposition_deg\tmode"));
}

void loop() {
  uint32_t now = millis();
  handleSerialInput(now);

  if (now - lastSampleMillis >= Ts_MS) {
    lastSampleMillis += Ts_MS;

    // ---- Measurement ----
    float measurement = readPositionDeg();
    if (firstSample) {
      prevMeasurement = measurement;
      firstSample = false;
    }

    bool belowLower = measurement <= LOWER_LIMIT_DEG;
    bool aboveUpper = measurement >= UPPER_LIMIT_DEG;

    // ---- Mode arbitration ----
    if (mode == MODE_OPEN_LOOP && (belowLower || aboveUpper)) {
      mode = MODE_SAFETY;
      safetySetpoint = belowLower ? (LOWER_LIMIT_DEG + SAFETY_HYST_DEG)
                                   : (UPPER_LIMIT_DEG - SAFETY_HYST_DEG);
      integralTerm = 0.0f; // fresh PID state entering safety mode
    } else if (mode == MODE_SAFETY) {
      bool recovered = (measurement > LOWER_LIMIT_DEG + SAFETY_HYST_DEG) &&
                        (measurement < UPPER_LIMIT_DEG - SAFETY_HYST_DEG);
      if (recovered) {
        mode = MODE_OPEN_LOOP;
        stepStartMillis = now; // resume the current step with a fresh timer
      }
    }

    float pwmCmd = 0.0f;

    if (mode == MODE_OPEN_LOOP) {
      // ---- Staircase system-ID profile ----
      if (currentStep < NUM_STEPS) {
        pwmCmd = (float)stepValues[currentStep];
        if (now - stepStartMillis >= STEP_DURATION_MS) {
          currentStep++;
          stepStartMillis = now;
        }
      } else {
        pwmCmd = 0.0f; // profile finished, hold at rest
      }
      prevMeasurement = measurement; // keep PID derivative state current for a clean handoff
    } else if (mode == MODE_SAFETY) {
      // ---- Closed-loop PID recovery (saturation + back-calculation anti-windup) ----
      float error = safetySetpoint - measurement;
      float pTerm = Kp * error;

      float dMeasurement = (measurement - prevMeasurement) / Ts_S;
      float dTerm = -Kd * dMeasurement;
      prevMeasurement = measurement;

      integralTerm += Ki * error * Ts_S;
      float outputUnsat = pTerm + integralTerm + dTerm;
      float outputSat = constrain(outputUnsat, -(float)PWM_MAX, (float)PWM_MAX);

      float satError = outputSat - outputUnsat;
      integralTerm += KAW * satError * Ts_S;

      pwmCmd = outputSat;
    } else { // MODE_STOPPED
      pwmCmd = 0.0f;
      prevMeasurement = measurement;
    }

    pwmCmd = constrain(pwmCmd, -(float)PWM_MAX, (float)PWM_MAX);
    setMotor(pwmCmd);

    // ---- Telemetry (pure numeric rows for MATLAB import) ----
    Serial.print(now);              Serial.print('\t');
    Serial.print(pwmCmd, 0);        Serial.print('\t');
    Serial.print(measurement, 2);   Serial.print('\t');
    Serial.println((int)mode);
  }
}

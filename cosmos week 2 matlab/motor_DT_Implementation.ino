/*
  ESP32 PID Position Control
  ---------------------------
  Plant   : DC motor driven through an L293D H-bridge
  Sensor  : Potentiometer coupled to the actuator shaft (analog position feedback)
  Control : Discrete PID with output saturation and back-calculation anti-windup
  Setpoint: Entered over Serial (type a number in degrees and press Enter)
  Telemetry: setpoint, measured position, error and PID output printed every sample

  Wiring
  ------
  Potentiometer wiper -> GPIO34 (ADC1_CH6, input-only pin)
  Potentiometer ends   -> 3V3 and GND

  L293D:
    EN (1,2EN, pin 1)  -> GPIO25 (PWM)
    1A (pin 2)         -> GPIO26 (direction)
    2A (pin 7)         -> GPIO27 (direction)
    1Y, 2Y (pins 3,6)  -> DC motor terminals
    Vcc1 (pin 16)      -> 5V logic supply (shared GND with ESP32)
    Vcc2 (pin 8)       -> motor supply voltage
    GND (pins 4,5,12,13) -> common ground with ESP32 and motor supply

  Notes
  -----
  - Written for Arduino-ESP32 core 2.x (ledcSetup/ledcAttachPin/ledcWrite).
    On core 3.x replace the three calls with:
      ledcAttach(PIN_MOTOR_EN, PWM_FREQ_HZ, PWM_RESOLUTION);
      ledcWrite(PIN_MOTOR_EN, duty);
  - Kp/Ki/Kd and KAW are starting points only; retune for your motor, gearbox
    and potentiometer range.
*/

#include <Arduino.h>

// ---------- Pin definitions ----------
const int PIN_POT       = 35;  // ADC input (potentiometer wiper)
const int PIN_MOTOR_IN1 = 13;  // L293D direction pin 1A
const int PIN_MOTOR_IN2 = 14;  // L293D direction pin 2A
const int PIN_MOTOR_EN  = 25;  // L293D enable / PWM pin

// ---------- PWM (LEDC) configuration ----------
const int PWM_CHANNEL    = 0;
const int PWM_FREQ_HZ    = 500;
const int PWM_RESOLUTION = 8;                         // 8-bit duty
const int PWM_MAX        = (1 << PWM_RESOLUTION) - 1; // 255
const int healthy=18;     //healthy system operation
const int fault=19;       //fault caused by lack of feedback or control effort


// ---------- Potentiometer / mechanical calibration ----------
// Adjust to match your potentiometer's electrical and mechanical range.
const int   ADC_MIN       = 0;      // raw ADC counts at one mechanical end stop
const int   ADC_MAX       = 4095;   // raw ADC counts at the other end stop
const float ANGLE_MIN_DEG = 0.0f;   // physical angle (deg) at ADC_MIN
const float ANGLE_MAX_DEG = 300.0f; // physical angle (deg) at ADC_MAX (typical 300 deg pot)

// ---------- PID tuning ----------
float Kp = 1.0f;
float Ki = 1.1f;
float Kd = 0.1f;
const float KAW = 1.0f;   // anti-windup back-calculation gain, tune alongside Ki

// ---------- Control loop timing ----------
const float    Ts_S  = 0.005f;                    // 10 ms sample time (100 Hz)
const uint32_t Ts_MS = (uint32_t)(Ts_S * 1000.0f);

// ---------- Control state ----------
float setpointDeg      = 0.0f;  // updated from Serial input
float integralTerm     = 0.0f;  // stores Ki * integral(error dt), already gain-scaled
float prevMeasurement  = 0.0f;
bool  firstSample       = true;

// ---------- DT parameters ---------
float y_k2=0;
float y_k1=0;
float SP_k2=0;
float SP_k1=0;

uint32_t lastSampleMillis = 0;
String   serialBuffer;

// ---------- Helpers ----------
float readPositionDeg() {
  int raw = analogRead(PIN_POT);
  float deg = (float)(raw - ADC_MIN) * (ANGLE_MAX_DEG - ANGLE_MIN_DEG)
              / (float)(ADC_MAX - ADC_MIN) + ANGLE_MIN_DEG;
  return deg;
}

void setMotor(float pidOutput) {
  // pidOutput is expected pre-saturated to [-PWM_MAX, +PWM_MAX]
  int duty = (int)fabsf(pidOutput);
  duty = constrain(duty, 0, PWM_MAX);

  const float DEADBAND = 0.5f; // avoid chattering direction pins near zero output
  if (pidOutput > DEADBAND) {
    ledcWrite(PIN_MOTOR_IN1, 0);
    ledcWrite(PIN_MOTOR_IN2, duty);
  } else if (pidOutput < -DEADBAND)
   {
    ledcWrite(PIN_MOTOR_IN1, duty);
    ledcWrite(PIN_MOTOR_IN2, 0);

  } else 
  {
     ledcWrite(PIN_MOTOR_IN1, 0);
    ledcWrite(PIN_MOTOR_IN2, 0);
  }
  //ledcWrite(PWM_CHANNEL, duty);
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        float val = serialBuffer.toFloat();
        setpointDeg = constrain(val, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
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
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);

  analogReadResolution(12);       // ESP32 ADC: 0-4095
  analogSetAttenuation(ADC_11db); // full 0-3.3V input range


  ledcAttach(PIN_MOTOR_IN1, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttach(PIN_MOTOR_IN2, PWM_FREQ_HZ, PWM_RESOLUTION);
  //ledcAttachPin(PIN_MOTOR_EN, PWM_CHANNEL);
  //ledcWrite(PWM_CHANNEL, 0);

  setpointDeg = readPositionDeg(); // hold current position until a setpoint arrives
  lastSampleMillis = millis();

  //Digital Twin variables initialization
  float y_k2=0;
  float y_k1=0;
  float SP_k2=0;
  float SP_k1=0;
    pinMode(healthy, OUTPUT);
  pinMode(fault, OUTPUT);

  Serial.println(F("# ESP32 PID position control ready"));
  Serial.println(F("# Send a setpoint in degrees over Serial, e.g. 120.0<Enter>"));
  Serial.println(F("t_ms,setpoint_deg,measured_deg,error_deg,pid_out"));
}

void loop() {
  handleSerialInput();

  uint32_t now = millis();
  if (now - lastSampleMillis >= Ts_MS) {
    lastSampleMillis += Ts_MS; // fixed-period scheduling, avoids long-term drift

    // ---- Measurement ----
    float measurement = readPositionDeg();
    if (firstSample) {
      prevMeasurement = measurement;
      firstSample = false;
    }

    // ---- Error ----
    float sp = setpointDeg;
    float error = sp - measurement;

    // ---- Proportional ----
    float pTerm = Kp * error;

    // ---- Derivative on measurement (avoids setpoint-change kick) ----
    float dMeasurement = (measurement - prevMeasurement) / Ts_S;
    float dTerm = -Kd * dMeasurement;
    prevMeasurement = measurement;

    // ---- Integral (tentative, pre-saturation) ----
    integralTerm += Ki * error * Ts_S;

    // ---- Unsaturated PID output ----
    float outputUnsat = pTerm + integralTerm + dTerm;

    // ---- Saturation ----
    float outputSat = constrain(outputUnsat, -(float)PWM_MAX, (float)PWM_MAX);

    // ---- Anti-windup: back-calculation ----
    // Pulls the integral back toward the saturated output whenever the
    // unsaturated command exceeds the actuator limits, so the integral
    // doesn't keep accumulating while the motor is already maxed out.
    float satError = outputSat - outputUnsat;
    integralTerm += KAW * satError * Ts_S;

    // ---- Drive motor ----
    setMotor(outputSat);

    // --- digital twin calculation ----
    // This DT represents the whole closed-loop control response of the system
    // using the closed-loop transfer function T identified as a second-order system
    // and translated into a difference equation.
    // input is the position setpoint
    // output is the motor position

    //float DTOutput= (0.01648* SP_k1 + 0.001574*SP_k2) + (1.867*y_k1- 0.8705*y_k2); 
    float DTOutput= (0.0008223*sp + 0.001648* SP_k1 + 0.0008223*SP_k2) + (1.867*y_k1- 0.8705*y_k2);  
    //variables updating
    //input variable (position setpoint)
    SP_k2=SP_k1;
    SP_k1=sp;
    //output variable (position)
    y_k2=y_k1;
    y_k1=DTOutput;
    
    //calculate DT normalized error
    float DT_error=abs(100*(measurement-DTOutput)/(sp+1e-3));

    //DT error threshold alarm
    
    if (DT_error>=10)
    {
    digitalWrite(fault,HIGH);
      digitalWrite(healthy,LOW);
    }
    else
    {
    digitalWrite(healthy,HIGH);
          digitalWrite(fault,LOW);
    }
    

    // ---- Telemetry ----
    Serial.print(now);            Serial.print(',');
    Serial.print(sp, 2);          Serial.print(',');
    Serial.print(measurement, 2); Serial.print(',');
    Serial.print(error, 2);       Serial.print(',');
    Serial.print(DTOutput, 2);       Serial.print(',');
    Serial.print(DT_error, 2);       Serial.print(',');
    Serial.println(outputSat, 1);
  }
}

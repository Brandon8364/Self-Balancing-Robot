// 1-DOF self-balancing robot: ESP32 + MPU6050 + TB6612FNG + 2x FIT0521 encoder motors
// 200 Hz fixed-rate control task, direct PWM output, selectable PID or LQR controller,
// WiFi access point + TCP command/telemetry link, ESP-NOW remote driving (speed + turn) on top of balancing,
// saved balance point (survives reboot), auto-arming at power-up, and tuning tools
// (live deadband / D-term filter commands and a 2 s, 200 Hz data recorder).
// Arduino-ESP32 core 2.x API (ledcSetup / ledcAttachPin), PlatformIO.
//
// Sign conventions (check these on the bench before balancing):
//   theta  > 0 : robot leaning toward the direction the wheels drive when PWM is positive
//   x, xdot > 0: robot moving in that same direction
//   yaw rate > 0: the turn you get when motor 1 runs faster than motor 2 (verify with the "w" command)

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_now.h>

// ============================== PINS ==============================
#define PWMA 25
#define AIN1 27
#define AIN2 26
#define ENC1_A 32
#define ENC1_B 33

#define PWMB 5                                   // change back to 4 if you kept the original wiring
#define BIN1 16
#define BIN2 17
#define ENC2_A 18
#define ENC2_B 19

#define STBY 14
#define SDA_PIN 21
#define SCL_PIN 22

// ============================== LOOP RATE ==============================
#define CTRL_HZ 200
const float CTRL_DT = 1.0f / CTRL_HZ;

// ============================== ROBOT CONSTANTS ==============================
const float WHEEL_R        = 0.030f;            // m
const float COUNTS_PER_REV = 341.2f * 4.0f;     // 4x decoding, output shaft
const float V_FULL         = 4.7f;              // effective motor volts at PWM 255 (MEASURE THIS)
const float RPM_FULL       = 165.0f;            // measured wheel RPM at PWM 255 on the same battery
const float KE_FF          = V_FULL / (RPM_FULL * 0.10472f);   // volts per rad/s of wheel speed (back-EMF)
float DB1 = 20.0f;                              // deadband compensation, motor 1 (PWM counts) - tune with: db1 <v>
float DB2 = 15.0f;                              // deadband compensation, motor 2 - tune with: db2 <v>
const bool  INVERT_M2 = true;

#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_FREQ 20000
#define PWM_RES  10                              // 0..1023
const float PWM_MAXCNT = 1023.0f;

// ============================== IMU SETTINGS ==============================
#define TILT_AXIS 0                              // 0: pitch is rotation about board X, 1: about board Y
#define YAW_AXIS  2                              // board axis that points up when balanced: 0=X 1=Y 2=Z
const float IMU_SIGN = 1.0f;                     // flip to -1.0f if angle sign is backwards
const float YAW_SIGN = 1.0f;                     // flip so "w 40" gives a positive yaw rate in the telemetry
const float COMP_TAU = 1.0f;                     // complementary filter time constant (s)

// ============================== SAFETY ==============================
const float CUTOFF_DEG   = 25.0f;                // motors stop beyond this tilt
const float YAW_CUTOFF   = 400.0f;               // deg/s, stops if the yaw loop runs away (wrong YAW_SIGN)
const float ARM_MAX_DEG  = 5.0f;                 // must be this close to balance point to arm
const float ARM_MAX_RATE = 20.0f;                // deg/s, must be nearly still to arm

// ============================== YAW LOOP ==============================
// Leave OFF until balancing works. Turn on after checking YAW_AXIS / YAW_SIGN with the "w" command.
const bool YAW_LOOP = false;

// ============================== AUTO-ARM (starts balancing at power-up) ==============================
// Arms by itself when the robot is held still at the balance point (the saved one, or the default
// angleZero if none has been saved). Hold it upright after power-up; send z once to save your own.
bool  autoArm      = true;
int   autoArmMode  = 1;                          // 1 = PID, 2 = LQR
const float AUTO_ARM_DEG  = 2.0f;                // within this many degrees of the balance point...
const float AUTO_ARM_RATE = 5.0f;                // ...and rotating slower than this (deg/s)...
const float AUTO_ARM_SEC  = 1.0f;                // ...for this long
Preferences prefs;
bool zeroSaved = false;

// ============================== REMOTE DRIVING LIMITS ==============================
const float V_MAX_CMD      = 0.20f;              // m/s   (top speed at full PWM is ~0.5 m/s; keep headroom to balance)
const float YAW_MAX_CMD    = 90.0f;              // deg/s
const float V_ACCEL        = 0.5f;               // m/s^2  reference ramp (a sudden step would saturate the motors)
const float YAW_ACCEL      = 360.0f;             // deg/s^2
const uint32_t CMD_TIMEOUT_MS = 500;             // no command for this long -> stop and hold position
const float STEER_MAX      = 60.0f;              // max PWM counts used for turning (balance has priority)
const float KY_P = 0.3f;                         // PWM per deg/s of yaw-rate error
const float KY_I = 1.0f;                         // PWM per degree of accumulated yaw error

// ============================== VELOCITY ESTIMATE ==============================
#define VEL_WIN 4                                // samples in the finite-difference window (20 ms @ 200 Hz)

// ============================== CONTROLLER GAINS (PLACEHOLDERS - RE-TUNE) ==============================
// PID mode: angle PD(+I) with an outer velocity/position loop that offsets the angle setpoint.
// Rule of thumb: kp*kv should be about 900-1000 PWM per (m/s) - it has to beat the motor back-EMF.
float kpA = 28.0f;     // PWM counts per degree
float kiA = 0.0f;      // PWM counts per degree-second
float kdA = 0.8f;      // PWM counts per deg/s
float kvA = 28.0f;     // degrees of lean per (m/s) of speed error
float kxA = 20.0f;     // degrees of lean per metre of position error (stops slow drift from a small balance-point error)
const float REF_MAX_DEG = 8.0f;
float rateAlpha = 1.0f;     // D-term low-pass: 1.0 = off, smaller = more filtering. Tune with: kf <0.05..1>
float rateF = 0;            // filtered tilt rate used by the D term (PID mode)

// Data recorder: 'rec' captures 2 s at the full control rate, then prints it as CSV
#define REC_N 400
float recTheta[REC_N], recRate[REC_N], recPwm[REC_N], recVel[REC_N];
volatile int  recIdx = 0;
volatile bool recActive = false, recDone = false, recStart = false;

// LQR mode: u = -K (z - zref) + feedforward, z = [x (m), xdot (m/s), theta (rad), thetadot (rad/s)], u in volts
float K[4] = {-1.6568f, -16.8626f, -17.0784f, -2.1430f};

// ============================== WIFI LINK ==============================
WiFiServer server(23);
WiFiClient client;

// Prints to USB serial and to the connected WiFi client
class Tee : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    Serial.write(b, n);
    if (client && client.connected()) client.write(b, n);
    return n;
  }
};
Tee Out;

// ============================== ENCODERS (4x decoding) ==============================
const int8_t QEM[16] = { 0, +1, -1,  0,
                        -1,  0,  0, +1,
                        +1,  0,  0, -1,
                         0, -1, +1,  0 };
volatile int32_t count1 = 0;
volatile int32_t count2 = 0;
volatile uint8_t last1 = 0, last2 = 0;

void IRAM_ATTR isr1() {
  uint8_t s = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
  count1 += QEM[(last1 << 2) | s];
  last1 = s;
}
void IRAM_ATTR isr2() {
  uint8_t s = (digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B);
  int8_t d = QEM[(last2 << 2) | s];
  count2 += INVERT_M2 ? -d : d;
  last2 = s;
}

// ============================== MOTORS ==============================
void writeMotor(int in1, int in2, int ch, float pwm) {
  int dir = (pwm > 0) - (pwm < 0);
  digitalWrite(in1, dir > 0);
  digitalWrite(in2, dir < 0);
  float mag = fminf(fabsf(pwm), 255.0f);
  ledcWrite(ch, (uint32_t)(mag * PWM_MAXCNT / 255.0f));
}
void setMotor1(float pwm) { writeMotor(AIN1, AIN2, PWM_CH1, pwm); }
void setMotor2(float pwm) { writeMotor(BIN1, BIN2, PWM_CH2, INVERT_M2 ? -pwm : pwm); }

// Smooth deadband compensation: skips the dead zone but ramps in near zero to avoid chatter
float withDeadband(float p, float db) {
  float a = fabsf(p);
  if (a < 0.5f) return 0.0f;
  float mag = db * fminf(a / 8.0f, 1.0f) + a * (255.0f - db) / 255.0f;
  mag = fminf(mag, 255.0f);
  return (p > 0) ? mag : -mag;
}

// ============================== IMU ==============================
uint8_t mpuAddr = 0x68;
float rAx, rAy, rAz, rGx, rGy, rGz;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  uint8_t addrs[2] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    mpuAddr = addrs[i];
    Wire.beginTransmission(mpuAddr);
    Wire.write(0x6B);
    Wire.write(0x00);                // wake up
    if (Wire.endTransmission() == 0) {
      delay(100);
      mpuWrite(0x19, 0x00);          // sample-rate divider 0 (1 kHz internal)
      mpuWrite(0x1A, 0x02);          // DLPF ~94 Hz (about 3 ms delay)
      mpuWrite(0x1B, 0x00);          // gyro +-250 dps (131 LSB/dps)
      mpuWrite(0x1C, 0x00);          // accel +-2 g (16384 LSB/g)
      return true;
    }
  }
  return false;
}

bool readMPU() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)mpuAddr, (uint8_t)14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  rAx = v[0] / 16384.0f; rAy = v[1] / 16384.0f; rAz = v[2] / 16384.0f;
  rGx = v[4] / 131.0f;   rGy = v[5] / 131.0f;   rGz = v[6] / 131.0f;
  return true;
}

float accelAngleDeg() {
#if TILT_AXIS == 0
  return atan2f(rAy, rAz) * 57.29578f;
#else
  return atan2f(-rAx, rAz) * 57.29578f;
#endif
}
float gyroRateDps() {
#if TILT_AXIS == 0
  return rGx;
#else
  return rGy;
#endif
}
float yawRateRawDps() {
#if YAW_AXIS == 0
  return rGx;
#elif YAW_AXIS == 1
  return rGy;
#else
  return rGz;
#endif
}

float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// ============================== SHARED STATE ==============================
enum Mode { IDLE = 0, PID_MODE = 1, LQR_MODE = 2 };
volatile int   mode = IDLE;
volatile float theta = 0, thetaRate = 0, xPos = 0, xVel = 0, pwmOut = 0;   // deg, deg/s, m, m/s, PWM
volatile float yawRate = 0, vRefOut = 0;
volatile float angleRaw = 0, angleZero = -2.40;
volatile int   armRequest = 0;        // 0 none, 1 PID, 2 LQR   (set by loop(), handled by control task)
volatile int   armReply = 0;          // 1 armed, 2 too far from balance, 3 moving too fast
volatile bool  cutoffFlag = false, faultFlag = false, calDone = false;
volatile int   calRequest = 0;        // samples remaining for gyro calibration
volatile float testPwm = 0, testSteer = 0;
volatile uint32_t testUntilMs = 0;
volatile float vTarget = 0, yawTarget = 0;      // remote drive commands
volatile uint32_t lastCmdMs = 0;
volatile uint32_t periodUsMax = 0, periodUsMin = 0xFFFFFFFF, loopUsMax = 0;
volatile bool  imuOk = false;

// ============================== ESP-NOW CONTROLLER LINK ==============================
typedef struct control_packet {      // must match the controller's struct exactly
  int16_t throttle;                  // -100..100
  int16_t steering;                  // -100..100
  bool buttonPressed;
} control_packet;

volatile bool lastButton = false;
volatile uint32_t rxCount = 0, rxLastMs = 0;     // link diagnostics (print with the "link" command)
volatile int16_t rxThr = 0, rxSte = 0;
volatile bool rxBtn = false;

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(control_packet)) return;
  control_packet p;
  memcpy(&p, data, sizeof(p));

  rxCount++; rxLastMs = millis();
  rxThr = p.throttle; rxSte = p.steering; rxBtn = p.buttonPressed;

  float t = constrain((int)p.throttle, -100, 100) / 100.0f;
  float s = constrain((int)p.steering, -100, 100) / 100.0f;
  vTarget   = t * V_MAX_CMD;         // flip the sign here if forward stick drives it backwards
  yawTarget = s * YAW_MAX_CMD;
  lastCmdMs = millis();              // feeds the existing 500 ms link-loss timeout

  if (p.buttonPressed && !lastButton) {   // button = emergency stop (rising edge)
    mode = IDLE; vTarget = 0; yawTarget = 0;
  }
  lastButton = p.buttonPressed;
}

// ============================== CONTROL STEP (runs at CTRL_HZ) ==============================
void controlStep() {
  static uint32_t lastUs = 0;
  static bool angleInit = false;
  static float gyroBias = 0, yawBias = 0, calSum = 0, calYawSum = 0;
  static int calCount = 0;
  static int imuFails = 0;
  static float phiBuf[VEL_WIN + 1];
  static int bufIdx = 0;
  static bool bufInit = false;
  static float xOffset = 0, pidInt = 0, yawInt = 0;
  static float vRef = 0, yawRef = 0, xRef = 0;
  static float stillTime = 0;

  uint32_t nowUs = micros();
  float dt = CTRL_DT;
  if (lastUs != 0) {
    uint32_t per = nowUs - lastUs;
    if (per > periodUsMax) periodUsMax = per;
    if (per < periodUsMin) periodUsMin = per;
    dt = constrain(per * 1e-6f, 0.5f * CTRL_DT, 2.0f * CTRL_DT);
  }
  lastUs = nowUs;

  // ---- 1. IMU: complementary filter ----
  if (imuOk) {
    if (readMPU()) {
      imuFails = 0;
      float rateRaw = gyroRateDps();
      float yawRaw = yawRateRawDps();
      float accS = IMU_SIGN * accelAngleDeg();
      if (!angleInit) { angleRaw = accS; angleInit = true; }

      if (calRequest > 0) {                      // gyro bias calibration (robot must be still)
        calSum += rateRaw; calYawSum += yawRaw; calCount++;
        if (--calRequest == 0) {
          gyroBias = calSum / calCount;
          yawBias = calYawSum / calCount;
          calSum = 0; calYawSum = 0; calCount = 0;
          calDone = true;
        }
      }

      float rate = IMU_SIGN * (rateRaw - gyroBias);
      float alpha = COMP_TAU / (COMP_TAU + dt);
      float a = angleRaw + rate * dt;
      a += (1.0f - alpha) * wrap180(accS - a);
      angleRaw = wrap180(a);
      thetaRate = rate;
      theta = wrap180(angleRaw - angleZero);
      yawRate = YAW_SIGN * (yawRaw - yawBias);
    } else if (++imuFails >= 3) {
      if (mode != IDLE) faultFlag = true;
      mode = IDLE;
    }
  }

  rateF += rateAlpha * (thetaRate - rateF);

  // ---- 2. wheel / robot states ----
  int32_t c1 = count1, c2 = count2;
  float phi = 0.5f * (float)(c1 + c2) / COUNTS_PER_REV * 6.2831853f;     // wheel angle rel. to body (rad)
  if (!bufInit) { for (int i = 0; i <= VEL_WIN; i++) phiBuf[i] = phi; bufInit = true; }
  phiBuf[bufIdx] = phi;
  bufIdx = (bufIdx + 1) % (VEL_WIN + 1);
  float phiDot = (phi - phiBuf[bufIdx]) / (VEL_WIN * CTRL_DT);

  float thRad  = theta * 0.0174533f;
  float thdRad = thetaRate * 0.0174533f;
  float xAbs   = WHEEL_R * (phi + thRad);        // ground position (encoder angle is relative to the body)
  float xdAbs  = WHEEL_R * (phiDot + thdRad);

  // ---- 2b. auto-arm: held still at the balance point ----
  if (autoArm && mode == IDLE && imuOk && angleInit && calRequest == 0 && armRequest == 0) {
    if (fabsf(theta) < AUTO_ARM_DEG && fabsf(thetaRate) < AUTO_ARM_RATE) stillTime += dt;
    else stillTime = 0;
    if (stillTime > AUTO_ARM_SEC) { armRequest = autoArmMode; stillTime = 0; }
  } else {
    stillTime = 0;
  }

  // ---- 3. arming ----
  if (armRequest) {
    int req = armRequest; armRequest = 0;
    if (!angleInit || !imuOk || fabsf(theta) > ARM_MAX_DEG) armReply = 2;
    else if (fabsf(thetaRate) > ARM_MAX_RATE) armReply = 3;
    else {
      pidInt = 0; yawInt = 0;
      vRef = 0; yawRef = 0; xRef = 0;
      xOffset = xAbs;
      mode = (req == 1) ? PID_MODE : LQR_MODE;
      armReply = 1;
    }
  }
  float x  = xAbs - xOffset;
  xPos = x; xVel = xdAbs;

  // ---- 4. safety cutoff ----
  if (mode != IDLE && (fabsf(theta) > CUTOFF_DEG || (YAW_LOOP && fabsf(yawRate) > YAW_CUTOFF))) {
    mode = IDLE;
    cutoffFlag = true;
  }

  // ---- 5. drive references (ramped, with link timeout) ----
  float vGoal = 0, yawGoal = 0;
  if (mode != IDLE && (uint32_t)(millis() - lastCmdMs) < CMD_TIMEOUT_MS) {
    vGoal = vTarget;
    yawGoal = yawTarget;
  }
  if (fabsf(pwmOut) > 0.85f * 255.0f) vGoal = 0;      // near saturation: stop asking for more speed
  vRef   += constrain(vGoal - vRef,     -V_ACCEL * dt,   V_ACCEL * dt);
  yawRef += constrain(yawGoal - yawRef, -YAW_ACCEL * dt, YAW_ACCEL * dt);
  if (mode == IDLE) { vRef = 0; yawRef = 0; }
  if (fabsf(vRef) > 1e-3f) xRef = x;                  // while driving, don't fight the position error;
                                                      // when it stops, the robot holds where it stopped
  vRefOut = vRef;
  float ffV = KE_FF * vRef / WHEEL_R;                 // volts needed to hold wheel speed vRef (back-EMF)

  // ---- 6. balance controller ----
  float pwmCmd = 0;
  if (mode == PID_MODE) {
    float thRef = -(kvA * (xdAbs - vRef) + kxA * (x - xRef));
    thRef = constrain(thRef, -REF_MAX_DEG, REF_MAX_DEG);
    float e = theta - thRef;
    pidInt += e * dt;
    if (kiA > 0.0f) pidInt = constrain(pidInt, -40.0f / kiA, 40.0f / kiA);
    pwmCmd = kpA * e + kiA * pidInt + kdA * rateF + ffV / V_FULL * 255.0f;
  } else if (mode == LQR_MODE) {
    float u = -(K[0] * (x - xRef) + K[1] * (xdAbs - vRef) + K[2] * thRad + K[3] * thdRad) + ffV;   // volts
    pwmCmd = u / V_FULL * 255.0f;
  }
  pwmCmd = constrain(pwmCmd, -255.0f, 255.0f);

  if (recStart) { recStart = false; recIdx = 0; recActive = true; }
  if (recActive) {
    recTheta[recIdx] = theta; recRate[recIdx] = thetaRate; recPwm[recIdx] = pwmCmd; recVel[recIdx] = xdAbs;
    if (++recIdx >= REC_N) { recActive = false; recDone = true; }
  }

  // ---- 7. yaw-rate loop (steering; also holds the robot straight when yawRef = 0) ----
  float steer = 0;
  if (mode != IDLE && YAW_LOOP) {
    float yawErr = yawRef - yawRate;
    float steerMax = fminf(STEER_MAX, 255.0f - fabsf(pwmCmd));   // balance has priority over turning
    yawInt += yawErr * dt;
    yawInt = constrain(yawInt, -STEER_MAX / KY_I, STEER_MAX / KY_I);
    steer = constrain(KY_P * yawErr + KY_I * yawInt, -steerMax, steerMax);
  } else {
    yawInt = 0;
  }

  // ---- 8. outputs ----
  if (mode != IDLE) {
    setMotor1(withDeadband(pwmCmd + steer, DB1));
    setMotor2(withDeadband(pwmCmd - steer, DB2));
    pwmOut = pwmCmd;
  } else {
    bool testing = millis() < testUntilMs;       // bench test: raw PWM for a short time
    float p  = testing ? testPwm : 0.0f;
    float st = testing ? testSteer : 0.0f;
    setMotor1(p + st);
    setMotor2(p - st);
    pwmOut = p;
    pidInt = 0;
  }

  uint32_t used = micros() - nowUs;
  if (used > loopUsMax) loopUsMax = used;
}

void controlTask(void *) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / CTRL_HZ);
  for (;;) {
    vTaskDelayUntil(&last, period);
    controlStep();
  }
}

// ============================== COMMANDS ==============================
void printGains() {
  Out.printf("PID: kp=%.2f ki=%.2f kd=%.2f kv=%.2f kx=%.2f | LQR K=[%.4f %.4f %.4f %.4f] | zero=%.2f deg | DB1=%.0f DB2=%.0f kf=%.2f\n",
             kpA, kiA, kdA, kvA, kxA, K[0], K[1], K[2], K[3], (float)angleZero, DB1, DB2, rateAlpha);
}

bool logOn = true;

void handleCommand(String s) {
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return;

  if (s.startsWith("d ")) {                      // remote drive: d <speed m/s> <yaw deg/s>
    float v = 0, w = 0;
    int n = sscanf(s.c_str(), "d %f %f", &v, &w);
    if (n >= 1) {
      vTarget = constrain(v, -V_MAX_CMD, V_MAX_CMD);
      yawTarget = (n >= 2) ? constrain(w, -YAW_MAX_CMD, YAW_MAX_CMD) : 0.0f;
      lastCmdMs = millis();
    }
  } else if (s == "pid" || s == "lqr") {
    armReply = 0;
    armRequest = (s == "pid") ? 1 : 2;
  } else if (s == "s") {
    mode = IDLE; testUntilMs = 0; vTarget = 0; yawTarget = 0;
    autoArm = false;
    Out.println("Stopped. Auto-arm is OFF (send: auto pid / auto lqr to re-enable).");
  } else if (s == "z") {
    if (mode != IDLE) { Out.println("Stop first (s)."); return; }
    angleZero = angleRaw;
    prefs.putFloat("zero", angleZero);
    zeroSaved = true;
    Out.printf("Balance point set and saved: raw angle %.2f deg is now 0.\n", (float)angleZero);
  } else if (s == "cal") {
    if (mode != IDLE) { Out.println("Stop first (s)."); return; }
    calDone = false;
    calRequest = 2 * CTRL_HZ;
    Out.println("Calibrating gyro for 2 s - keep the robot still...");
  } else if (s.startsWith("auto")) {
    String a = s.substring(4);
    a.trim();
    if (a == "off")      { autoArm = false; Out.println("Auto-arm OFF."); }
    else if (a == "lqr") { autoArm = true; autoArmMode = 2; Out.println("Auto-arm ON (LQR)."); }
    else                 { autoArm = true; autoArmMode = 1; Out.println("Auto-arm ON (PID)."); }
  } else if (s.startsWith("zt ")) {          // trim the balance point by <deg> (works while balancing)
    float d = constrain(s.substring(3).toFloat(), -10.0f, 10.0f);
    angleZero = angleZero + d;
    prefs.putFloat("zero", angleZero);
    Out.printf("Balance point trimmed by %.2f deg (zero now %.2f).\n", d, (float)angleZero);
  } else if (s.startsWith("zo ")) {                // nudge the balance point by <deg>; works while armed
    angleZero = angleZero + s.substring(3).toFloat();
    Out.printf("Balance point nudged to %.2f deg (send zs to save it).\n", (float)angleZero);
  } else if (s == "zs") {
    prefs.putFloat("zero", angleZero);
    zeroSaved = true;
    Out.println("Balance point saved.");
  } else if (s == "link") {                        // ESP-NOW remote diagnostics
    Out.printf("LINK: rx=%lu age=%lu ms thr=%d ste=%d btn=%d vTarget=%.3f yawTarget=%.1f\n",
               (unsigned long)rxCount, (unsigned long)(millis() - rxLastMs),
               (int)rxThr, (int)rxSte, (int)rxBtn, (float)vTarget, (float)yawTarget);
  } else if (s == "rec") {
    recStart = true;
    Out.println("Recording 2 s at 200 Hz (keep it balancing)...");
  } else if (s.startsWith("db1 ")) { DB1 = constrain(s.substring(4).toFloat(), 0.0f, 80.0f); printGains();
  } else if (s.startsWith("db2 ")) { DB2 = constrain(s.substring(4).toFloat(), 0.0f, 80.0f); printGains();
  } else if (s.startsWith("kf "))  { rateAlpha = constrain(s.substring(3).toFloat(), 0.05f, 1.0f); printGains();
  } else if (s == "p") {
    printGains();
  } else if (s.startsWith("log")) {
    logOn = s.substring(3).toInt() != 0;
  } else if (s.startsWith("m ")) {               // bench: both wheels forward for 2 s
    if (mode != IDLE) { Out.println("Stop first (s)."); return; }
    testSteer = 0;
    testPwm = s.substring(2).toFloat();
    testUntilMs = millis() + 2000;
  } else if (s.startsWith("w ")) {               // bench: spin in place for 2 s (motor 1 fwd, motor 2 rev)
    if (mode != IDLE) { Out.println("Stop first (s)."); return; }
    testPwm = 0;
    testSteer = s.substring(2).toFloat();
    testUntilMs = millis() + 2000;
  } else if (s.startsWith("kp ")) { kpA = s.substring(3).toFloat(); printGains(); }
  else if (s.startsWith("ki "))  { kiA = s.substring(3).toFloat(); printGains(); }
  else if (s.startsWith("kd "))  { kdA = s.substring(3).toFloat(); printGains(); }
  else if (s.startsWith("kv "))  { kvA = s.substring(3).toFloat(); printGains(); }
  else if (s.startsWith("kx "))  { kxA = s.substring(3).toFloat(); printGains(); }
  else if (s.startsWith("k ")) {
    float a, b, c, d;
    if (sscanf(s.c_str(), "k %f %f %f %f", &a, &b, &c, &d) == 4) {
      K[0] = a; K[1] = b; K[2] = c; K[3] = d;
      printGains();
    } else {
      Out.println("Usage: k <Kx> <Kxdot> <Ktheta> <Kthetadot>");
    }
  } else {
    Out.println("Unknown command.");
  }
}

void pumpInput(Stream &s, String &buf) {
  while (s.available()) {
    char c = s.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) { handleCommand(buf); buf = ""; }
    } else {
      buf += c;
    }
  }
}

// ============================== SETUP / LOOP ==============================
void setup() {
  Serial.begin(115200);
  delay(1500);   // time to put the robot down and hold it still for gyro calibration

  prefs.begin("bot", false);
  float savedZero = prefs.getFloat("zero", 999.0f);
  if (savedZero < 900.0f) {
    angleZero = savedZero;
    zeroSaved = true;
    Serial.printf("Saved balance point loaded: %.2f deg\n", savedZero);
  } else {
    Serial.printf("No balance point saved: using default %.2f deg. Hold the robot at balance and send z to save yours.\n",
                  (float)angleZero);
  }

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES); ledcAttachPin(PWMA, PWM_CH1);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES); ledcAttachPin(PWMB, PWM_CH2);

  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  last1 = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
  last2 = (digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), isr2, CHANGE);

  imuOk = mpuInit();
  if (imuOk) {
    Serial.println("IMU found. Keep the robot still - calibrating gyro (2 s)...");
    calRequest = 2 * CTRL_HZ;
  } else {
    Serial.println("IMU NOT found - check SDA/SCL wiring.");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP("balancebot", "balance123", 1);    // fixed channel 1 so the controller matches
  server.begin();
  server.setNoDelay(true);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);
    Serial.print("ESP-NOW ready. Robot MAC for the controller: ");
    Serial.println(WiFi.softAPmacAddress());
  } else {
    Serial.println("ESP-NOW init failed.");
  }

  // Control task: above the lwIP task, on core 1 (WiFi driver tasks run on core 0)
  xTaskCreatePinnedToCore(controlTask, "ctrl", 6144, NULL, 19, NULL, 1);

  Serial.println("Commands: pid | lqr (arm) | s (stop) | z (set + save balance point) | cal | auto pid/lqr/off | d <m/s> <deg/s> (drive)");
  Serial.println("          m <pwm> / w <pwm> (2 s bench tests) | kp/ki/kd/kv/kx <v> | k <Kx> <Kxd> <Kth> <Kthd> | p | log 0/1 | link");
  Serial.println("          tuning: db1 <v> | db2 <v> | kf <0.05-1> (D filter, 1 = off) | rec (record 2 s, then prints it)");
  Serial.println("          balance point: zo <deg> (nudge, live) | zs (save)");
  Serial.println("CSV: t_ms,mode,theta_deg,rate_dps,x_m,xdot_mps,vref_mps,pwm,yaw_dps,period_us_min,period_us_max,loop_us_max");
}

void loop() {
  if (!client || !client.connected()) {
    WiFiClient c = server.available();
    if (c) { client = c; client.setNoDelay(true); Out.println("Connected."); }
  }
  static String rxUsb, rxWifi;
  pumpInput(Serial, rxUsb);
  pumpInput(client, rxWifi);

  if (armReply) {
    int r = armReply; armReply = 0;
    if (r == 1)      Out.println("ARMED.");
    else if (r == 2) Out.println("Not armed: too far from the balance point (hold it upright, send z first).");
    else if (r == 3) Out.println("Not armed: robot is moving - hold it still.");
  }
  if (cutoffFlag) { cutoffFlag = false; Out.println("CUTOFF: tilt/yaw limit exceeded, motors stopped."); }
  if (faultFlag)  { faultFlag = false;  Out.println("FAULT: IMU read failed, motors stopped."); }
  if (calDone)    { calDone = false;    Out.println("Gyro calibrated."); }

  if (recDone) {
    recDone = false;
    Out.println("REC,i,theta_deg,rate_dps,pwm,xdot_mps");
    for (int i = 0; i < REC_N; i++) {
      Out.printf("REC,%d,%.2f,%.1f,%.0f,%.3f\n", i, recTheta[i], recRate[i], recPwm[i], recVel[i]);
    }
    Out.println("REC end");
  }

  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (logOn && now - lastPrint >= 40) {        // 25 Hz telemetry
    lastPrint = now;
    Out.printf("%lu,%d,%.2f,%.1f,%.3f,%.3f,%.3f,%.0f,%.1f,%lu,%lu,%lu\n",
               (unsigned long)now, (int)mode, (float)theta, (float)thetaRate,
               (float)xPos, (float)xVel, (float)vRefOut, (float)pwmOut, (float)yawRate,
               (unsigned long)periodUsMin, (unsigned long)periodUsMax, (unsigned long)loopUsMax);
    periodUsMin = 0xFFFFFFFF; periodUsMax = 0; loopUsMax = 0;
  }
  delay(2);
}
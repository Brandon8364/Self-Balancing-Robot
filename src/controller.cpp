/*
  Robot controller - ESP32-WROOM-32D (PlatformIO / Arduino, Arduino-ESP32 core 2.x)
  Sends throttle + steering to the balancing robot over ESP-NOW at 50 Hz.

  Left stick  (X -> GPIO32, Y -> GPIO33): UP/DOWN    = drive forward/back
  Right stick (X -> GPIO39, Y -> GPIO34): LEFT/RIGHT = steer
  Left stick SEL -> GPIO25 (to GND): emergency stop (robot drops to IDLE, re-arms when held upright again)
  Power the sticks from 3V3. All four axes are on ADC1, so they work with WiFi on.

  Serial monitor 115200 baud:
    - prints stick directions when they change (set PRINT_DIRECTIONS false to silence)
    - prints a TX status line once per second (ok = robot ACKed, fail = no ACK)
    - send 'r' to recalibrate the stick centres (leave sticks untouched)
*/

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Robot SOFT-AP MAC (printed by the robot at boot: "Robot MAC for the controller: ...")
uint8_t robotAddress[] = {0x58, 0x2A, 0xBD, 0x7D, 0x17, 0x69};

#define BUTTON_PIN 25                 // left stick SEL, button to GND, internal pull-up
#define ESPNOW_CHANNEL 1              // must match WiFi.softAP(..., channel) on the robot

// ---------------- Configuration ----------------
const int DEADZONE      = 700;        // ADC counts: direction printout only
const int CTRL_DEADZONE = 200;        // ADC counts: dead zone for the drive commands
const bool PRINT_DIRECTIONS = true;   // print stick directions on change

// Flip these (applied to the raw axes, before the rotation) if a stick direction is mirrored
const bool INVERT_X[2] = { false, false };
const bool INVERT_Y[2] = { false, false };

// Flip these if the robot drives / turns the wrong way (applied after the rotation).
// Change them HERE only, not also on the robot, or the two flips cancel out.
const bool INVERT_THROTTLE = false;
const bool INVERT_STEER    = false;

struct Stick {
  const char* name;
  uint8_t pinX, pinY;
  int centerX, centerY;
  const char* lastDir;
};

Stick sticks[2] = {
  //  name           X   Y
  { "LEFT STICK",  32, 33 },
  { "RIGHT STICK", 39, 34 }   // GPIO39 = VN pin
};

typedef struct control_packet {       // must match the robot's struct exactly
  int16_t throttle;                   // -100..100
  int16_t steering;                   // -100..100
  bool buttonPressed;
} control_packet;

control_packet pkt;
esp_now_peer_info_t peerInfo;
volatile uint32_t sentOk = 0, sentFail = 0;

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) sentOk++; else sentFail++;
}

// ---------------- Helpers ----------------
int readAvg(uint8_t pin, int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) { sum += analogRead(pin); delayMicroseconds(200); }
  return sum / n;
}

void calibrate() {
  Serial.println("Calibrating - keep both sticks centred...");
  delay(500);
  for (int i = 0; i < 2; i++) {
    sticks[i].centerX = readAvg(sticks[i].pinX, 64);
    sticks[i].centerY = readAvg(sticks[i].pinY, 64);
    sticks[i].lastDir = "";   // force a fresh print
  }
  Serial.println("Ready.\n");
}

const char* getDirection(const Stick &s, int idx, int x, int y) {
  int dx = x - s.centerX;
  int dy = y - s.centerY;
  if (INVERT_X[idx]) dx = -dx;
  if (INVERT_Y[idx]) dy = -dy;

  int xd = (dx >  DEADZONE) ? 1 : (dx < -DEADZONE) ? -1 : 0;
  int yd = (dy >  DEADZONE) ? 1 : (dy < -DEADZONE) ? -1 : 0;
  // Sticks are mounted rotated: raw UP -> real RIGHT, raw RIGHT -> real DOWN,
  // raw DOWN -> real LEFT, raw LEFT -> real UP.
  int tx =  yd;   // real horizontal: +1 = RIGHT, -1 = LEFT
  int ty = -xd;   // real vertical:   +1 = UP,    -1 = DOWN

  if (tx ==  0 && ty ==  0) return "CENTER";
  if (tx ==  0 && ty ==  1) return "UP";
  if (tx ==  1 && ty ==  1) return "UP-RIGHT";
  if (tx ==  1 && ty ==  0) return "RIGHT";
  if (tx ==  1 && ty == -1) return "DOWN-RIGHT";
  if (tx ==  0 && ty == -1) return "DOWN";
  if (tx == -1 && ty == -1) return "DOWN-LEFT";
  if (tx == -1 && ty ==  0) return "LEFT";
  return "UP-LEFT";
}

// Raw reading -> -100..100 with dead zone and full-range scaling
int scaleAxis(int raw, int center, bool invert) {
  int d = raw - center;
  if (abs(d) < CTRL_DEADZONE) return 0;
  float v = (d > 0) ? (d - CTRL_DEADZONE) / (float)(4095 - center - CTRL_DEADZONE)
                    : (d + CTRL_DEADZONE) / (float)(center - CTRL_DEADZONE);
  int out = constrain((int)(v * 100), -100, 100);
  return invert ? -out : out;
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  analogReadResolution(12);                              // 0-4095
  for (int i = 0; i < 2; i++) {
    analogSetPinAttenuation(sticks[i].pinX, ADC_11db);   // full 0-3.3V range
    analogSetPinAttenuation(sticks[i].pinY, ADC_11db);
  }
  calibrate();                                           // leave sticks centred at power-up

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("Controller MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Targeting robot: %02X:%02X:%02X:%02X:%02X:%02X\n",
                robotAddress[0], robotAddress[1], robotAddress[2],
                robotAddress[3], robotAddress[4], robotAddress[5]);

  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW Init Failed"); return; }
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;                                  // 0 = use the channel set above
  peerInfo.ifidx   = WIFI_IF_STA;
  peerInfo.encrypt = false;
  esp_err_t r = esp_now_add_peer(&peerInfo);
  Serial.printf("add_peer: %s\n", esp_err_to_name(r));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') calibrate();
  }

  int x[2], y[2];
  for (int i = 0; i < 2; i++) {
    x[i] = readAvg(sticks[i].pinX, 4);
    y[i] = readAvg(sticks[i].pinY, 4);

    if (PRINT_DIRECTIONS) {
      const char* dir = getDirection(sticks[i], i, x[i], y[i]);
      if (strcmp(dir, sticks[i].lastDir) != 0) {
        Serial.printf("%s: %s\n", sticks[i].name, dir);
        sticks[i].lastDir = dir;
      }
    }
  }

  // Left stick: real UP = negated raw X.  Right stick: real RIGHT = raw Y.
  int thr = scaleAxis(x[0], sticks[0].centerX, !INVERT_X[0]);
  int ste = scaleAxis(y[1], sticks[1].centerY,  INVERT_Y[1]);
  pkt.throttle = INVERT_THROTTLE ? -thr : thr;
  pkt.steering = INVERT_STEER    ? -ste : ste;
  pkt.buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  esp_err_t sr = esp_now_send(robotAddress, (uint8_t *)&pkt, sizeof(pkt));
  static uint32_t lastErr = 0;
  if (sr != ESP_OK && millis() - lastErr > 1000) {
    lastErr = millis();
    Serial.printf("send error: %s\n", esp_err_to_name(sr));
  }

  static uint32_t lastRep = 0;
  if (millis() - lastRep > 1000) {
    lastRep = millis();
    Serial.printf("TX ok=%lu fail=%lu | thr=%d ste=%d btn=%d\n",
                  (unsigned long)sentOk, (unsigned long)sentFail,
                  pkt.throttle, pkt.steering, pkt.buttonPressed);
  }

  delay(20);                                             // 50 Hz
}
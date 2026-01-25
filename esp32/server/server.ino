#include <WiFi.h>

// ---------- Wi-Fi AP ----------
const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";
WiFiServer server(80);

// ---------- STEP/DIR/EN PINS ----------
#define J1_STEP_PIN  25
#define J1_DIR_PIN   26
#define J1_EN_PIN    27

#define J2_STEP_PIN  14
#define J2_DIR_PIN   12
#define J2_EN_PIN    13

// active LOW enable like your STM32 code (A4988/TB6600 style) [web:27]
#define STEPS_PER_REV 1600   // same as STM32 define

String header;

// current joint angles
float current_angle_j1 = 0.0f;
float current_angle_j2 = 0.0f;

// ---------- HELPERS ----------
uint32_t angle_to_steps(float angle_deg, uint32_t steps_per_rev) {
  float steps_f = (angle_deg / 360.0f) * (float)steps_per_rev;
  if (steps_f < 0) steps_f = 0;
  return (uint32_t)(steps_f + 0.5f);
}

// enable/disable helpers (active LOW)
void enable_driver(uint8_t en_pin) {
  digitalWrite(en_pin, LOW);
}
void disable_driver(uint8_t en_pin) {
  digitalWrite(en_pin, HIGH);
}

// ---------- HTTP PARSER: /arm?j1=..&j2=.. ----------
void handleArmRequest(const String& path) {
  float target_j1 = current_angle_j1;
  float target_j2 = current_angle_j2;

  int qPos = path.indexOf('?');
  if (qPos >= 0) {
    String query = path.substring(qPos + 1);

    int j1Pos = query.indexOf("j1=");
    if (j1Pos >= 0) {
      int amp = query.indexOf('&', j1Pos);
      String v = (amp >= 0) ? query.substring(j1Pos + 3, amp)
                            : query.substring(j1Pos + 3);
      target_j1 = v.toFloat();
    }

    int j2Pos = query.indexOf("j2=");
    if (j2Pos >= 0) {
      int amp = query.indexOf('&', j2Pos);
      String v = (amp >= 0) ? query.substring(j2Pos + 3, amp)
                            : query.substring(j2Pos + 3);
      target_j2 = v.toFloat();
    }
  }

  // clamp like STM32 (0..180)
  if (target_j1 < 0.0f)   target_j1 = 0.0f;
  if (target_j1 > 180.0f) target_j1 = 180.0f;
  if (target_j2 < -180.0f)   target_j2 = -180.0f;
  if (target_j2 > 180.0f) target_j2 = 180.0f;

  // deltas
  float delta1 = target_j1 - current_angle_j1;
  float delta2 = target_j2 - current_angle_j2;

  // nothing to do?
  if (delta1 == 0.0f && delta2 == 0.0f) {
    return;
  }

  uint8_t dirHigh1 = (delta1 >= 0.0f) ? HIGH : LOW;
  uint8_t dirHigh2 = (delta2 >= 0.0f) ? HIGH : LOW;
  float move_angle1 = (delta1 >= 0.0f) ? delta1 : -delta1;
  float move_angle2 = (delta2 >= 0.0f) ? delta2 : -delta2;

  uint32_t steps1 = angle_to_steps(move_angle1, STEPS_PER_REV);
  uint32_t steps2 = angle_to_steps(move_angle2, STEPS_PER_REV);

  if (steps1 == 0 && steps2 == 0) {
    return;
  }

  // set directions once
  digitalWrite(J1_DIR_PIN, dirHigh1);
  digitalWrite(J2_DIR_PIN, dirHigh2);

  // enable both drivers
  enable_driver(J1_EN_PIN);
  enable_driver(J2_EN_PIN);

  // interleaved stepping: both motors move in parallel [web:50][web:41]
  uint32_t i1 = 0, i2 = 0;
  const uint32_t pulseHigh_ms = 2;   // your 2 ms high
  const uint32_t baseDelay_ms = 2;   // your low delay (speed)

  while (i1 < steps1 || i2 < steps2) {
    // rising edges
    if (i1 < steps1) digitalWrite(J1_STEP_PIN, HIGH);
    if (i2 < steps2) digitalWrite(J2_STEP_PIN, HIGH);
    delay(pulseHigh_ms);

    // falling edges
    if (i1 < steps1) digitalWrite(J1_STEP_PIN, LOW);
    if (i2 < steps2) digitalWrite(J2_STEP_PIN, LOW);
    delay(baseDelay_ms);

    if (i1 < steps1) i1++;
    if (i2 < steps2) i2++;
  }

  // disable both
  disable_driver(J1_EN_PIN);
  disable_driver(J2_EN_PIN);

  current_angle_j1 = target_j1;
  current_angle_j2 = target_j2;

  Serial.print("J1 now: ");
  Serial.print(current_angle_j1);
  Serial.print("  J2 now: ");
  Serial.println(current_angle_j2);
}

void setup() {
  Serial.begin(115200);

  pinMode(J1_STEP_PIN, OUTPUT);
  pinMode(J1_DIR_PIN,  OUTPUT);
  pinMode(J1_EN_PIN,   OUTPUT);
  pinMode(J2_STEP_PIN, OUTPUT);
  pinMode(J2_DIR_PIN,  OUTPUT);
  pinMode(J2_EN_PIN,   OUTPUT);

  // start disabled
  disable_driver(J1_EN_PIN);
  disable_driver(J2_EN_PIN);

  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    String currentLine = "";
    header = "";
    bool requestDone = false;

    while (client.connected() && !requestDone) {
      if (client.available()) {
        char c = client.read();
        header += c;

        if (c == '\n') {
          if (currentLine.length() == 0) {
            // parse once we hit blank line
            int getPos  = header.indexOf("GET ");
            int httpPos = header.indexOf(" HTTP/");
            if (getPos >= 0 && httpPos > getPos) {
              String path = header.substring(getPos + 4, httpPos);

              if (path.startsWith("/arm")) {
                handleArmRequest(path);
              }
            }

            // respond
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/plain");
            client.println("Connection: close");
            client.println();
            client.print("J1=");
            client.print(current_angle_j1);
            client.print(" J2=");
            client.println(current_angle_j2);
            client.println();

            requestDone = true;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }

    header = "";
    client.stop();
  }
}

#include <HardwareSerial.h>

HardwareSerial STMSerial(2);

#define STM_RX_PIN 16
#define STM_TX_PIN 17

void setup() {
  Serial.begin(115200);
  delay(1000);

  STMSerial.begin(115200, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);  // RX, TX

  Serial.println("ESP32 -> STM32 angle test");

  delay(2000);

  STMSerial.print("40 0\n");
  Serial.println("Sent: 40 0");
  delay(3000);

  STMSerial.print("10 0\n");
  Serial.println("Sent: 10 0");
  delay(3000);

  STMSerial.print("0 0\n");
  Serial.println("Sent: 0 0");
}

void loop() {
    STMSerial.print("180 111");
    delay(500);
}

#include <Arduino.h>

// User requested GPIO 1 for this test
#define IR_PIN 4

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting IR ON (Constant High)...");

  pinMode(IR_PIN, OUTPUT);
  digitalWrite(IR_PIN, HIGH);

  Serial.printf("GPIO %d set to HIGH.\n", IR_PIN);
}

void loop() {
  delay(1000);
  Serial.println("IR LED is ON");
}

#include <Arduino.h>

#define IR_PIN 4

// ===== Carrier PWM =====
const int pwmChannel = 0;
const int pwmFreq = 8000;    // 8 kHz carrier
const int pwmResolution = 8; // 0–255 duty

// ===== Envelope Modulation =====
const int envelopeHz = 40;   // 40 Hz envelope
const int framesPerCode = 8; // 8-bit ID code

// Example ID code sequence (change per dancer)
uint8_t code[framesPerCode] = {1, 0, 1, 1, 0, 1, 0, 0};

volatile int codeIndex = 0;

hw_timer_t *timer = NULL;

void IRAM_ATTR onEnvelopeTick() {
  // Set duty based on current code bit
  if (code[codeIndex]) {
    ledcWrite(pwmChannel, 180); // ON brightness
  } else {
    ledcWrite(pwmChannel, 0); // OFF
  }

  codeIndex++;
  if (codeIndex >= framesPerCode)
    codeIndex = 0;
}

void setup() {
  // Carrier PWM
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(IR_PIN, pwmChannel);
  ledcWrite(pwmChannel, 0);

  // Envelope timer
  timer = timerBegin(0, 80, true);
  // 80 MHz / 80 = 1 MHz tick

  int alarm = 1000000 / envelopeHz;
  timerAttachInterrupt(timer, &onEnvelopeTick, true);
  timerAlarmWrite(timer, alarm, true);
  timerAlarmEnable(timer);
}

void loop() {
  // Nothing needed here
}

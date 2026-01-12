#include <Arduino.h>
#include <FastLED.h>

// ===============================
// IR EMITTER MODE
// ===============================
#define IR_ALWAYS_ON true   // keep IR on for testing

#define IR_LED_PIN 4
#define PWM_CHANNEL 0
#define PWM_RES 8
#define PWM_FREQ 8000
#define PWM_DUTY 90

// ===============================
// MATRIX BRIGHTNESS APPROXIMATION
// ===============================
// These values approximate how bright the camera will see
// the Adafruit high-power IR emitter at ~10 ft.
#define MATRIX_PIN 14
#define NUM_LEDS 64

#define MATRIX_GLOBAL_BRIGHTNESS 80     // ~30% drive current
#define MATRIX_COLOR CRGB(180,180,180) // strong neutral white

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("IR + MATRIX BRIGHTNESS APPROXIMATION MODE");

  // ---- IR emitter ----
  if (IR_ALWAYS_ON) {
    Serial.println("IR MODE = ALWAYS_ON");
    pinMode(IR_LED_PIN, OUTPUT);
    digitalWrite(IR_LED_PIN, HIGH);
  } else {
    Serial.println("IR MODE = PULSED_8KHZ");
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(IR_LED_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, PWM_DUTY);
  }

  // ---- Matrix ----
FastLED.addLeds<WS2812B, MATRIX_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(MATRIX_GLOBAL_BRIGHTNESS);
// Clear all LEDs
for (int i = 0; i < NUM_LEDS; i++) {
  leds[i] = CRGB::Black;
}

// Light a 3x3 block in the center of 8x8 matrix
int center = 27; // approximate center index in 8x8 matrix

int indices[] = {
  center-9, center-8, center-7,
  center-1, center,   center+1,
  center+7, center+8, center+9
};

for (int i = 0; i < 9; i++) {
  leds[indices[i]] = CRGB(180,180,180);
}

FastLED.show();


  Serial.println("Matrix set to high brightness approximation");
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 1000) {
    t = millis();
    Serial.println("RUNNING");
  }
}

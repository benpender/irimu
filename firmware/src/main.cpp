#include <Arduino.h>
#include <FastLED.h>
#include <SensorLib.h>
#include <SensorQMI8658.hpp>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ----------------------------------------------------------------------------
// CONFIGURATION
// ----------------------------------------------------------------------------
const char *ssid = "WebPage";
const char *password = "fivezerofourpage";
const char *udpAddress = "10.0.0.221"; // Host PC IP
const int udpPort = 44444;

#define SDA_PIN 11
#define SCL_PIN 12
#define IMU_IRQ 21 // Verify if needed, often not for polling

#define LED_PIN 14
#define NUM_LEDS 64
#define BRIGHTNESS 80

// ----------------------------------------------------------------------------
// GLOBALS
// ----------------------------------------------------------------------------
WiFiUDP udp;
CRGB leds[NUM_LEDS];
SensorQMI8658 qmi;

struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t timestamp;
};

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(3000); // Wait for Serial
  Serial.println("Starting ESP32-S3 Matrix Firmware (WiFi Mode)...");

  // Setup FastLED
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // Turn on ALL LEDs white for maximum tracking visibility
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();

  // Setup WiFi
  // LOWER POWER TO PREVENTBROWNOUTS
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("Scanning for networks...");
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("No networks found");
  } else {
    Serial.printf("%d networks found:\n", n);
    for (int i = 0; i < n; ++i) {
      Serial.printf("  %d: %s (%d) %s\n", i + 1, WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i),
                    (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
    }
  }

  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);

  // Wait for connect
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) { // 20 seconds
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Sending to %s:%d\n", udpAddress, udpPort);

    // Blink Green to indicate success
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(500);
    leds[0] = CRGB::Black;
    FastLED.show();

  } else {
    Serial.print("\nWiFi Connection Failed! Status Code: ");
    Serial.println(WiFi.status());
    Serial.println("Running in offline mode.");

    // Blink Red to indicate failure
    for (int i = 0; i < 5; i++) {
      leds[0] = CRGB::Red;
      FastLED.show();
      delay(200);
      leds[0] = CRGB::Black;
      FastLED.show();
      delay(200);
    }
  }

  // Setup IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SCL_PIN, SDA_PIN)) {
    if (!qmi.begin(Wire, QMI8658_H_SLAVE_ADDRESS, SCL_PIN, SDA_PIN)) {
      Serial.println("Failed to find QMI8658!");
    } else {
      Serial.println("QMI8658 Found!");
    }
  } else {
    Serial.println("QMI8658 Found!");
  }

  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                          SensorQMI8658::ACC_ODR_250Hz,
                          SensorQMI8658::LPF_MODE_0);
  qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS,
                      SensorQMI8658::GYR_ODR_224_2Hz,
                      SensorQMI8658::LPF_MODE_0);
  qmi.enableGyroscope();
  qmi.enableAccelerometer();
}

// ----------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------
void loop() {
  if (qmi.getDataReady()) {
    float acc[3], gyr[3];
    if (qmi.getAccelerometer(acc[0], acc[1], acc[2]) &&
        qmi.getGyroscope(gyr[0], gyr[1], gyr[2])) {
      // Send UDP
      if (WiFi.status() == WL_CONNECTED) {
        ImuData data;
        data.ax = acc[0];
        data.ay = acc[1];
        data.az = acc[2];
        data.gx = gyr[0];
        data.gy = gyr[1];
        data.gz = gyr[2];
        data.timestamp = millis();
        udp.beginPacket(udpAddress, udpPort);
        udp.write((uint8_t *)&data, sizeof(data));
        udp.endPacket();
      }
    }
  }

  // Visual Heartbeat
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 2000) {
    lastBlink = millis();
    if (leds[0])
      leds[0] = CRGB::Black;
    else
      leds[0] = (WiFi.status() == WL_CONNECTED) ? CRGB::Blue : CRGB::Orange;
    FastLED.show();

    if (WiFi.status() == WL_CONNECTED) {
      // Serial.println("Ping: Alive and Connected");
    } else {
      Serial.println("Ping: Disconnected");
    }
  }
  delay(2);
}

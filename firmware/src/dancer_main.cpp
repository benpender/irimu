#include <Arduino.h>
#include <FastLED.h>
#include <SensorLib.h>
#include <SensorQMI8658.hpp>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ----------------------------------------------------------------------------
// CONFIGURATION
// ----------------------------------------------------------------------------
// NO DEVICE ID! (Generic Firmware)
#define SDA_PIN 11
#define SCL_PIN 12
#define IMU_IRQ 21

#define LED_PIN 14
#define NUM_LEDS 64
#define BRIGHTNESS 50 // Keep reasonable for battery

// ----------------------------------------------------------------------------
// GLOBALS
// ----------------------------------------------------------------------------
CRGB leds[NUM_LEDS];
SensorQMI8658 qmi;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t myMac[6]; // Store my own MAC

struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t timestamp;
};

// Packet Types
#define PKT_IMU 0x01
#define PKT_HEARTBEAT 0xHB

// ----------------------------------------------------------------------------
// ESP-NOW CALLBACKS
// ----------------------------------------------------------------------------

// RECEIVE LED DATA (From Bridge TX)
// Format: [TargetMAC (6)] [RGB Data (192)]
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // 1. MAC Addressed Packet (198 bytes)
  if (len == 198) {
    if (memcmp(incomingData, myMac, 6) == 0) {
      // SUCCESS: Green
      fill_solid(leds, NUM_LEDS, CRGB::Green);
      FastLED.show();
    } else {
      // WRONG MAC: Blue
      fill_solid(leds, NUM_LEDS, CRGB::Blue);
      FastLED.show();
    }
  }
  // 2. Legacy Broadcast (192 bytes)
  else if (len == 192) {
    // BROADCAST: Magenta
    fill_solid(leds, NUM_LEDS, CRGB::Magenta);
    FastLED.show();
  } else {
    // WRONG LENGTH: Yellow
    fill_solid(leds, NUM_LEDS, CRGB::Yellow);
    FastLED.show();
  }
}

// SENT STATUS (From IMU Send)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print(status == ESP_NOW_SEND_SUCCESS ? "." : "X");
}

// ----------------------------------------------------------------------------
// HEARTBEAT
// ----------------------------------------------------------------------------
void sendHeartbeat() {
  uint8_t hbPacket[8];
  hbPacket[0] = 0xFA; // Magic Heartbeat (Using 0xFA to match bridge known types
                      // if needed, or stick to 0xHB)
  // Actually, let's use a distinct Header for Heartbeat so Bridge can wrap it
  // easily. Bridge wraps everything... so it doesn't matter. Let's use 0xHB
  // (HeartBeat) -> 0x48 0x42
  hbPacket[0] = 'H';
  hbPacket[1] = 'B';
  memcpy(hbPacket + 2, myMac, 6);

  esp_now_send(broadcastAddress, hbPacket, 8);
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Fast Boot
  Serial.println("BOOT_GENERIC_DANCER");

  // Get MAC
  WiFi.mode(WIFI_STA);
  esp_wifi_get_mac(WIFI_IF_STA, myMac);
  Serial.printf("MY_MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", myMac[0], myMac[1],
                myMac[2], myMac[3], myMac[4], myMac[5]);

  // Setup FastLED
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // 1. START [BLUE]
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(250);

  // FORCE CHANNEL 1
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
    return;
  }

  // 2. ESP-NOW OK [CYAN]
  fill_solid(leds, NUM_LEDS, CRGB::Cyan);
  FastLED.show();
  delay(100);

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // Register Peer (Broadcast)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); // <--- CRITICAL FIX: Zero init!
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; // Use Current Channel
  peerInfo.encrypt = false;

  // Debug Channel
  Serial.print("WiFi Channel: ");
  Serial.println(WiFi.channel());

  esp_err_t addStatus = esp_now_add_peer(&peerInfo);
  if (addStatus != ESP_OK) {
    Serial.printf("Failed to add peer: %s\n", esp_err_to_name(addStatus));
    fill_solid(leds, NUM_LEDS, CRGB::Red); // Fail Red if peer bad
    FastLED.show();
    return;
  }

  // 3. READY [RED]
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(100);

  // Setup IMU
  Wire.begin(SDA_PIN, SCL_PIN);

  // 4. IMU INIT [YELLOW]
  fill_solid(leds, NUM_LEDS, CRGB::Yellow);
  FastLED.show();

  Serial.println("Starting QMI8658 Begin...");
  bool qmiFound = false;
  // Correct order for SensorLib: (Wire, address, sda, scl)
  // 0x6A and 0x6B are the standard QMI8658 addresses
  if (qmi.begin(Wire, 0x6B, SDA_PIN, SCL_PIN)) {
    qmiFound = true;
  } else if (qmi.begin(Wire, 0x6A, SDA_PIN, SCL_PIN)) {
    qmiFound = true;
  }
  Serial.println("DANCER_READY");

  if (!qmiFound) {
    Serial.println("Failed to find QMI8658!");
    // Pulse Red/Yellow to warn, but continue (so LEDs still work)
    for (int i = 0; i < 5; i++) {
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      FastLED.show();
      delay(100);
      fill_solid(leds, NUM_LEDS, CRGB::Yellow);
      FastLED.show();
      delay(100);
    }
  }

  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                          SensorQMI8658::ACC_ODR_250Hz,
                          SensorQMI8658::LPF_MODE_0);
  qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS,
                      SensorQMI8658::GYR_ODR_224_2Hz,
                      SensorQMI8658::LPF_MODE_0);
  qmi.enableGyroscope();
  qmi.enableAccelerometer();

  // 5. READY [GREEN]
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  leds[0] = CRGB::Green;
  FastLED.show();
}

// ----------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------
void loop() {
  // 1. Read IMU
  // Force read attempts (Bypass getDataReady which seems broken/stuck)
  float acc[3], gyr[3];
  bool success = qmi.getAccelerometer(acc[0], acc[1], acc[2]);
  bool successG = qmi.getGyroscope(gyr[0], gyr[1], gyr[2]);

  static uint32_t lastReadLog = 0;
  bool timeToLog = (millis() - lastReadLog > 500);

  // Heartbeat Logic
  static uint32_t lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 1000) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }

  if (success && successG) {
    ImuData data;
    data.ax = acc[0];
    data.ay = acc[1];
    data.az = acc[2];
    data.gx = gyr[0];
    data.gy = gyr[1];
    data.gz = gyr[2];
    data.timestamp = millis();

    esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data));

    if (timeToLog) {
      lastReadLog = millis();
      // Serial.print(".R");
      Serial.printf("AX:%.2f AY:%.2f AZ:%.2f\n", data.ax, data.ay, data.az);
    }
  } else {
    if (timeToLog) {
      lastReadLog = millis();
      Serial.print("F");
    }
    // Re-init logic could go here
  }

  // 2. Refresh Display (Rate Limited to ~60Hz)
  static uint32_t lastFrame = 0;
  if (millis() - lastFrame > 16) {
    lastFrame = millis();
    // Logic removed to allow Python to control display
    FastLED.show();
  }

  // DEBUG: Heartbeat
  static uint32_t lastLog = 0;
  if (millis() - lastLog > 1000) {
    lastLog = millis();
    Serial.print("L"); // Heatbeat
    if (!qmi.getDataReady())
      Serial.print("?"); // IMU Not Ready
  }

  // 3. Keep loop fast
  delay(1);
}

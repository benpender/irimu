#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ----------------------------------------------------------------------------
// BRIDGE FIRMWARE
// ----------------------------------------------------------------------------
// Roles:
// 0 = UNITIALIZED
// 1 = RX_MODE (Receiver): Listens to ESP-NOW, prints to Serial
// 2 = TX_MODE (Sender): Reads Serial, blasts to ESP-NOW Broadcast

#define ROLE_IDLE 0
#define ROLE_RX 1
#define ROLE_TX 2

int currentRole = ROLE_RX;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ----------------------------------------------------------------------------
// SERIAL PROTOCOL
// ----------------------------------------------------------------------------
// We use a simple packet format to talk to Python:
// CMD_SET_RX:  "MODE:RX\n"
// CMD_SET_TX:  "MODE:TX\n"
// DATA_OUT:    "DATA:<binary_bytes>\n" (Only for TX mode)

// Buffer for incoming Serial data
uint8_t serialBuf[512];
int serialIdx = 0;

// ----------------------------------------------------------------------------
// ESP-NOW CALLBACKS
// ----------------------------------------------------------------------------

// ON DATA RECV (Bridge acting as RX)
// Called when we hear a packet from a Dancer
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // DEBUG: Prove radio works
  // Serial.print(".");

  if (currentRole != ROLE_RX) {
    // Print once per second to avoid flooding if IDLE
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 1000) {
      lastLog = millis();
      Serial.printf("DEBUG: SAW PACKET len=%d (Role=%d)\n", len, currentRole);
    }
    return;
  }

  // Send to Mac via Serial
  // Format: <MAC bytes> <Data bytes>
  // We add a simple header/footer for Python to find frames
  Serial.write(0xAA);              // Header
  Serial.write(0xBB);              // Header
  Serial.write(len);               // Length
  Serial.write(incomingData, len); // Payload
  Serial.write(0xCC);              // Footer
  Serial.write(0xDD);              // Footer
}

// ON DATA SENT (Bridge acting as TX)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: Could report ACKs to Serial, but for high-speed rain demo
  // we assume fire-and-forget to save bus bandwidth.
}

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); // Standard Baud Rate
  delay(2000);          // Give time for USB to enumerate
  Serial.println("BOOT_START");

  // Init WiFi in Station Mode
  WiFi.mode(WIFI_STA);
  Serial.println("WIFI_STA_OK");

  // FORCE CHANNEL 1
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.println("CHANNEL_1_OK");

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP_NOW_OK");

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register Broadcast Peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); // <--- Fix: Zero init
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1; // Force Channel 1
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("BRIDGE_READY");
}

// ----------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------
void loop() {
  // Process Serial Commands from Python
  while (Serial.available()) {
    uint8_t c = Serial.read();

    // Simple State Machine for "Binary Mode" vs "Command Mode"
    // For this demo, we will check generic strings first for config

    if (serialIdx < 511) {
      serialBuf[serialIdx++] = c;
    }

    // Check for Newline (End of Command)
    if (c == '\n') {
      serialBuf[serialIdx] = 0; // Null terminate
      String cmd = String((char *)serialBuf);
      cmd.trim();

      if (cmd == "MODE:RX") {
        currentRole = ROLE_RX;
        Serial.println("OK:ROLE_RX");
      } else if (cmd == "MODE:TX") {
        currentRole = ROLE_TX;
        Serial.println("OK:ROLE_TX");
      }
      // If we are in TX mode, and see a data packet header...
      // Note: For robust binary, we usually use cobs or fixed-headers.
      // For this quick demo, we will look for a magic byte in the stream
      // instead of string parsing for the High Speed LED data.

      serialIdx = 0; // Reset buffer
    }

    // SPECIAL BINARY HANDLING FOR LED DATA (TX MODE ONLY)
    // If we see 0xFA (Start Frame) and we are TX, assume next 192 bytes are
    // LEDs.
    if (currentRole == ROLE_TX && c == 0xFA) {
      // Blocking read for payload (fast & dirty for demo)
      // Payload: 192 bytes (64 leds * 3)
      uint8_t ledData[192];
      int readStart = 0;
      while (readStart < 192) {
        if (Serial.available()) {
          ledData[readStart++] = Serial.read();
        }
      }
      // Broadcast it!
      esp_now_send(broadcastAddress, ledData, 192);
      serialIdx = 0; // Reset parser
    }
  }
}

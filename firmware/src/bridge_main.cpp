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
// ----------------------------------------------------------------------------
// ESP-NOW CALLBACKS
// ----------------------------------------------------------------------------
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Only forward if we are RX
  if (currentRole == ROLE_RX) {
    // Transparent Forwarding:
    // Header: 0xAA 0xBB
    // Length: 1 byte
    // Payload: [Data...]
    // Footer: 0xCC 0xDD

    // We forward EVERYTHING. Host decides what it is (IMU, Heartbeat, etc).
    uint8_t buffer[260];
    int idx = 0;
    buffer[idx++] = 0xAA;
    buffer[idx++] = 0xBB;
    buffer[idx++] = (uint8_t)len;

    memcpy(buffer + idx, incomingData, len);
    idx += len;

    buffer[idx++] = 0xCC;
    buffer[idx++] = 0xDD;

    Serial.write(buffer, idx);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: Could report ACKs
}

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  // Increase RX Buffer for high-speed streaming
  // We receive ~600 bytes per frame (3 matrices). Default 256 bytes overflows
  // instantly.
  Serial.setRxBufferSize(4096);

  Serial.begin(921600); // Standard Baud Rate
  delay(1000);          // Give time for USB to enumerate (Reduced to 1s)
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
// ----------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------
void loop() {
  if (Serial.available()) {

    // ------------------------------------------------------------------------
    // TX MODE WRAPPER: 0xAA 0xBB [Type] [Payload]
    // ------------------------------------------------------------------------
    if (currentRole == ROLE_TX) {
      // Peek to see if possibly header.
      // Actually simpler: State machine or blocking read if AA seen?
      // Let's use Peek.
      if (Serial.peek() == 0xAA) {
        if (Serial.available() >= 3) { // Need [AA] [BB] [Type]
          Serial.read();               // Consume AA
          if (Serial.read() == 0xBB) {
            uint8_t type = Serial.read(); // Capture Type

            if (type == 0xFC) {
              // Addressed: [MAC 6] [Data 192] = 198 bytes
              uint8_t packetData[198];
              size_t r = Serial.readBytes(packetData, 198);
              if (r == 198)
                esp_now_send(broadcastAddress, packetData, 198);
            } else if (type == 0xFA) {
              // Legacy: [Data 192]
              uint8_t packetData[192];
              size_t r = Serial.readBytes(packetData, 192);
              if (r == 192)
                esp_now_send(broadcastAddress, packetData, 192);
            }
          }
        }
        return; // Don't check text commands if we saw AA
      }
    }

    // ------------------------------------------------------------------------
    // TEXT COMMANDS (Fallback / Control)
    // ------------------------------------------------------------------------
    // Process only if not AA header (or if AA was garbage/partial)
    // NOTE: Serial.readStringUntil blocks until timeout if newline not found.
    // Be careful.
    // Ideally we'd buffer properly. For now, assuming human typing or simple
    // script.

    // Check if it looks like ASCII command (M for MODE, R for ROLE, P for PING)
    uint8_t c = Serial.peek();
    if (c >= 'A' && c <= 'Z') {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      if (cmd == "MODE:RX" || cmd == "ROLE_RX") {
        currentRole = ROLE_RX;
        Serial.println("OK:ROLE_RX");
      } else if (cmd == "MODE:TX" || cmd == "ROLE_TX") {
        currentRole = ROLE_TX;
        Serial.println("OK:ROLE_TX");
      } else if (cmd == "PING") {
        Serial.println("PONG");
      }
    } else {
      // Consumer garbage to prevent stuck loop
      Serial.read();
    }
  }
}

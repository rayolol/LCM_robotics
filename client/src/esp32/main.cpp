#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <time.h>
#include "Motor.hpp"
#include <SPI.h>
#define SS 5


const char* ssid = "TP-LINK_A3B2AE";
const char* password = "tplink2017";
const int   UDP_PORT = 12345;

WiFiUDP udp;
uint8_t   buf[64];

TaskHandle_t updateMotorTaskHandle;
TaskHandle_t ParsePacketTaskHandle;

// Convert big-endian 64-bit to host byte order
int64_t be64toh_signed(uint8_t* bytes) {
    uint64_t result =
        ((uint64_t)bytes[0] << 56) |
        ((uint64_t)bytes[1] << 48) |
        ((uint64_t)bytes[2] << 40) |
        ((uint64_t)bytes[3] << 32) |
        ((uint64_t)bytes[4] << 24) |
        ((uint64_t)bytes[5] << 16) |
        ((uint64_t)bytes[6] << 8)  |
        ((uint64_t)bytes[7]);
    return (int64_t)result;
}
void ParsePacketTask(void* arg) {
  Serial.println("ParsePacketTask started");

  // Ensure WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, ParsePacketTask will wait");
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
    Serial.println("WiFi connected, ParsePacketTask continuing");
  }

  while (true) {
    int len = udp.parsePacket();
    if (len >= 56) {
      Serial.printf("Received UDP packet of length %d\n", len);

      // Read the packet data
      if (udp.read(buf, 56) != 56) {
        Serial.println("Error reading UDP packet");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      
      // Process timestamp
      int64_t ts_be = be64toh_signed(buf);
      Serial.printf("Big-endian timestamp: %lld\n", ts_be);
      
      // Use ts_be directly (or adjust if needed)
      int64_t ts = ts_be;
      
      // Process timestamp to get seconds and milliseconds
      time_t seconds = ts / 1000;
      int milliseconds = ts % 1000;
      
      // Convert to local time (+1 hour for summer time adjustment)
      seconds += ((-5 + 1) * 3600);
      
      if (milliseconds < 0) {
        seconds--;
        milliseconds += 1000;
      }
      
      struct tm timeinfo;
      gmtime_r(&seconds, &timeinfo);
      
      char timeStr[30];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
      Serial.printf("Timestamp: %s.%03d\n", timeStr, milliseconds);
      
      // Extract angles and speeds
      float angles[6], speeds[6];
      size_t offset = sizeof(ts_be);
      
      if (offset + 6 * sizeof(float) <= (size_t)len) {
        memcpy(angles, buf + offset, 6 * sizeof(float));
        offset += 6 * sizeof(float);
        
        if (offset + 6 * sizeof(float) <= (size_t)len) {
          memcpy(speeds, buf + offset, 6 * sizeof(float));
          
          // Print received angles and speeds
          Serial.println("Received angles and speeds:");
          for (int i = 0; i < 6; i++) {
            Serial.printf("Motor %d: angle=%.2f, speed=%.2f\n",
                          i+1, angles[i], speeds[i]);
          }
          
          // Set target angles for each motor
          Serial.println("Setting target angles");
          sendSPI(speeds, angles);
          Serial.println("Done setting target angles");
        }
        else {
          Serial.println("Packet too short for speeds data");
        }
      }
      else {
        Serial.println("Packet too short for angles data");
      }
    }
    // Prevent task from hogging CPU
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
size_t sendSPI(float speed[], float angle[]) {
  size_t len = 0;
  uint8_t buffer[64];
  buf[len++] = 0xAA; // Start byte
  buf[len++] = 6 * sizeof(float) + 6 * sizeof(float); // Length byte
  buf[len++] = 0x01; // Command byte

  memcpy(buffer + len, &angle, 6* sizeof(float));
  len += sizeof(float) * 6;
  memcpy(buffer + len, &speed, 6* sizeof(float));
  len += sizeof(float) * 6;

  size_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum += buffer[i];
  }
  buffer[len++] = checksum; // Checksum byte

  digitalWrite(SS, LOW);
  for (size_t i = 0; i < len; i++) {
    SPI.transfer(buffer[i]);
  }
  digitalWrite(SS, HIGH);
  Serial.println("SPI data sent");
  return len;
}


void setup() {
  Serial.begin(115200);
  SPI.begin();
  delay(1000); // Give serial time to initialize
  Serial.println("\n\nESP32 Starting up...");

  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH); // Set SS pin high

  // Initialize WiFi with timeout
  Serial.printf("Connecting to WiFi network: %s\n", ssid);
  WiFi.begin(ssid, password);
  
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttemptTime < 30000) {
    Serial.print(".");
    delay(500);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected successfully!");
    Serial.println("IP address: " + WiFi.localIP().toString());
    udp.begin(UDP_PORT);
    Serial.printf("UDP listening on port %d\n", UDP_PORT);
  }
  else {
    Serial.println("\nWiFi connection failed! Continuing without network...");
  }

  // Create packet parsing task on core 1
  Serial.println("Creating packet parsing task...");
  xTaskCreatePinnedToCore(
    ParsePacketTask,     // Task function
    "ParsePacket",       // Name of task
    4096,               // Stack size in bytes
    NULL,                // Task input parameter
    1,                   // Priority
    &ParsePacketTaskHandle, // Task handle
    0                    // Core 1
  );
  
  Serial.println("Setup complete!");
}

void loop() {
}







#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <time.h>
#include "Motor.hpp"

//MOTOR 1 ROLL AXIS                     REFENCE ARDUINO PIN
#define STEPPIN_1 33 // BLACK <-YELLOW <- D5
#define DIRPIN_1 32 //GREEN <- YELLOW <- D2

//MOTOR 2 DIFFERENTIAL RIGHT
#define STEPPIN_2 26 // BLACK <-ORANGE <- D6
#define DIRPIN_2 25 // GREEN <- ORANGE <- D3

//MOTOR 3 DIFFERENTIAL LEFT
#define STEPPIN_3 14 // BLACK <-GREEN <- D7
#define DIRPIN_3 11 // GREEN <- GREEN <- D4

//MOTOR 4 ELBOW
#define STEPPIN_4 34 // BLUE
#define DIRPIN_4 35 //GREEN


//MOTOR 5 SHOULDER
#define STEPPIN_5 18 // BLUE
#define DIRPIN_5 19 // GREEN


//MOTOR 6 BASE
#define STEPPIN_6 21
#define DIRPIN_6 23


// #define ENPIN_1 22
// #define ENPIN_2 18
// #define ENPIN_3 17
// #define ENPIN_4 16
// #define ENPIN_5 15
// #define ENPIN_6 14

#define stepRev 1600

const char* ssid = "TP-LINK_A3B2AE";
const char* password = "tplink2017";
const int   UDP_PORT = 12345;

//TODO: make the TCA9548 I2C MUltiplexer address configurable
Motor motors[6] = {
    Motor(STEPPIN_1, DIRPIN_1, 1),
    Motor(STEPPIN_2, DIRPIN_2, 2),
    Motor(STEPPIN_3, DIRPIN_3, 3),
    Motor(STEPPIN_4, DIRPIN_4, 4),
    Motor(STEPPIN_5, DIRPIN_5, 5),
    Motor(STEPPIN_6, DIRPIN_6, 6)
};

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
void updateMotorTask(void* arg) {
  while(true) {
    for (auto &motor : motors) {
      motor.update();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void ParsePacketTask(void* arg) {
  Serial.println("ParsePacketTask started");

  // Only run this task if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, ParsePacketTask will wait");
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
    Serial.println("WiFi connected, ParsePacketTask continuing");
  }

  while(true) {
    try {
      int len = udp.parsePacket();
      if (len >= 56) {
        Serial.printf("Received UDP packet of length %d\n", len);

        // Read the packet data
        udp.read(buf, 56);

        // Process timestamp
        int64_t ts_be = be64toh_signed(buf);
        Serial.printf("Big-endian timestamp: %lld\n", ts_be);

        // Use the one that gives a reasonable result
        int64_t ts = ts_be;  // or ts_le if that gives better results

        // Process timestamp
        time_t seconds = ts / 1000;
        int milliseconds = ts % 1000;

        // Convert to local time + 1 for summer time
        seconds += ((-5 + 1) * 3600);

        // Ensure milliseconds is positive
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

        // Safely copy data with bounds checking
        if (offset + 6*sizeof(float) <= len) {
          memcpy(angles, buf + offset, 6*sizeof(float));
          offset += 6*sizeof(float);

          if (offset + 6*sizeof(float) <= len) {
            memcpy(speeds, buf + offset, 6*sizeof(float));

            // Print received angles and speeds
            Serial.println("Received angles and speeds:");
            for (int i = 0; i < 6; i++) {
              Serial.printf("Motor %d: angle=%.2f, speed=%.2f\n",
                           i+1, angles[i], speeds[i]);
            }

            // Set target angles for each motor
            Serial.println("Setting target angles");
            for (int i = 0; i < 6; i++) {
              // Skip motors with invalid pins
              if (motors[i].stepPin != 0 && motors[i].dirPin != 0) {
                motors[i].setTargetAngle(angles[i]);
              }
            }
            Serial.println("Done setting target angles");
          } else {
            Serial.println("Packet too short for speeds data");
          }
        } else {
          Serial.println("Packet too short for angles data");
        }
      }

      // Small delay to prevent task from hogging CPU
      vTaskDelay(pdMS_TO_TICKS(5));

    } catch (...) {
      Serial.println("Error in ParsePacketTask");
      // Continue running even if there's an error
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  Serial.println("\n\nESP32 Starting up...");

  // Initialize WiFi with timeout
  Serial.printf("Connecting to WiFi network: %s\n", ssid);
  WiFi.begin(ssid, password);

  // Add a timeout for WiFi connection (30 seconds)
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
  } else {
    Serial.println("\nWiFi connection failed! Continuing without network...");
  }

  // Initialize motors with error checking
  Serial.println("Initializing motors...");
  for (int i = 0; i < 6; i++) {
    Serial.printf("Initializing Motor %d (pins: step=%d, dir=%d)...\n",
                 i+1, motors[i].stepPin, motors[i].dirPin);
    try {
      motors[i].begin();
      Serial.printf("Motor %s initialized successfully\n", motors[i].name.c_str());
    } catch (...) {
      Serial.printf("Error initializing Motor %d\n", i+1);
    }
  }

  // Create motor update task on core 0 with higher priority
  Serial.println("Creating motor update task...");
  xTaskCreatePinnedToCore(
    updateMotorTask,   // Function to implement the task
    "UpdateMotor",     // Name of the task
    10000,             // Stack size in bytes
    NULL,              // Task input parameter
    2,                 // Priority of the task (higher)
    &updateMotorTaskHandle, // Task handle
    0                  // Run on core 0 (leave core 1 for WiFi/BT)
  );

  // Create packet parsing task on core 1
  Serial.println("Creating packet parsing task...");
  xTaskCreatePinnedToCore(
    ParsePacketTask,   // Function to implement the task
    "ParsePacket",     // Name of the task
    10000,             // Stack size in bytes
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &ParsePacketTaskHandle, // Task handle
    1                  // Run on core 1
  );

  Serial.println("Setup complete!");
}


void loop() {
  // Add a heartbeat message to show the ESP32 is still running
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) { // Every 5 seconds
    lastHeartbeat = millis();
    Serial.println("ESP32 heartbeat - system running");

    // Print WiFi status
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected: " + WiFi.localIP().toString());
    } else {
      Serial.println("WiFi disconnected");
    }
  }

  // Small delay to prevent watchdog issues
  delay(100);
}







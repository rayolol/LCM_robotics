#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <time.h>
#include "Motor.hpp"
#include <SPI.h>
#define SS 5

//MOTOR 1 ROLL AXIS                     REFENCE ARDUINO PIN
#define STEPPIN_1 33 // BLACK <-YELLOW <- D5
#define DIRPIN_1 32 //GREEN <- YELLOW <- D2

//MOTOR 2 DIFFERENTIAL RIGHT
#define STEPPIN_2 26 // BLACK <-ORANGE <- D6
#define DIRPIN_2 25 // GREEN <- ORANGE <- D3

//MOTOR 3 DIFFERENTIAL LEFT
#define STEPPIN_3 14 // BLACK <-GREEN <- D7
#define DIRPIN_3 12 // GREEN <- GREEN <- D4

//MOTOR 4 ELBOW
#define STEPPIN_4 34 // BLUE
#define DIRPIN_4 35 //GREEN

//MOTOR 5 SHOULDER
#define STEPPIN_5 18 // BLUE
#define DIRPIN_5 19 // GREEN

//MOTOR 6 BASE
#define STEPPIN_6 21
#define DIRPIN_6 23

#define ENPIN 22

#define stepRev 1600

const char* ssid = "TP-LINK_A3B2AE";
const char* password = "tplink2017";
const int   UDP_PORT = 12345;

//TODO: make the TCA9548 I2C Multiplexer address configurable
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
  while (true) {
    for (auto &motor : motors) {
      motor.update();
      // Yield in between updates to let other tasks run
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
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
          for (int i = 0; i < 6; i++) {
            if (motors[i].stepPin != 0 && motors[i].dirPin != 0) {
              motors[i].setTargetAngle(angles[i]);
            }
          }
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
void sendSPI(float speed[], float angle[]) {
  size_t len = 0;
  uint8_t buf[64];
  buf[len++] = 0xAA; // Start byte
  buf[len++] = sizeof(speed) + sizeof(angle); // Length byte
  buf[len++] = 0x01; // Command byte

  memcpy(buf + len, &angle, sizeof(speed));
  len += sizeof(angle);
  memcpy(buf + len, &speed, sizeof(angle));
  len += sizeof(speed);

  size_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum += buf[i];
  }
  buf[len++] = checksum; // Checksum byte

  digitalWrite(SS, LOW);
  for (size_t i = 0; i < len; i++) {
    SPI.transfer(buf[i]);
  }
  digitalWrite(SS, HIGH);
  Serial.println("SPI data sent");
}


void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  Serial.println("\n\nESP32 Starting up...");
  pinMode(ENPIN, OUTPUT);
  digitalWrite(ENPIN, HIGH); // Enable motor driver

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
  
  // Initialize motors with error checking
  Serial.println("Initializing motors...");
  for (int i = 0; i < 6; i++) {
    Serial.printf("Initializing Motor %d (pins: step=%d, dir=%d)...\n",
                  i+1, motors[i].stepPin, motors[i].dirPin);
    motors[i].begin();
    Serial.printf("Motor %s initialized successfully\n", motors[i].name.c_str());
  }
  
  // Create motor update task on core 0 with higher priority
  Serial.println("Creating motor update task...");
  xTaskCreatePinnedToCore(
    updateMotorTask,     // Task function
    "UpdateMotor",       // Name of task
    4096,               // Stack size in bytes
    NULL,                // Task input parameter
    2,                   // Priority
    &updateMotorTaskHandle, // Task handle
    1                    // Core 0
  );
  
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







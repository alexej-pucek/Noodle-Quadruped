#include <Arduino.h>
#include <HardwareSerial.h>
#include "lidar.h"

HardwareSerial LIDAR(2);


const int RX_PIN   = 17;   
const int TX_DUMMY = 16;   
const int MOT_EN   = 15;   

const int   PACKET_LEN  = 22;
const float ANGLE_STEP  = 1.0f;

void sendLidarStart() {
  uint8_t startCmd[] = { 0xA5, 0x60 };
  LIDAR.write(startCmd, sizeof(startCmd));
  Serial.println("# Sent start-scan command (A5 60)");
}

void sendLidarStop() {
  uint8_t stopCmd[] = { 0xA5, 0x65 };
  LIDAR.write(stopCmd, sizeof(stopCmd));
  Serial.println("# Sent stop-scan command (A5 65)");
}

void lidarsetup() {
  pinMode(MOT_EN, OUTPUT);

  
  analogWriteResolution(MOT_EN, 8);     
  analogWriteFrequency(MOT_EN, 1000);   
  analogWrite(MOT_EN, 0);               

  Serial.begin(115200);
  delay(300);
  Serial.println("# LDS02RR parser start");

  LIDAR.begin(115200, SERIAL_8N1, RX_PIN, TX_DUMMY);

  while (LIDAR.available()) LIDAR.read();
  delay(100);
}

void lidarloop() {
  static uint8_t buf[PACKET_LEN];
  static int pos = 0;
  static bool scanning = false;

  
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "START") {
      analogWrite(MOT_EN, 115);   
      delay(200);
      sendLidarStart();
      scanning = true;
      Serial.println("# START command received");

    } else if (cmd == "STOP") {
      sendLidarStop();
      analogWrite(MOT_EN, 0);     
      scanning = false;
      Serial.println("# STOP command received");

    } else if (cmd.startsWith("SPEED")) {
      int value = cmd.substring(5).toInt();   
      value = constrain(value, 0, 255);
      analogWrite(MOT_EN, value);
      Serial.printf("# Motor speed set to %d\n", value);
    }
  }

  if (!scanning) return;

  
  static String batchBuffer = "";
  static unsigned long lastSend = 0;
  const unsigned long sendInterval = 360;  
  const int maxBatchSize = 2048;           

  while (LIDAR.available()) {
    uint8_t b = LIDAR.read();

    if (pos == 0 && b != 0xFA) continue;

    buf[pos++] = b;

    if (pos == PACKET_LEN) {
      pos = 0;

      uint8_t index = buf[1];
      float base_angle = (index - 0xA0) * 4.0f;
      if (base_angle < 0) base_angle = 0;

      for (int i = 0; i < 4; i++) {
        int offset = 4 + i * 4;
        uint16_t dist = buf[offset] | (buf[offset + 1] << 8);
        float angle = base_angle + (i * 1.0f);

        if (angle >= 360.0f) angle -= 360.0f;
        if (dist == 0 || dist > 6000) continue;

        char line[32];
        snprintf(line, sizeof(line), "%.1f,%d\n", angle, dist);
        batchBuffer += line;
      }
    }
  }

  if (batchBuffer.length() > 0 &&
      (millis() - lastSend > sendInterval || batchBuffer.length() > maxBatchSize)) {
    Serial.print(batchBuffer);
    batchBuffer = "";
    lastSend = millis();
  }
}
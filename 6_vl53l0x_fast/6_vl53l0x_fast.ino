#include <Wire.h>
#include "VL53L0X.h"

VL53L0X lox[6];

int16_t lox_mosfets[6] = { 22, 23, 24, A13, A14, A15 };

int16_t lox_adresses[6] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36 };

int16_t lox_shts[6] = { 31, 32, 33, 30, 35, 34 };

// VL53L0X lox[5];

// int16_t lox_mosfets[5] = { 22, 24, A13, A14, A15 };

// int16_t lox_adresses[5] = { 0x31, 0x33, 0x34, 0x35, 0x36 };

// int16_t lox_shts[5] = { 31, 33, 30, 35, 34 };

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Serial.println("aaa");
  setID();
}

void loop() {
  // put your main code here, to run repeatedly:
  read_lasers();
}

void setID() {
  for (int i = 0; i < sizeof(lox_shts) / sizeof(lox_shts[0]); i++) {
    // if (i != 1) {
      pinMode(lox_mosfets[i], OUTPUT);
      digitalWrite(lox_mosfets[i], LOW);
      delay(10);
    // }
  }

  Serial.println("ccc");

  for (int i = 0; i < sizeof(lox_shts) / sizeof(lox_shts[0]); i++) {
    // if (i != 1) {
      pinMode(lox_shts[i], OUTPUT);
      // all reset
      digitalWrite(lox_shts[i], LOW);
      delay(10);
    // }
  }

  Serial.println("ddd");

  for (int i = 0; i < sizeof(lox_shts) / sizeof(lox_shts[0]); i++) {
    // initing LOXs
    // if (i != 1) {
      Serial.println(i);
      delay(10);
      digitalWrite(lox_shts[i], HIGH);
      delay(10);
      Serial.println(i);
      lox[i].setAddress(lox_adresses[i]);
      Serial.println(i);
      if (!lox[i].init()) {
        Serial.println("Failed to boot " + String(i) + " VL53L0X");
      } else {
        lox[i].startContinuous();
      }
      Serial.println(i);
    // }

    delay(10);
  }
  Serial.println("bbb");
}

uint32_t debugTimer;

void read_lasers() {
  debugTimer = micros();
  for (int i = 0; i < sizeof(lox_shts) / sizeof(lox_shts[0]); i++) {
    // if (i != 1) {
      Serial.print(lox[i].readRangeContinuousMillimeters());
      Serial.print("\t\t");
    // }
  }
  debugTimer = micros() - debugTimer;
  Serial.println(debugTimer);
}

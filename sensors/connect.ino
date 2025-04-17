void setID() {
  for (int i = 0; i < sizeof(lox_shts) / sizeof(lox_shts[0]); i++) {
    pinMode(lox_mosfets[i], OUTPUT);
    digitalWrite(lox_mosfets[i], LOW);
    delay(10);
  }

  Serial.println("ccc");

  for (int i = 0; i < sizeof(lox_shts) / sizeof(lox_shts[0]); i++) {
    pinMode(lox_shts[i], OUTPUT);
    // all reset
    digitalWrite(lox_shts[i], LOW);
    delay(10);
  }

  Serial.println("ddd");

  for (int i = 0; i < sizeof(lox_shts) / sizeof(lox_shts[0]); i++) {
    // initing LOXs
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

    delay(10);
  }
}

void readLasers() {
  static int32_t debugTimer;
  debugTimer = micros();
  for (int i = 0; i < sizeof(lox_shts) / sizeof(lox_shts[0]); i++) {
    lasers[i] = kFilterLaser * (lox[i].readRangeContinuousMillimeters()) + lasers[i] * (1 - kFilterLaser);
  }
  debugTimer = micros() - debugTimer;
  // Serial.println(debugTimer);
}

bool readMySerial() {
  static byte crc;
  if (errSerial) {
    Serial3.read();
    errSerial = 0;
  }
  Serial3.readBytes((byte *)&buf, sizeof(buf));
  crc = crc8_bytes((byte *)&buf, sizeof(buf));
  if (crc == 0) {
    for (int i = 0; i < 2; i++) {
      fromSerial.yr[i] = buf.yr[i];
      gyro[i] = fromSerial.yr[i] - zeroGyro[i];
    }
    for (int i = 0; i < 2; i++) {
      fromSerial.enc[i] = buf.enc[i];
      oldEncs[i] = encoders[i];
      encoders[i] = fromSerial.enc[i];
    }
    horizont += sin(gyro[1]) * (encoders[0] + encoders[1] - oldEncs[0] - oldEncs[1]) / 2;
    return 1;
  }
  errSerial = 1;
  return 0;
}

void writeSerial() {
  toSerial.crc = crc8_bytes((byte *)&toSerial, sizeof(toSerial) - 1);
  Serial3.write((byte *)&toSerial, sizeof(toSerial));
}

byte crc8_bytes(byte *buffer, byte size) {
  byte crc = 0;
  for (byte i = 0; i < size; i++) {
    byte data = buffer[i];
    for (int j = 8; j > 0; j--) {
      crc = ((crc ^ data) & 1) ? (crc >> 1) ^ 0x8C : (crc >> 1);
      data >>= 1;
    }
  }
  return crc;
}
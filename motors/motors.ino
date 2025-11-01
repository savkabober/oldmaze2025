#include "MotorsLib.h"

float kMotor = 2;

float kiMotor = 0.03;

int32_t *motorEncs[4];

uint8_t motorPins[] = { 8, 9, 10, 11, 5, 4, 7, 6 };

const uint8_t interrupts[] = { 1, 0, 5, 4 };

const uint8_t directionPins[] = { A1, A0, A3, A2 };

const uint8_t reverses[] = { 1, 1, 0, 0 };

const int16_t deltaNMax = 3;

int16_t deltaN = 0;

void interr0() {
  *motorEncs[0] = *motorEncs[0] - (reverses[0] * 2 - 1) * (digitalRead(directionPins[0]) * 2 - 1);
}

void interr1() {
  *motorEncs[1] = *motorEncs[1] - (reverses[1] * 2 - 1) * (digitalRead(directionPins[1]) * 2 - 1);
}

void interr2() {
  *motorEncs[2] = *motorEncs[2] - (reverses[2] * 2 - 1) * (digitalRead(directionPins[2]) * 2 - 1);
}

void interr3() {
  *motorEncs[3] = *motorEncs[3] - (reverses[3] * 2 - 1) * (digitalRead(directionPins[3]) * 2 - 1);
}

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <EEPROM.h>


MPU6050 mpu;

struct DataToEEPROM {
  int XAccelOffset;
  int YAccelOffset;
  int ZAccelOffset;
  int XGyroOffset;
  int YGyroOffset;
  int ZGyroOffset;
};

struct DataToSerial {
  float yr[2];
  int32_t enc[2];
  byte crc;
};

struct DataFromSerial {
  int32_t encMotor[2];
};

struct Buffer {
  int32_t encMotor[2];
  byte crc;
};

Buffer buf;

bool errSerial = 0;
uint8_t fifoBuffer[64];  // буфер
float ypr[3], myYpr[3], oldYpr[3];
uint32_t myTimer;
const uint32_t Ts = 5;
float corr = 0;
uint32_t oldTime;
float kYR = 1.05;

int32_t dMax = 20;

Quaternion q;
VectorFloat gravity;

DataToEEPROM dataEEPROM;
DataToSerial toSerial;
DataFromSerial fromSerial;

MOTORS motor[4];

void setup() {
  myTimer = millis() + 2000;
  // put your setup code here, to run once:
  motor[0].attach(motorPins[0], motorPins[1], kMotor, kiMotor);
  motor[1].attach(motorPins[2], motorPins[3], kMotor, kiMotor);
  motor[2].attach(motorPins[4], motorPins[5], kMotor, kiMotor);
  motor[3].attach(motorPins[6], motorPins[7], kMotor, kiMotor);
  pinMode(directionPins[0], 0);
  pinMode(directionPins[0], 0);
  pinMode(directionPins[0], 0);
  pinMode(directionPins[0], 0);
  Serial.begin(115200);
  Serial2.begin(115200 * 3);
  motorEncs[0] = motor[0].interruptEnc();
  motorEncs[1] = motor[1].interruptEnc();
  motorEncs[2] = motor[2].interruptEnc();
  motorEncs[3] = motor[3].interruptEnc();
  attachInterrupt(interrupts[0], interr0, RISING);
  attachInterrupt(interrupts[1], interr1, RISING);
  attachInterrupt(interrupts[2], interr2, RISING);
  attachInterrupt(interrupts[3], interr3, RISING);

  Wire.begin();
  // Wire.setClock(1000000UL);  // разгоняем шину на максимум

  // инициализация DMP
  mpu.initialize();

  mpu.dmpInitialize();

  EEPROM.get(0, dataEEPROM);

  mpu.setXAccelOffset(dataEEPROM.XAccelOffset);
  mpu.setYAccelOffset(dataEEPROM.YAccelOffset);
  mpu.setZAccelOffset(dataEEPROM.ZAccelOffset);
  mpu.setXGyroOffset(dataEEPROM.XGyroOffset);
  mpu.setYGyroOffset(dataEEPROM.YGyroOffset);
  mpu.setZGyroOffset(dataEEPROM.ZGyroOffset);

  mpu.setDMPEnabled(true);

  oldTime = millis();
  // Serial.println("aaa");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // расчёты
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    for (int i = 0; i < 2; i++) {
      if (abs(ypr[i * 2] - oldYpr[i * 2]) > M_PI and i * 2 == 0) {
        corr += 2 * M_PI * sgn(oldYpr[i * 2] - ypr[i * 2]);
      }
      toSerial.yr[i] = (ypr[i * 2] + corr * (i == 0)) * kYR;
      oldYpr[i * 2] = ypr[i * 2];
      Serial.print(toSerial.yr[i]);
      Serial.print(" ");
    }
    Serial.println();
  }

  if (millis() > Ts + myTimer && deltaN < deltaNMax) {
    writeSerial();
    myTimer = millis();
    deltaN++;
  }

  for (int i = 0; i < 2; i++) {
    toSerial.enc[i] = (motor[i * 2].returnEnc() + motor[i * 2 + 1].returnEnc()) / 2;
  }

  if (Serial2.available()) {
    deltaN = max(deltaN - 1, 0);
    if (!readSerial()) Serial.println("err with Serial2");
    for (int i = 0; i < 2; i++) {
      // fromSerial.encMotor[i] = constrain(millis() * 0.0001 * (2 * i - 1), -580, 580);
      for (int j = 0; j < 2; j++) {
        motor[i * 2 + j].setAngle(fromSerial.encMotor[i]);
        if (abs(motor[i * 2 + 1].returnEnc() - motor[i * 2].returnEnc()) > dMax && abs(motor[i * 2 + j].returnEnc() - fromSerial.encMotor[i]) < abs(motor[i * 2 + 1 - j].returnEnc() - fromSerial.encMotor[i])) {
          motor[i * 2 + j].setAngle(motor[i * 2 + 1 - j].returnEnc() + sgn(fromSerial.encMotor[i] - motor[i * 2 + 1 - j].returnEnc()) * dMax);
          Serial.println(i * 2 + j);
          Serial.println(motor[i * 2 + 1 - j].returnEnc());
          Serial.println(sgn(fromSerial.encMotor[i] - motor[i * 2 + 1 - j].returnEnc()) * dMax);
        }
      }
    }
    // Serial.println(String(fromSerial.encMotor[0]) + " " + String(motor[0].returnEnc()) + " " + String(motor[1].returnEnc()) + " " + String(motor[2].returnEnc()) + " " + String(motor[3].returnEnc()));
  }


  for (int i = 0; i < 4; i++) {
    motor[i].regAngle(millis() - oldTime);
    // Serial.print(motor[i].returnEnc());
    // Serial.print(" ");
  }
  oldTime = millis();
}

bool readSerial() {
  static byte crc;
  if (errSerial) {
    Serial2.read();
    errSerial = 0;
  }
  Serial2.readBytes((byte *)&buf, sizeof(buf));
  crc = crc8_bytes((byte *)&buf, sizeof(buf));
  if (crc == 0) {
    for (int i = 0; i < 2; i++) {
      fromSerial.encMotor[i] = buf.encMotor[i];
    }
    return 1;
  }
  errSerial = 1;
  return 0;
}

void writeSerial() {
  toSerial.crc = crc8_bytes((byte *)&toSerial, sizeof(toSerial) - 1);
  Serial2.write((byte *)&toSerial, sizeof(toSerial));
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
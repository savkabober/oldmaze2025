//инклуды
#include <Wire.h>
#include "VL53L0X.h"
#include <EEPROM.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

//дефайны
#define sgn(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))

//функции
void turnAngle(double angle, bool detect = false);
int16_t toRange(int16_t value, int16_t maxVal);
void qReset();
void configReset();
void weightReset();
void weightSet(int16_t val, int16_t x, int16_t y, int16_t z);
int16_t weightGet(int16_t x, int16_t y, int16_t z);
void visitSet(int8_t val, int16_t x, int16_t y, int16_t z);
int8_t visitGet(int16_t x, int16_t y, int16_t z);
void wallSet(int8_t val, int16_t x, int16_t y, int16_t z, int8_t orient);
int8_t wallGet(int16_t x, int16_t y, int16_t z, int8_t orient);
void roofSet(int8_t val, int16_t x, int16_t y, int16_t z, int8_t orient);
int8_t roofGet(int16_t x, int16_t y, int16_t z, int8_t orient);
void orientSet(int8_t val, int16_t x, int16_t y, int16_t z);
int8_t orientGet(int16_t x, int16_t y, int16_t z);
void typeSet(int8_t val, int16_t x, int16_t y, int16_t z);
int8_t typeGet(int16_t x, int16_t y, int16_t z);
int8_t xP(int8_t orient);
int8_t yP(int8_t orient);
int8_t zP(int8_t orient);
int16_t weightDelta(int16_t x, int16_t y, int16_t z, int8_t orient);

//константы, переменные
const int16_t xMax = 10, yMax = 10, zMax = 2, qLen = 100, wayLen = 100;
int16_t rbtPos[4] = { 0, 0, 0, 0 }, goalPos[3] = { 0, 0, 0 }, virtPos[3];
uint8_t configMas[xMax][yMax][zMax], weightMas[xMax][yMax][zMax];
int8_t way[wayLen];
int16_t qMas[qLen][3];
int16_t lox_mosfets[6] = { 22, 23, 24, A13, A14, A15 };
int16_t lox_adresses[6] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36 };
int16_t lox_shts[6] = { 31, 32, 33, 30, 35, 34 };
int16_t lasers[6];
const int32_t tS = 10;
bool errSerial = false;
int16_t vMax = 5000, aMax = 5000, squareEnc = 1000, maxCorr = 100, forwButStop = 200, vMin = 400, blackStartEnc = 200, brickEnc = 150, aMaxTurn = 5000, vMinTurn = 300, vMaxTurn = 5000, vReset = 400;
int32_t myTimer;
float staticGyro = 0, zeroGyro[2] = { 0, 0 }, gyro[2] = { 0, 0 }, encoders[2] = { 0, 0 }, oldEncs[2] = { 0, 0 };
float kRadEnc = 369, kGyro = 50, k1Laser = 1.5, k2Laser = 1.5, kSymb = 0.25;
int16_t edgeDist[2] = { 120, 150 }, forwardDist[3] = { 100, 410, 660 }, backwardDist[3] = { 80, 400, 700 }, sideDist[2] = { 110, 140 };
int16_t extremeSideDist[2] = { 170, 200 }, extremeStraightDist[2] = { 160, 130 }, extremeEdgeDist[2] = { 180, 210 };
float kFilterLaser = 0.5, kFilterLight = 0.5, horizont, dLevelHorizont = 100, dLevelGyro = 0.26, brickAngle = 0.5, dMaxGyro = 0.1;
int16_t forwBut[2] = { 2, 3 }, brightness = 255, ledPin = 4, ledCount = 26;
int32_t deltaEncMax = 150;
int8_t lightPin = 0;
int16_t colors[3][2] = { { 100, 200 }, { 300, 400 }, { 500, 600 } };
int8_t dLevel = 0, nowHelps[4] = {0, 0, 0, 0};
int8_t switches[2] = { 0, 0 };
int16_t nBlacks;
double virtEnc[2] = { 0, 0 };
int32_t delayReset = 500;
int16_t nMaxGyro = 2, nGyro = 0, visibleH[2] = {0, 0}, nFlash = 5, nLights = 13, helps[6] = { 2, 0, 3, 0, 1, 1 }, numHelps = 0, numHelpsMax = 12;
char zeroSymb = '0';
int16_t srednServo = 100, servoPin = 5, dServo = 10;

//структуры
struct DataFromSerial {
  float yr[2];
  int32_t enc[2];
};

struct Buffer {
  float yr[2];
  int32_t enc[2];
  byte crc;
};

struct DataToSerial {
  int32_t encMotor[2];
  byte crc;
};

//объекты классов, структур
VL53L0X lox[6];
DataFromSerial fromSerial;
DataToSerial toSerial;
Buffer buf;
Servo myServo;
Adafruit_NeoPixel strip(ledCount, ledPin, NEO_GRB);

void setup() {
  pinMode(13, OUTPUT);
  myTimer = millis() + 2000;
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200 * 3);
  Wire.begin();
  Serial.println("aaa");
  setID();
  qReset();
  configReset();
  weightReset();
  pinMode(forwBut[0], INPUT_PULLUP);
  pinMode(forwBut[1], INPUT_PULLUP);
  myServo.attach(5);
  myServo.write(srednServo + dServo);
  delay(200);
  myServo.write(srednServo - dServo);
  strip.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();   // Turn OFF all pixels ASAP
  strip.setBrightness(brightness);
  for (int i = 0; i < 15; i++) {
    readLasers();
  }
  // for (;;) {
  //   updateNearConfig();
  //   Serial.println(String(wallGet(0, 0, 0, 0)) + " " + String(wallGet(0, 0, 0, 1)) + " " + String(wallGet(0, 0, 0, 2)) + " " + String(wallGet(0, 0, 0, 3)));
  // }
  wallSet(1, 0, 0, 0, 0);
  Serial.println(wallGet(0, 0, 0, 0));
  // throwHelps(1, 1);
  while(findWay());
  myServo.detach();

  // while(1) Serial.println(String(digitalRead(forwBut[0])) + " " + String(digitalRead(forwBut[1])));
  // for(int i = 0; i < 4; i++) {
  //   turnOrient(i);
  // }
  // toSerial.encMotor[0] = toSerial.encMotor[1] = 200;
  // writeSerial();
}

void loop() {
}

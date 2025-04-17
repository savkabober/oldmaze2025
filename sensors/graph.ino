int16_t toRange(int16_t value, int16_t maxVal) {
  value %= maxVal;
  if (value < 0) {
    value += maxVal;
  }
  return value;
}

void qReset() {
  for (int i = 0; i < qLen; i++) {
    for (int j = 0; j < 3; j++) {
      qMas[i][j] = 0;
    }
  }
}

void wayReset() {
  for (int i = 0; i < wayLen; i++) {
    way[i] = 0;
  }
}

void configReset() {
  for (int x = 0; x < xMax; x++) {
    for (int y = 0; y < yMax; y++) {
      for (int z = 0; z < zMax; z++) {
        for (int i = 0; i < 7; i++) {
          bitWrite(configMas[x][y][z], i, 0);
        }
      }
    }
  }
}

void weightReset() {
  for (int x = 0; x < xMax; x++) {
    for (int y = 0; y < yMax; y++) {
      for (int z = 0; z < zMax; z++) {
        bitWrite(configMas[x][y][z], 7, 0);
        weightMas[x][y][z] = 0;
      }
    }
  }
}

void weightSet(int16_t val, int16_t x, int16_t y, int16_t z) {
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z, zMax);
  weightMas[x][y][z] = val % 256;
  bitWrite(configMas[x][y][z], 7, val / 256);
}

int16_t weightGet(int16_t x, int16_t y, int16_t z) {
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z, zMax);
  return weightMas[x][y][z] + 256 * bitRead(configMas[x][y][z], 7);
}

void visitSet(int8_t val, int16_t x, int16_t y, int16_t z) {
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z, zMax);
  bitWrite(configMas[x][y][z], 0, val);
}

int8_t visitGet(int16_t x, int16_t y, int16_t z) {
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z, zMax);
  return bitRead(configMas[x][y][z], 0);
}

void wallSet(int8_t val, int16_t x, int16_t y, int16_t z, int8_t orient) {
  orient = toRange(orient, 4);
  x = toRange(x - (orient == 3), xMax);
  y = toRange(y - (orient == 2), yMax);
  z = toRange(z, zMax);
  orient %= 2;
  bitWrite(configMas[x][y][z], 1 + orient, val);
}

int8_t wallGet(int16_t x, int16_t y, int16_t z, int8_t orient) {
  orient = toRange(orient, 4);
  x = toRange(x - (orient == 3), xMax);
  y = toRange(y - (orient == 2), yMax);
  z = toRange(z, zMax);
  orient %= 2;
  return bitRead(configMas[x][y][z], 1 + orient);
}

void roofSet(int8_t val, int16_t x, int16_t y, int16_t z, int8_t orient) {
  orient = toRange(orient, 2);
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z - (orient == 0), zMax);
  bitWrite(configMas[x][y][z], 3, val);
}

int8_t roofGet(int16_t x, int16_t y, int16_t z, int8_t orient) {
  orient = toRange(orient, 2);
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z - (orient == 0), zMax);
  return bitRead(configMas[x][y][z], 3);
}

void orientSet(int8_t val, int16_t x, int16_t y, int16_t z) {
  val = toRange(val, 2);
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z, zMax);
  bitWrite(configMas[x][y][z], 4, val);
}

int8_t orientGet(int16_t x, int16_t y, int16_t z) {
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z, zMax);
  return bitRead(configMas[x][y][z], 4);
}

void typeSet(int8_t val, int16_t x, int16_t y, int16_t z) {
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z, zMax);
  bitWrite(configMas[x][y][z], 5, val % 2);
  bitWrite(configMas[x][y][z], 6, val / 2);
}

int8_t typeGet(int16_t x, int16_t y, int16_t z) {
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z, zMax);
  return bitRead(configMas[x][y][z], 5) + 2 * bitRead(configMas[x][y][z], 6);
}

int8_t xP(int8_t orient) {
  return (orient % 2 != 0) * (-toRange(orient, 4) + 2);
}

int8_t yP(int8_t orient) {
  return (orient % 2 == 0) * (-toRange(orient, 4) + 1);
}

int8_t zP(int8_t orient) {
  return toRange(orient, 2) * 2 - 1;
}

int16_t weightDelta(int16_t x, int16_t y, int16_t z, int8_t orient) {
  orient = toRange(orient, 4);
  x = toRange(x, xMax);
  y = toRange(y, yMax);
  z = toRange(z, zMax);
  static int16_t weightPlus;
  weightPlus = (orient % 2) ^ orientGet(x, y, z);
  if (x == rbtPos[0] && y == rbtPos[1] && z == rbtPos[2] && toRange(orient - rbtPos[3], 4) == 2) {
    weightPlus = 2;
  }
  static int8_t typeSquare;
  typeSquare = typeGet(x, y, z);
  if (typeSquare == 2) {
    weightPlus += 5;
  }
  if (typeSquare == 3) {
    weightPlus++;
  }
  return weightPlus + 1;
}

void updateNearConfig() {
  readLasers();
  if (!dLevel) {
    for (int i = 0; i < 2; i++) {
      if (lasers[i * 5] > extremeStraightDist[i]) {
        wallSet(1, rbtPos[0], rbtPos[1], rbtPos[2], rbtPos[3] + 2 * i);
      } else {
        wallSet(0, rbtPos[0], rbtPos[1], rbtPos[2], rbtPos[3] + 2 * i);
      }
    }
    for (int i = 0; i < 2; i++) {
      if (lasers[3 + i] > extremeSideDist[i]) {
        wallSet(1, rbtPos[0], rbtPos[1], rbtPos[2], rbtPos[3] + (2 * i - 1));
      } else {
        wallSet(0, rbtPos[0], rbtPos[1], rbtPos[2], rbtPos[3] + (2 * i - 1));
      }
    }
  } else {
    wallSet(1, rbtPos[0], rbtPos[1], rbtPos[2], rbtPos[3]);
  }
}

bool findWay() {
  static int16_t qI, qMax, weightPlus, wayI, weightNow, weightNext;
  Serial.println("bil");
  Serial.println(visitGet(0, 0, 0));
  Serial.println(visitGet(0, 1, 0));
  Serial.println(visitGet(-1, 1, 0));
  updateNearConfig();
  weightReset();
  wayReset();
  qReset();
  for (int i = 0; i < 3; i++) {
    qMas[0][i] = rbtPos[i];
    goalPos[i] = 0;
  }
  weightSet(1, rbtPos[0], rbtPos[1], rbtPos[2]);
  orientSet(rbtPos[3], rbtPos[0], rbtPos[1], rbtPos[2]);
  qI = 0;
  qMax = 1;
  while (qI != qMax) {
    for (int i = 0; i < 4; i++) {
      if (wallGet(qMas[qI][0], qMas[qI][1], qMas[qI][2], i) && typeGet(qMas[qI][0] + xP(i), qMas[qI][1] + yP(i), qMas[qI][2]) != 1) {
        weightPlus = weightDelta(qMas[qI][0], qMas[qI][1], qMas[qI][2], i);
        weightNext = weightGet(qMas[qI][0] + xP(i), qMas[qI][1] + yP(i), qMas[qI][2]);
        weightNow = weightGet(qMas[qI][0], qMas[qI][1], qMas[qI][2]);
        if (weightNext == 0 || weightNext > weightPlus + weightNow) {
          weightSet(weightPlus + weightNow, qMas[qI][0] + xP(i), qMas[qI][1] + yP(i), qMas[qI][2]);
          orientSet(i % 2, qMas[qI][0] + xP(i), qMas[qI][1] + yP(i), qMas[qI][2]);
          Serial.println("vot " + String(qMas[qI][0] + xP(i)) + " " + String(qMas[qI][1] + yP(i)));
          if (!visitGet(qMas[qI][0] + xP(i), qMas[qI][1] + yP(i), qMas[qI][2])) {
            if (weightNow + weightPlus < weightGet(goalPos[0], goalPos[1], goalPos[2]) || (goalPos[0] == 0 && goalPos[1] == 0 && goalPos[2] == 0)) {
              goalPos[0] = toRange(qMas[qI][0] + xP(i), xMax);
              goalPos[1] = toRange(qMas[qI][1] + yP(i), yMax);
              goalPos[2] = qMas[qI][2];
              Serial.println("novi " + String(weightNow + weightPlus));
            }
          } else {
            qMas[qMax][0] = toRange(qMas[qI][0] + xP(i), xMax);
            qMas[qMax][1] = toRange(qMas[qI][1] + yP(i), yMax);
            qMas[qMax][2] = qMas[qI][2];
            qMax = toRange(qMax + 1, qLen);
          }
        }
      }
    }
    for (int i = 0; i < 2; i++) {
      if (roofGet(qMas[qI][0], qMas[qI][1], qMas[qI][2], i)) {
        weightNext = weightGet(qMas[qI][0], qMas[qI][1], qMas[qI][2] + zP(i));
        weightNow = weightGet(qMas[qI][0], qMas[qI][1], qMas[qI][2]);
        if (weightNext == 0 || weightNext > 1 + weightNow) {
          weightSet(1 + weightNow, qMas[qI][0], qMas[qI][1], qMas[qI][2] + zP(i));
          orientSet(orientGet(qMas[qI][0], qMas[qI][1], qMas[qI][2]), qMas[qI][0], qMas[qI][1], qMas[qI][2] + zP(i));
          qMas[qMax][0] = qMas[qI][0];
          qMas[qMax][1] = qMas[qI][1];
          qMas[qMax][2] = toRange(qMas[qI][2] + zP(i), zMax);
          qMax = toRange(qMax + 1, qLen);
        }
      }
    }
    qI = toRange(qI + 1, qLen);
  }
  Serial.println("goal");
  Serial.println(String(goalPos[0]) + " " + String(goalPos[1]) + " " + String(goalPos[2]));
  Serial.println(String(weightGet(0, 1, 0)) + " " + String(weightGet(1, 0, 0)) + " " + String(weightGet(0, -1, 0)) + " " + String(weightGet(-1, 0, 0)));
  for (int i = 0; i < 3; i++) {
    virtPos[i] = goalPos[i];
  }
  wayI = 0;
  while (virtPos[0] != rbtPos[0] || virtPos[1] != rbtPos[1] || virtPos[2] != rbtPos[2]) {
    for (int i = 0; i < 4; i++) {
      if (weightGet(virtPos[0], virtPos[1], virtPos[2]) - weightDelta(virtPos[0] + xP(i), virtPos[1] + yP(i), virtPos[2], i + 2)
            == weightGet(virtPos[0] + xP(i), virtPos[1] + yP(i), virtPos[2])
          && wallGet(virtPos[0], virtPos[1], virtPos[2], i) && weightGet(virtPos[0] + xP(i), virtPos[1] + yP(i), virtPos[2]) != 0) {
        way[wayI] = toRange(i + 2, 4) + 1;
        Serial.println("dwiqhd9qw");
        Serial.println(String(wayI) + " " + String(way[wayI]));
        virtPos[0] = toRange(virtPos[0] + xP(i), xMax);
        virtPos[1] = toRange(virtPos[1] + yP(i), yMax);
        wayI++;
      }
      Serial.println(i);
      Serial.println(String(virtPos[0]) + " " + String(virtPos[1]) + " " + String(virtPos[2]));
    }
    for (int i = 0; i < 2; i++) {
      if (weightGet(virtPos[0], virtPos[1], virtPos[2]) - 1 == weightGet(virtPos[0], virtPos[1], virtPos[2] + zP(i)) && roofGet(virtPos[0], virtPos[1], virtPos[2], i)
          && weightGet(virtPos[0], virtPos[1], virtPos[2] + zP(i)) != 0) {
        virtPos[2] = toRange(virtPos[2] + zP(i), zMax);
        wayI++;
      }
    }
  }
  Serial.println("writing way");
  for (int i = wayLen - 1; i >= 0; i--) {
    if (way[i] != 0) {
      Serial.println("way:");
      Serial.println(String(way[i]) + " " + String(i));
      turnOrient(way[i] - 1);
      forward();
    }
  }
  Serial.println("written");
  if (goalPos[0] == 0 && goalPos[1] == 0 && goalPos[2] == 0) {
    return false;
  }
  return true;
}
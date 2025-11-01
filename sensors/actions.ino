double tToX(double t, double s, double v, double a) {
  static double aS, aT;
  aS = abs(s);
  aT = abs(t);
  if (aS < v * v / a) {
    if (aT * aT > 4 * aS / a) {
      return s * sgn(t);
    }
    if (aT * aT < aS / a) {
      return a * aT * aT / 2 * sgn(s) * sgn(t);
    }
    return (2 * aT * sqrt(a * aS) - aS - a * aT * aT / 2) * sgn(s) * sgn(t);
  }
  if (aT > aS / v + v / a) {
    return s * sgn(t);
  }
  if (aT < v / a) {
    return a * aT * aT / s * sgn(s) * sgn(t);
  }
  if (aT < s / v) {
    return (v * aT - v * v / a / 2) * sgn(s) * sgn(t);
  }
  return (a * aT * aS / v + v * aT - v * v / a / 2 - a * aT * aT / 2 - a * aS * aS / v / v / 2) * sgn(s) * sgn(t);
}

double timeToGo(double s, double v, double a) {
  static double aS;
  aS = abs(s);
  if (aS < v * v / a) {
    return 2 * sqrt(aS / a);
  }
  return aS / v + v / a;
}

double xToT(double x, double s, double v, double a) {
  static double aX, aS;
  aX = abs(x);
  aS = abs(s);
  if (aS < v * v / a) {
    if (aX > aS) {
      return 2 * sqrt(aS / a) * sgn(x * s);
    }
    if (aX < aS / 2) {
      return sqrt(2 * aX / a) * sgn(x * s);
    }
    return (2 * sqrt(aS / a) - sqrt(2 * (aS - aX) / a)) * sgn(x * s);
  }
  if (aX > aS) {
    return (aS / v + v / a) * sgn(x * s);
  }
  if (aX < v * v / a / 2) {
    return sqrt(2 * aX / a) * sgn(x * s);
  }
  if (aX < aS - v * v / a / 2) {
    return (aX / v + v / a / 2) * sgn(x * s);
  }
  return (v / a + aS / v - sqrt(2 * (aS - aX) / a)) * sgn(x * s);
}

void forward() {
  getFirstValues();
  double localVirt = 0, oldLocalVirt = 0, deltaEnc, vU, corrLocalVirt;
  bool rideFlag = true, blackSquare = false;
  int32_t localTime = 0, deltaTime;
  bool normalTS = true;
  double light = 0;
  int16_t buffH[2] = { 0, 0 };
  horizont = 0;
  while (rideFlag) {
    readMc();
    if (millis() > myTimer + tS) {
      //рассчитать время, успевание программы
      deltaTime = millis() - myTimer;
      myTimer = millis();
      if (!normalTS) {
        Serial.println("это жесть, задержка " + String(deltaTime) + " мкс.");
      }
      normalTS = false;
      //прочитать значения лазеров
      readLasers();
      if ((abs(virtEnc[0] - encoders[0]) + abs(virtEnc[1] - encoders[1])) / 2 < deltaEncMax) {
        //подкорректировать положение виртуальной точки
        if (abs(gyro[0] - staticGyro) < dMaxGyro && abs(gyro[1]) < dMaxGyro) {
          digitalWrite(13, 1);
          for (int i = 0; i < sizeof(backwardDist) / sizeof(backwardDist[0]) - 1; i++) {
            corrLocalVirt = double(lasers[5] - backwardDist[i]) / (backwardDist[i + 1] - backwardDist[i]) * squareEnc;
            corrLocalVirt += double(virtEnc[0] + virtEnc[1] - encoders[0] - encoders[1]) * cos(gyro[0] - staticGyro) * cos(gyro[1]) / 2;
            if (abs(corrLocalVirt - oldLocalVirt) < maxCorr) {
              oldLocalVirt = corrLocalVirt;
              localTime = xToT(oldLocalVirt, squareEnc, vMax, aMax) * 1000;
            }
          }
          for (int i = 0; i < sizeof(forwardDist) / sizeof(forwardDist[0]) - 1; i++) {
            corrLocalVirt = double(forwardDist[i + 1] - lasers[0]) / (forwardDist[i + 1] - forwardDist[i]) * squareEnc;
            corrLocalVirt += double(virtEnc[0] + virtEnc[1] - encoders[0] - encoders[1]) * cos(gyro[0] - staticGyro) * cos(gyro[1]) / 2;
            if (abs(corrLocalVirt - oldLocalVirt) < maxCorr) {
              oldLocalVirt = corrLocalVirt;
              localTime = xToT(oldLocalVirt, squareEnc, vMax, aMax) * 1000;
            }
          }
        } else {
          digitalWrite(13, 0);
        }
        //рассчитать виртуальную точку по времени
        localTime += deltaTime;
        localVirt = tToX(double(localTime) / 1000, squareEnc, vMax, aMax);
        if (oldLocalVirt < 0 || (localVirt - oldLocalVirt) < vMin * double(deltaTime) / 1000) {
          localVirt = oldLocalVirt + vMin * double(deltaTime) / 1000;
          localTime = xToT(localVirt, squareEnc, vMax, aMax) * 1000;
          localVirt = min(localVirt, squareEnc);
        }
        deltaEnc = (localVirt - oldLocalVirt) / cos(gyro[0] - staticGyro) / cos(gyro[1]);
        oldLocalVirt = localVirt;
        //выравняться параллельно стенкам
        vU = 0;
        for (int i = 0; i < 2; i++) {
          if (lasers[1 + i] < extremeEdgeDist[i]) {
            vU -= (edgeDist[i] - lasers[1 + i]) * k1Laser * (i * 2 - 1);
            if (lasers[3 + i] < extremeSideDist[i]) {
              vU -= (lasers[3 + i] - lasers[1 + i] - sideDist[i] + edgeDist[i]) * k2Laser * (i * 2 - 1);
            }
          }
        }
        vU += (staticGyro - gyro[0]) * kGyro;
        virtEnc[0] += deltaEnc;
        virtEnc[1] += deltaEnc;
        virtEnc[0] += vU * double(deltaTime) / 1000;
        virtEnc[1] -= vU * double(deltaTime) / 1000;
      }
      //отослать значения целевых точек на проц моторов
      writeMc();
      //проверка, не пора ли выходить из цикла
      if (timeToGo(squareEnc, vMax, aMax) * 1000 - 1 < localTime) {
        Serial.println(localTime);
        rideFlag = false;
      }
      if (digitalRead(forwBut[0]) && digitalRead(forwBut[1]) && localVirt + forwButStop > squareEnc) {
        rideFlag = false;
        Serial.println("knopki");
      }
      //черная клетка
      light = kFilterLight * analogRead(lightPin) + (1 - kFilterLight) * light;
      if (light > colors[0][0] && light < colors[0][1] && 0) {
        rideFlag = false;
        visitSet(1, rbtPos[0], rbtPos[1], rbtPos[2]);
        typeSet(1, rbtPos[0] + xP(rbtPos[3]), rbtPos[1] + yP(rbtPos[3]), rbtPos[2]);
        localVirt -= double(virtEnc[0] + virtEnc[1] - encoders[0] - encoders[1]) * cos(gyro[0] - staticGyro) * cos(gyro[1]) / 2;
        oldLocalVirt = localVirt;
        fastStop();
        while (roofGet(rbtPos[0], rbtPos[1], rbtPos[2], 0) || roofGet(rbtPos[0], rbtPos[1], rbtPos[2], 1)) {
          nBlacks++;
          if (roofGet(rbtPos[0], rbtPos[1], rbtPos[2], 0)) {
            rbtPos[2] = toRange(rbtPos[2] - 1, zMax);
          } else {
            rbtPos[2] = toRange(rbtPos[2] + 1, zMax);
          }
          rbtPos[0] = toRange(rbtPos[0] - xP(rbtPos[3]), xMax);
          rbtPos[1] = toRange(rbtPos[1] - yP(rbtPos[3]), yMax);
        }
        forwardEnc(-localVirt - nBlacks * squareEnc);
        myTimer = millis();
        blackSquare = true;
      }
      //объезд кирпича
      switches[0] = digitalRead(forwBut[0]);
      switches[1] = digitalRead(forwBut[1]);
      if (lasers[0] > extremeStraightDist[0] && (switches[0] ^ switches[1])) {
        localVirt -= double(virtEnc[0] + virtEnc[1] - encoders[0] - encoders[1]) * cos(gyro[0] - staticGyro) * cos(gyro[1]) / 2;
        oldLocalVirt = localVirt;
        fastStop();
        forwardEnc(-2 * brickEnc);
        turnAngle(brickAngle * (2 * switches[0] - 1));
        forwardEnc(brickEnc / cos(brickAngle));
        turnAngle(-brickAngle * (2 * switches[0] - 1));
        forwardEnc(brickEnc);
        myTimer = millis();
      }
      //спасательные комплекты
      readCam();
      if (oldLocalVirt / squareEnc < kSymb) {
        for (int i = 0; i < 2; i++) {
          if (lasers[3 + i] < extremeSideDist[i] && !wallGet(rbtPos[0], rbtPos[1], rbtPos[2], rbtPos[3] + (i * 2) - 1) && !visitGet(rbtPos[0], rbtPos[1], rbtPos[2])
              && !nowHelps[toRange(rbtPos[3] + (i * 2) - 1, 4)] && visibleH[i] != 0) {
            nowHelps[toRange(rbtPos[3] + (i * 2) - 1, 4)] = 1;
            localVirt -= double(virtEnc[0] + virtEnc[1] - encoders[0] - encoders[1]) * cos(gyro[0] - staticGyro) * cos(gyro[1]) / 2;
            oldLocalVirt = localVirt;
            fastStop();
            throwHelps(visibleH[i], i * 2 - 1);
          }
        }
      }
      if (oldLocalVirt / squareEnc > 1.0 - kSymb) {
        for (int i = 0; i < 2; i++) {
          if (visibleH[i] != 0 && lasers[3 + i] < extremeSideDist[i]) {
            buffH[i] = visibleH[i];
          }
        }
      }
    } else {
      normalTS = true;
    }
  }
  if (!blackSquare) {
    visitSet(1, rbtPos[0], rbtPos[1], rbtPos[2]);
    Serial.println(visitGet(rbtPos[0], rbtPos[1], rbtPos[2]));
    Serial.println(String(rbtPos[0]) + " " + String(rbtPos[1]) + " " + String(rbtPos[2]));
    Serial.println("postavil");
    nowHelps[0] = nowHelps[1] = nowHelps[2] = nowHelps[3] = 0;
    for (int i = 0; i < 2; i++) {
      if (lasers[3 + i] < extremeSideDist[i] && !visitGet(rbtPos[0] + xP(rbtPos[3]), rbtPos[1] + yP(rbtPos[3]), rbtPos[2]) && buffH[i] != 0) {
        nowHelps[toRange(rbtPos[3] + (i * 2) - 1, 4)] = 1;
        throwHelps(buffH[i], i * 2 - 1);
      }
    }
    if (abs(horizont) > dLevelHorizont && abs(gyro[1]) > dLevelGyro && horizont * gyro[1] > 0) {
      dLevel = sgn(gyro[1]);
      roofSet(1, rbtPos[0], rbtPos[1], rbtPos[2], (dLevel + 1) / 2);
      rbtPos[2] = toRange(rbtPos[2] + zP(dLevel), zMax);
      visitSet(1, rbtPos[0], rbtPos[1], rbtPos[2]);
    } else {
      dLevel = 0;
    }
    //синяя клетка
    if (light > colors[1][0] && light < colors[1][1]) {
      delay(5000);
    }
    //серебристая клетка
    if (light > colors[2][0] && light < colors[2][1]) {
      delay(1000);
    }
    rbtPos[0] = toRange(rbtPos[0] + xP(rbtPos[3]), xMax);
    rbtPos[1] = toRange(rbtPos[1] + yP(rbtPos[3]), yMax);
    if (lasers[0] < extremeStraightDist[0]) {
      nGyro = (nGyro + 1) % nMaxGyro;
      if (nGyro == 0) {
        resetGyro();
      }
    }
  }
}

bool readMc() {
  // Serial.println(virtEnc[0]);
  if (Serial3.available()) {
    if (!readMySerial()) {
      // Serial.println("чето со связью с моторами.");
      return false;
    }
    // Serial.println("AAAAA");
    return true;
  }
  // Serial.println("yttnrjg");
  return false;
}

void turnOrient(int8_t orient) {
  int8_t deltaOrient = toRange(orient - rbtPos[3], 4);
  if (deltaOrient) {
    if (deltaOrient > 2) {
      deltaOrient -= 4;
    }
    for (int i = 0; i < abs(deltaOrient); i++) {
      turnAngle(M_PI / 2 * sgn(deltaOrient), true);
    }
    rbtPos[3] = toRange(orient, 4);
  }
}

void turnAngle(double angle, bool detect = false) {
  getFirstValues();
  double localVirt = 0, oldLocalVirt = 0, deltaEnc, distance = angle * kRadEnc;
  bool rideFlag = true;
  int32_t localTime = 0, deltaTime;
  bool normalTS = true;
  int16_t buffH[2] = { 0, 0 };
  while (rideFlag) {
    readMc();
    if (millis() > myTimer + tS) {
      Serial.println(String(timeToGo(distance, vMaxTurn, aMaxTurn) * 1000) + " " + String(localTime));
      //рассчитать время, успевание программы
      deltaTime = millis() - myTimer;
      myTimer = millis();
      if (!normalTS) {
        Serial.println("это жесть, задержка " + String(deltaTime) + " мкс.");
      }
      normalTS = false;
      //прочитать значения лазеров
      readLasers();
      if ((abs(virtEnc[0] - encoders[0]) + abs(virtEnc[1] - encoders[1])) / 2 < deltaEncMax) {
        //подкорректировать положение виртуальной точки
        oldLocalVirt = (gyro[0] - staticGyro) * kRadEnc;
        oldLocalVirt += double(virtEnc[0] - virtEnc[1] - encoders[0] + encoders[1]) / 2;
        localTime = xToT(oldLocalVirt, distance, vMaxTurn, aMaxTurn) * 1000;
        //рассчитать виртуальную точку по времени
        localTime += deltaTime;
        localVirt = tToX(double(localTime) / 1000, distance, vMaxTurn, aMaxTurn);
        if (oldLocalVirt * distance < 0 || (localVirt - oldLocalVirt) * sgn(distance) < vMinTurn * double(deltaTime) / 1000) {
          localVirt = oldLocalVirt + vMinTurn * sgn(distance) * double(deltaTime) / 1000;
          localTime = xToT(localVirt, distance, vMaxTurn, aMaxTurn) * 1000;
          localVirt = min(localVirt * sgn(distance), abs(distance)) * sgn(distance);
        }
        deltaEnc = localVirt - oldLocalVirt;
        oldLocalVirt = localVirt;
        virtEnc[0] += deltaEnc;
        virtEnc[1] -= deltaEnc;
      }
      //отослать значения целевых точек на проц моторов
      writeMc();
      //проверка, не пора ли выходить из цикла
      if (timeToGo(distance, vMaxTurn, aMaxTurn) * 1000 - 1 < localTime) {
        rideFlag = false;
      }
      if (detect) {
        //спасательные комплекты
        readCam();
        if (oldLocalVirt / distance < kSymb) {
          for (int i = 0; i < 2; i++) {
            if (lasers[3 + i] < extremeSideDist[i] && !wallGet(rbtPos[0], rbtPos[1], rbtPos[2], rbtPos[3] + (i * 2) - 1) && !visitGet(rbtPos[0], rbtPos[1], rbtPos[2])
                && !nowHelps[toRange(rbtPos[3] + (i * 2) - 1, 4)] && visibleH[i] != 0) {
              nowHelps[toRange(rbtPos[3] + (i * 2) - 1, 4)] = 1;
              localVirt -= double(virtEnc[0] - virtEnc[1] - encoders[0] + encoders[1]) / 2;
              fastStop();
              throwHelps(visibleH[i], i * 2 - 1);
            }
          }
        }
        if (oldLocalVirt / distance > 1.0 - kSymb) {
          for (int i = 0; i < 2; i++) {
            if (visibleH[i] != 0 && lasers[3 + i] < extremeSideDist[i]) {
              buffH[i] = visibleH[i];
            }
          }
        }
      }
    } else {
      normalTS = true;
    }
  }
  if (detect) {
    for (int i = 0; i < 2; i++) {
      if (lasers[3 + i] < extremeSideDist[i] && !visitGet(rbtPos[0], rbtPos[1], rbtPos[2]) && buffH[i] != 0 && !nowHelps[toRange(rbtPos[3] + (i * 2) - 1 + sgn(angle), 4)]) {
        nowHelps[toRange(rbtPos[3] + (i * 2) - 1 + sgn(angle), 4)] = 1;
        throwHelps(buffH[i], i * 2 - 1);
      }
    }
  }

  staticGyro += angle;
}

void forwardEnc(int32_t dEnc) {
  getFirstValues();
  double localVirt = 0, oldLocalVirt = 0, deltaEnc, vU;
  bool rideFlag = true;
  int32_t localTime = 0, deltaTime;
  bool normalTS = true;
  Serial.println(virtEnc[0]);
  while (rideFlag) {
    readMc();
    if (millis() > myTimer + tS) {
      Serial.println("bbb");
      Serial.println(String(encoders[0]) + " " + String(virtEnc[0]) + " " + String(localVirt) + " " + String(localTime));
      //рассчитать время, успевание программы
      deltaTime = millis() - myTimer;
      myTimer = millis();
      if (!normalTS) {
        Serial.println("это жесть, задержка " + String(deltaTime) + " мкс.");
      }
      normalTS = false;
      //прочитать значения лазеров
      readLasers();
      if ((abs(virtEnc[0] - encoders[0]) + abs(virtEnc[1] - encoders[1])) / 2 < deltaEncMax) {
        //рассчитать виртуальную точку по времени
        localTime += deltaTime;
        localVirt = tToX(double(localTime) / 1000, dEnc, vMax, aMax);
        if (oldLocalVirt * dEnc < 0 || (localVirt - oldLocalVirt) * sgn(dEnc) < vMin * double(deltaTime) / 1000) {
          localVirt = oldLocalVirt + vMin * sgn(dEnc) * double(deltaTime) / 1000;
          localTime = xToT(localVirt, dEnc, vMax, aMax) * 1000;
          localVirt = min(localVirt * sgn(dEnc), abs(dEnc)) * sgn(dEnc);
        }
        deltaEnc = (localVirt - oldLocalVirt) / cos(gyro[0] - staticGyro) / cos(gyro[1]);
        oldLocalVirt = localVirt;
        //выравняться параллельно стенкам
        vU = (staticGyro - gyro[0]) * kGyro;
        Serial.println(deltaEnc);
        virtEnc[0] += deltaEnc;
        virtEnc[1] += deltaEnc;
        virtEnc[0] += vU * double(deltaTime) / 1000;
        virtEnc[1] -= vU * double(deltaTime) / 1000;
      }
      //отослать значения целевых точек на проц моторов
      writeMc();
      //проверка, не пора ли выходить из цикла
      if (timeToGo(dEnc, vMax, aMax) * 1000 - 1 < localTime) {
        rideFlag = false;
      }
    } else {
      normalTS = true;
    }
  }
}

void writeMc() {
  // Serial.println(virtEnc[0]);
  toSerial.encMotor[0] = int32_t(virtEnc[0]);
  toSerial.encMotor[1] = int32_t(virtEnc[1]);
  // Serial.println(virtEnc[0]);
  writeSerial();
}

void fastStop() {
  virtEnc[0] = encoders[0];
  virtEnc[1] = encoders[1];
  writeMc();
}

void getFirstValues() {
  digitalWrite(13, 1);
  static bool ok = false;
  while (!ok) {
    if (myTimer + tS > millis()) {
      readLasers();
      myTimer = millis();
      writeMc();
    }
    ok = readMc();
  }
  digitalWrite(13, 0);
  while (Serial1.available() || Serial2.available()) readCam();
  myTimer = millis();
}

void readCam() {
  while (Serial1.available()) visibleH[1] = Serial1.read() - int(zeroSymb);
  while (Serial2.available()) visibleH[0] = Serial2.read() - int(zeroSymb);
}

double colorCos(int x1, int y1, int z1, int x2, int y2, int z2) {
  static double l1, l2, l3;
  l1 = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
  l2 = sqrt(x2 * x2 + y2 * y2 + z2 * z2);
  l3 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
  if (l1 * l2 == 0) {
    return 0;
  }
  return (l1 * l1 + l2 * l2 - l3 * l3) / (2 * l1 * l2);
}

void resetGyro() {
  getFirstValues();
  double vU, deltaEnc;
  int32_t deltaTime;
  bool normalTS = true, rideFlag = true;
  while (rideFlag) {
    readMc();
    if (millis() > myTimer + tS) {
      //рассчитать время, успевание программы
      deltaTime = millis() - myTimer;
      myTimer = millis();
      if (!normalTS) {
        Serial.println("это жесть, задержка " + String(deltaTime) + " мкс.");
      }
      normalTS = false;
      //прочитать значения лазеров
      readLasers();
      if ((abs(virtEnc[0] - encoders[0]) + abs(virtEnc[1] - encoders[1])) / 2 < deltaEncMax) {
        deltaEnc = double(deltaTime) / 1000 * vReset;
        virtEnc[0] += (1 - digitalRead(forwBut[0])) * deltaEnc;
        virtEnc[1] += (1 - digitalRead(forwBut[1])) * deltaEnc;
      }
      //отослать значения целевых точек на проц моторов
      writeMc();
      if (digitalRead(forwBut[0]) && digitalRead(forwBut[1])) {
        rideFlag = false;
      }
    } else {
      normalTS = true;
    }
  }
  fastStop();
  waitmillis(delayReset);
  getFirstValues();
  zeroGyro[0] += gyro[0] - staticGyro;
  rideFlag = true;
  while (rideFlag) {
    readMc();
    if (millis() > myTimer + tS) {
      //рассчитать время, успевание программы
      deltaTime = millis() - myTimer;
      myTimer = millis();
      if (!normalTS) {
        Serial.println("это жесть, задержка " + String(deltaTime) + " мкс.");
      }
      normalTS = false;
      //прочитать значения лазеров
      readLasers();
      if ((abs(virtEnc[0] - encoders[0]) + abs(virtEnc[1] - encoders[1])) / 2 < deltaEncMax) {
        deltaEnc = -double(deltaTime) / 1000 * vReset;
        vU = (staticGyro - gyro[0]) * kGyro;
        virtEnc[0] += deltaEnc;
        virtEnc[1] += deltaEnc;
        virtEnc[0] += vU * double(deltaTime) / 1000;
        virtEnc[1] -= vU * double(deltaTime) / 1000;
      }
      //отослать значения целевых точек на проц моторов
      writeMc();
      if (lasers[0] > forwardDist[0]) {
        rideFlag = false;
      }
      Serial.println(String(virtEnc[0]) + " " + String(virtEnc[1]) + " " + String(encoders[0]) + " " + String(encoders[1]));
    } else {
      normalTS = true;
    }
  }
  fastStop();
}

void waitmillis(int32_t waitTime) {
  static int32_t localTimer;
  localTimer = myTimer;
  while (localTimer + waitTime > millis()) {
    readMc();
    if (millis() > myTimer + tS) {
      readLasers();
      myTimer = millis();
      writeMc();
    }
  }
}

void throwHelps(int16_t type, int8_t orient) {
  type--;
  for (int j = 0; j < 2 * nFlash; j++) {
    for (int i = (orient + 1) * nLights / 2; i < nLights + (orient + 1) * nLights / 2; i++) {
      strip.setPixelColor(i, strip.Color(255 * ((j + 1) % 2), 0, 0));  //  Set pixel's color (in RAM)
    }
    strip.show();
    waitmillis(5000000 / 2 / nFlash);
  }
  for (int i = 0; i < helps[type]; i++) {
    Serial.println(String(i) + " " + String(type) + " " + String(helps[type]));
    if (numHelps < numHelpsMax) {
      myServo.write(srednServo * (2 - orient) / 2);
      waitmillis(500000);
      myServo.write(srednServo + 10 * orient);
      waitmillis(500000);
      numHelps++;
    }
  }
}

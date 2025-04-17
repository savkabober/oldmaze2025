#pragma once

#include <Arduino.h>

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

class MOTORS {
public:
  void attach(uint8_t ma, uint8_t mb, float k, float ki);
  int32_t returnEnc();
  void setPwm(int16_t pwm);
  void setAngle(int32_t angle);
  void regAngle(uint32_t myTime = 1);
  int32_t* interruptEnc();
  void set(int16_t pwm);
private:
  uint8_t _maPin, _mbPin, _interrupt, _directionPin;
  float _angle, _k, _ki, _motorPwm;
  int32_t _motorEnc = 0, _err;
  uint32_t _myTimer;
  float _I = 0;
};
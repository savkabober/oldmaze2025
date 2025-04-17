#include "MotorsLib.h"

void MOTORS::attach(uint8_t ma, uint8_t mb, float k, float ki) {
  _k = k;
  _ki = ki;

  _maPin = ma;

  _mbPin = mb;

  pinMode(ma, OUTPUT);
  pinMode(mb, OUTPUT);
}

void MOTORS::set(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  analogWrite(_maPin, max(pwm, 0));
  analogWrite(_mbPin, sgn(pwm) * min(pwm, 0));
}

int32_t* MOTORS::interruptEnc() {
  return &_motorEnc;
}

void MOTORS::setAngle(int32_t angle) {
  _angle = angle;
}

int32_t MOTORS::returnEnc() {
  return _motorEnc;
}

void MOTORS::regAngle(uint32_t myTime) {
  _err = _angle - _motorEnc;
  _motorPwm = _err * _k;
  _motorPwm += _I * _ki;
  _I += float(_err * (abs(_motorPwm) < 256) * int32_t(myTime)) / 1000000;
  set(_motorPwm);
}
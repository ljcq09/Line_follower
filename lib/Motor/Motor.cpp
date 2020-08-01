#include "Motor.h"
#include "mbed.h"

Motor::Motor(PinName pwm, PinName dir) : _pwm(pwm), _dir(dir) {
  _pwm.period(0.00005f); // 20 kHz
}

void Motor::motorSpeed(float speed) { _pwm.write(1.0f - speed); }

void Motor::motorDirection(bool direction) { _dir = direction; }
#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

class Motor
{
  public:
    Motor(PinName pwm, PinName dir);

    void motorSpeed(float speed);

    void motorDirection(bool direction);

  private:
    PwmOut _pwm;
    DigitalOut _dir;
};

#endif /* MOTOR_H */
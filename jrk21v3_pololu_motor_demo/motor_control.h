#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <SoftwareSerial.h>

class MotorControl
{
  private:
    SoftwareSerial *serial;
    
  public:
    MotorControl(int baudrate);
    void move(int x);
    void off();
};

#endif
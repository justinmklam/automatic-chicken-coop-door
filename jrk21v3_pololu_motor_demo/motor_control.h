#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <SoftwareSerial.h>

class MotorControl
{
  private:
    SoftwareSerial *serial;
    
  public:
    MotorControl(uint32_t baudrate);
    uint16_t move(uint16_t x);
    void off();
};

#endif
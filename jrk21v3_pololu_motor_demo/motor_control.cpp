#include "motor_control.h"

MotorControl::MotorControl(uint32_t baudrate)
{
  serial = new SoftwareSerial(PIN_RX, PIN_TX);
  serial->begin(baudrate);
}

uint16_t MotorControl::setSpeed(uint16_t x)
{
  uint16_t target = x; //only pass this ints, i tried doing math in this and the remainder error screwed something up

  // Make sure the target is within the bounds
  if (target > MAX_INPUT_CONTROL_VALUE)
  {
    target = MAX_INPUT_CONTROL_VALUE;
  }

  serial->write(POLOLU_PROTOCOL_START);                               //tells the controller we're starting to send it commands
  serial->write(POLOLU_DEVICE_NUMBER);                                //This is the pololu device # you're connected too that is found in the config utility(converted to hex). I'm using #11 in this example
  serial->write(POLOLU_PROTOCOL_TARGET_HIGH_START + (target & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
  serial->write((target >> 5) & POLOLU_PROTOCOL_END);                 //second half of the target, " " "

  return target;
}

void MotorControl::off()
{
  serial->write(POLOLU_PROTOCOL_START);
  serial->write(POLOLU_DEVICE_NUMBER);
  serial->write(POLOLU_PROTOCOL_END);
}

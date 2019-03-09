/*
* Motor control commands for Pololu jrk23v1
*/
#include "motor_control.h"

// RX, TX, plug your control line into pin 8 and connect it to the RX pin on the JRK21v3
#define PIN_RX 7
#define PIN_TX 8

// Values from user driver user guide https://www.pololu.com/docs/0J38/all
#define MAX_INPUT_CONTROL_VALUE 4095
#define POLOLU_PROTOCOL_START 0xAA
#define POLOLU_PROTOCOL_END 0x7F
#define POLOLU_PROTOCOL_TARGET_HIGH_START 0x40

#define POLOLU_DEVICE_NUMBER 0xB   // Defaults to 11, but can change through config software

MotorControl::MotorControl(uint32_t baudrate)
{
  serial = new SoftwareSerial(PIN_RX, PIN_TX);
  serial->begin(baudrate);
}

//sets the new target for the JRK21V3 controller, this uses pololu high resulution protocal
uint16_t MotorControl::move(uint16_t x)
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
  serial->write((target >> 5) & POLOLU_PROTOCOL_END);        
  
  return target;         //second half of the target, " " "
}

void MotorControl::off()
{
  serial->write(POLOLU_PROTOCOL_START);
  serial->write(POLOLU_DEVICE_NUMBER);
  serial->write(POLOLU_PROTOCOL_END);
}

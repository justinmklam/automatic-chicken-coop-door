/*
* Motor control commands for Pololu jrk23v3
*/
#include "motor_control.h"


MotorControl::MotorControl(int baudrate)
{
  serial = new SoftwareSerial(7, 8); // RX, TX, plug your control line into pin 8 and connect it to the RX pin on the JRK21v3
  serial->begin(baudrate);
}

//sets the new target for the JRK21V3 controller, this uses pololu high resulution protocal
void MotorControl::move(int x)
{
  uint16_t target = x;                        //only pass this ints, i tried doing math in this and the remainder error screwed something up
  serial->write(0xAA);                   //tells the controller we're starting to send it commands
  serial->write(0xB);                    //This is the pololu device # you're connected too that is found in the config utility(converted to hex). I'm using #11 in this example
  serial->write(0x40 + (target & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
  serial->write((target >> 5) & 0x7F);   //second half of the target, " " "
}

void MotorControl::off()
{
  serial->write(0xAA);
  serial->write(0xB);
  serial->write(0x7F);
}

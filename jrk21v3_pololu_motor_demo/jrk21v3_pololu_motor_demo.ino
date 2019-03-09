/*
* Motor control setup for pololu jrk21v3 with Arduino UNO R3, verified using linear actualtor LACT2P
*
* Pololu jrk config utility in Serial mode using UART detect baud rate interface.
* starting with the default configuration settings for LACT2P linear actuators provided on the pololu website
*
* pin 8 connected to jrk pin Rx
* jrk grnd connected to arduino ground
*/

#define BAUDRATE 9600
#define BUFFER_SIZE 5   // ie. for input = #2000

#include "motor_control.h"

MotorControl motorControl(BAUDRATE);

void setup()
{
  Serial.begin(BAUDRATE);
  Serial.println("Initialized");
  Serial.flush(); // Give reader a chance to see the output.

  Serial.println("Enter '#' and 4 digit position level (#0000-#4095)");
}

void loop()
{
  static uint8_t inByte = 0;
  static uint16_t targetPosition = 0; // target position, 0-4095 is the range of the JRK21V3 controller.
  static uint16_t actualPosition = 0;

  //stuff used for input from pc
  static char buffer[BUFFER_SIZE];

  if (Serial.available() > 0)
  {
    // read the incoming byte:
    inByte = Serial.read();
    delay(10);

    // If the marker's found, next 4 characters are the position
    if (inByte == '#')
    {
      loadBufferFromSerial(buffer);
      targetPosition = stringBufferToInt(buffer);

      actualPosition = motorControl.setSpeed(targetPosition);
      announcePos(actualPosition);
    }
    // Turn motor off if this character is received
    else if (inByte == '!')
    {
      motorControl.off();

      Serial.println("Motor off");
      Serial.flush();
    }
  }
}

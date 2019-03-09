/*
* Motor control setup for pololu jrk21v3 with Arduino UNO R3, verified using linear actualtor LACT2P
*
* Pololu jrk config utility in Serial mode using UART detect baud rate interface. 
* starting with the default configuration settings for LACT2P linear actuators provided on the pololu website
*
* pin 8 connected to jrk pin Rx
* jrk grnd connected to arduino ground
*/

#include <SoftwareSerial.h>
SoftwareSerial mySerial(7, 8); // RX, TX, plug your control line into pin 8 and connect it to the RX pin on the JRK21v3

int myTarget = 0; // target position, 0-4095 is the range of the JRK21V3 controller.

//stuff used for input from pc
char buffer[5];
byte inByte = 0;

void loadBufferFromSerial(char *buffer)
{
  int pointer = 0;

  while (pointer < 4)
  {                                  // accumulate 4 chars
    buffer[pointer] = Serial.read(); // store in the buffer
    pointer++;                       // move the pointer forward by 1
  }

  Serial.flush();
}

int stringBufferToTargetInt(char buffer[])
{
  int target = 0;

  target = (buffer[0] - 48) * 1000 + (buffer[1] - 48) * 100 + (buffer[2] - 48) * 10 + (buffer[3] - 48);

  //makes sure the target is within the bounds
  if (target < 0)
  {
    target = 0;
  }
  else if (target > 4095)
  {
    target = 4095;
  }
  return target;
}

void setup()
{
  mySerial.begin(9600);
  Serial.begin(9600);
  Serial.println("Initialized");
  Serial.flush(); // Give reader a chance to see the output.

  int myTarget = 0; //the health level at any point in time
  Serial.println("Enter '#' and 4 digit position level (#0000-#4095)");
}

void loop()
{

  if (Serial.available() > 0)
  {
    // read the incoming byte:

    inByte = Serial.read();
    delay(10);

    // If the marker's found, next 4 characters are the position
    if (inByte == '#')
    {
      loadBufferFromSerial(buffer);
      myTarget = stringBufferToTargetInt(buffer);

      Move(myTarget);
      announcePos(myTarget);
    }
    // Turn motor off if this character is received
    else if (inByte == '!')
    {
      MotorOff();
    }
  }
}

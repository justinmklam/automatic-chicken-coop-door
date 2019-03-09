/*
* Motor control commands for Pololu jrk23v1
*/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <SoftwareSerial.h>

// RX, TX, plug your control line into pin 8 and connect it to the RX pin on the JRK21v3
#define PIN_RX 7
#define PIN_TX 8

// Values from user driver user guide https://www.pololu.com/docs/0J38/all
#define MAX_INPUT_CONTROL_VALUE 4095
#define POLOLU_PROTOCOL_START 0xAA
#define POLOLU_PROTOCOL_END 0x7F
#define POLOLU_PROTOCOL_TARGET_HIGH_START 0x40

#define POLOLU_DEVICE_NUMBER 0xB   // Defaults to 11, but can change through config software

class MotorControl
{
  private:
    SoftwareSerial *serial;
    
  public:
    MotorControl(uint32_t baudrate);

    // Sets the new target speed for the JRK21V3 controller, this uses pololu high resulution protocol
    uint16_t setSpeed(uint16_t x);
    void off();
};

#endif
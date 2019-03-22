#ifndef MOTOR_TB6612_H
#define MOTOR_TB6612_H

#include <SparkFun_TB6612.h>

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 7
#define AIN2 8
#define PWMA 9
#define STBY 6

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

class MotorTb6612
{
private:
    Motor *motor;

public:
    MotorTb6612();
    void up();
    void down();
    void brake();
    void standby();
};

#endif
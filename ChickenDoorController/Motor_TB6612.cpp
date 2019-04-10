#include "Motor_TB6612.h"

MotorTb6612::MotorTb6612()
{
  motor = new Motor(AIN1, AIN2, PWMA, offsetA, STBY);
}

void MotorTb6612::up()
{
  motor->drive(255);
}

void MotorTb6612::down()
{
  motor->drive(-255);
}

void MotorTb6612::brake()
{
  motor->brake();
}

void MotorTb6612::standby()
{
  motor->standby();
}
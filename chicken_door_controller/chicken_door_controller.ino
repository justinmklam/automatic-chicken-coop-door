#include <LowPower.h>
#include "ssd1306.h"
#include "rtc_ds3231.h"
#include "motor_tb6612.h"
// #include <Arduino_FreeRTOS.h>
#include "Task.h"
#include "TaskScheduler.h"

Rtc rtc;      //we are using the DS3231 RTC
Ssd1306 display;
MotorTb6612 motor;

// RTC stuff
const uint8_t alarmNumber = 1;
const uint8_t wakePin = 2;    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
const uint8_t ledPin = 13;    //use arduino on-board led for indicating sleep or wakeup status

const uint8_t buttonPinLeft = 10;
const uint8_t buttonPinMiddle = 3;
const uint8_t buttonPinRight = 12;

// Pullup resistor used
const uint8_t defaultButtonState = HIGH;

const uint16_t intervalDisplayRefresh = 1000;
const uint16_t intervalInactive = 10e3;

//------------------------------------------------------------

void setup() {

  //switch-on the on-board led for 1 second for indicating that the sketch is ok and running
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(1000);

  // display.begin();

  // //Set alarm1 every day at 18:33
  rtc.set_alarm(alarmNumber, 21, 06, 0);

  // display.println(rtc.get_datestamp_str());
  // display.print(rtc.get_timestamp_str());

  // delay(1000);
}

//------------------------------------------------------------

class UpdateDisplay : public TimedTask
{
public:
  UpdateDisplay();
  virtual void run(uint32_t now);

private:
  char bufferTimestamp[10];
};

UpdateDisplay::UpdateDisplay() : TimedTask(millis())
{
  display.begin();
  display.println("Hello World");
}

void UpdateDisplay::run(uint32_t now)
{
  display.clear();
  rtc.get_timestamp_str(bufferTimestamp);
  display.println(bufferTimestamp);
  display.print("hello");
  display.show();

  incRunTime(1000);
}

//------------------------------------------------------------


class SleepMode : public TriggeredTask
{
public:
  SleepMode();
  virtual void run(uint32_t now);

private:
  bool sleepState;
  void powerDown();
  void wakeUp();
  static void wakeUpInterruptHandler();

};

SleepMode::SleepMode() : TriggeredTask()
{
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(wakePin, INPUT);
  rtc.begin();
}

void SleepMode::run(uint32_t now)
{
  powerDown();

  wakeUp();
  sleepState = false;

  resetRunnable();
}

void SleepMode::powerDown() {
  display.sleep();

  attachInterrupt(digitalPinToInterrupt(wakePin), wakeUpInterruptHandler, LOW);
  attachInterrupt(digitalPinToInterrupt(buttonPinMiddle), wakeUpInterruptHandler, CHANGE);

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void SleepMode::wakeUp() {
  display.wake();

  detachInterrupt(digitalPinToInterrupt(wakePin));                                    //execution resumes from here after wake-up
  detachInterrupt(digitalPinToInterrupt(buttonPinMiddle));
}

//-------------------------------------------------

void SleepMode::wakeUpInterruptHandler()        // here the interrupt is handled after wakeup
{
}


//------------------------------------------------------------

class UserInput : public TimedTask
{
public:
  UserInput(SleepMode *_ptrSleep);
  virtual void run(uint32_t now);

private:
  uint8_t buttonStateLeft = HIGH;
  uint8_t buttonStateMid = HIGH;
  uint8_t buttonStateRight = HIGH;

  SleepMode *ptrSleep;
};

UserInput::UserInput(SleepMode *_ptrSleep) : TimedTask(millis()), ptrSleep(_ptrSleep)
{
    pinMode(buttonPinLeft, INPUT);
    pinMode(buttonPinMiddle, INPUT);
    pinMode(buttonPinRight, INPUT);
}

void UserInput::run(uint32_t now)
{
  buttonStateLeft = digitalRead(buttonPinLeft);
  buttonStateMid = digitalRead(buttonPinMiddle);
  buttonStateRight = digitalRead(buttonPinRight);

  if (buttonStateMid == LOW) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);

    ptrSleep->setRunnable();
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }

  if (buttonStateLeft == LOW) {
    motor.up();
  }
  else if (buttonStateRight == LOW) {
    motor.down();
  }
  else {
    motor.brake();
  }

  incRunTime(100);
}

//------------------------------------------------------------

void loop() {

  UpdateDisplay updateDisplay;
  SleepMode sleepMode;
  UserInput userInput(&sleepMode);

  // Order determines task priority
  Task *tasks[] = {
    &userInput,
    &updateDisplay,
    &sleepMode
  };

  TaskScheduler scheduler(tasks, NUM_TASKS(tasks));

  scheduler.runTasks();

}

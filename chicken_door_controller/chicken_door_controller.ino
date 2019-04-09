#include <LowPower.h>
#include <EEPROM.h>

#include "Task.h"
#include "TaskScheduler.h"

#include "ssd1306.h"
#include "rtc_ds3231.h"
#include "motor_tb6612.h"

Rtc rtc;      //we are using the DS3231 RTC
Ssd1306 display;
MotorTb6612 motor;

const uint8_t alarmNumber = 1;  // two alarms available, but we just need one
const uint8_t alarmPin = 2;    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
const uint8_t ledPin = 13;    //use arduino on-board led for indicating sleep or wakeup status

const uint8_t buttonPinLeft = 10;
const uint8_t buttonPinMiddle = 3;
const uint8_t buttonPinRight = 12;

// Pullup resistor used
const uint8_t defaultButtonState = HIGH;

const uint16_t intervalInactive = 100;

/*************************************************************************/

class UpdateDisplay : public TimedTask
{
public:
  UpdateDisplay();
  virtual void run(uint32_t now);
  void pauseMainScreen();
  void resumeMainScreen();

private:
  bool pauseUpdate = false;
  const uint16_t intervalDisplayRefresh = 1000;

  char bufferTimestamp[10];
};

UpdateDisplay::UpdateDisplay() : TimedTask(millis())
{
  display.begin();
  display.println("Hello World");
}

void UpdateDisplay::pauseMainScreen()
{
  pauseUpdate = true;
}

void UpdateDisplay::resumeMainScreen()
{
  pauseUpdate = false;
}

void UpdateDisplay::run(uint32_t now)
{
  if (!pauseUpdate)
  {
    display.clear();
    rtc.get_timestamp_str(bufferTimestamp);
    display.println(bufferTimestamp);
    display.print("hello");
    display.show();
  }

  incRunTime(intervalDisplayRefresh);
}

/*************************************************************************/

class SleepMode : public TriggeredTask
{
public:
  SleepMode();
  void enableSleepFromAlarm();
  void enableSleepFromUserInactivity();
  virtual void run(uint32_t now);

private:
  bool sleepFromAlarm = false;
  bool sleepFromUserInactivity = false;

  void powerDown(uint8_t pinNumber);
  void wakeUp(uint8_t pinNumber);
  static void wakeUpInterruptHandler();

};

SleepMode::SleepMode() : TriggeredTask()
{
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(alarmPin, INPUT);
  rtc.begin();
}

void SleepMode::run(uint32_t now)
{
  if (sleepFromAlarm) {
    powerDown(alarmPin);
    wakeUp(alarmPin);

    sleepFromAlarm = false;
  }

  else if (sleepFromUserInactivity) {
    powerDown(buttonPinMiddle);
    wakeUp(buttonPinMiddle);

    sleepFromUserInactivity = false;
  }

  resetRunnable();
}

void SleepMode::enableSleepFromAlarm() {
  sleepFromAlarm = true;
}

void SleepMode::enableSleepFromUserInactivity() {
  sleepFromUserInactivity = true;
}

void SleepMode::powerDown(uint8_t pinNumber) {
  display.sleep();

  // TODO: Not sure if this needs to be LOW for RTC alarm
  attachInterrupt(
    digitalPinToInterrupt(pinNumber),
    wakeUpInterruptHandler,
    CHANGE
  );

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void SleepMode::wakeUp(uint8_t pinNumber) {
  display.wake();

  detachInterrupt(digitalPinToInterrupt(pinNumber));
}

void SleepMode::wakeUpInterruptHandler()        // here the interrupt is handled after wakeup
{
}

/*************************************************************************/

class UserInput : public TimedTask
{
public:
  UserInput(SleepMode *_ptrSleep, UpdateDisplay *_ptrDisplay);
  virtual void run(uint32_t now);

private:

  void checkInactivity(bool isButtonPressed);
  uint8_t userState = 0;
  bool doorStateOpen = false;

  int eepromAddr = 0;
  uint32_t distance = EEPROM.read(eepromAddr);

  bool calibrationMode = false;
  bool isButtonPressed = false;

  uint8_t buttonStateLeft = defaultButtonState;
  uint8_t buttonStateMid = defaultButtonState;
  uint8_t buttonStateRight = defaultButtonState;

  uint32_t previousMillis;
  uint32_t inactivityCounter = 0;

  const uint16_t refreshInterval = 100;
  // const uint16_t inactivityInterval =

  SleepMode *ptrSleep;
  UpdateDisplay *ptrDisplay;
};

UserInput::UserInput(SleepMode *_ptrSleep, UpdateDisplay *_ptrDisplay) :
  TimedTask(millis()),
  ptrSleep(_ptrSleep),
  ptrDisplay(_ptrDisplay)
{
    pinMode(buttonPinLeft, INPUT);
    pinMode(buttonPinMiddle, INPUT);
    pinMode(buttonPinRight, INPUT);

    previousMillis = millis();
}

void UserInput::run(uint32_t now)
{
  buttonStateLeft = digitalRead(buttonPinLeft);
  buttonStateMid = digitalRead(buttonPinMiddle);
  buttonStateRight = digitalRead(buttonPinRight);

  isButtonPressed = buttonStateLeft == LOW || buttonStateMid == LOW || buttonStateRight == LOW;

  if (calibrationMode == true) {
    switch(userState) {
      default:
        display.clear();
        display.println("Door up");
        display.print("Next >");
        display.show();

        userState++;

        break;

      case 1:
        buttonStateLeft = digitalRead(buttonPinLeft);
        buttonStateMid = digitalRead(buttonPinMiddle);
        buttonStateRight = digitalRead(buttonPinRight);

        if (buttonStateLeft == LOW) {
          motor.up();
        }
        else if (buttonStateRight == LOW) {
          motor.down();
        }
        else {
          motor.brake();
        }

        if (buttonStateMid == LOW) {
          distance = 0;

          display.clear();
          display.println("Door down");
          display.print("Next >");
          display.show();

          userState++;
        }

        break;

      case 2:
        buttonStateLeft = digitalRead(buttonPinLeft);
        buttonStateMid = digitalRead(buttonPinMiddle);
        buttonStateRight = digitalRead(buttonPinRight);

        if (buttonStateLeft == LOW) {
          motor.up();
          distance++;
        }
        else if (buttonStateRight == LOW) {
          motor.down();
          if (distance > 0) {
            distance--;
          }
          else {
            distance = 0;
          }
        }
        else {
          motor.brake();
        }

        if (buttonStateMid == LOW) {
          display.clear();
          display.println("Done");
          display.print(distance);
          display.show();

          EEPROM.write(eepromAddr, distance);

          userState++;
        }

        break;

      case 3:
        userState = 0;
        calibrationMode = false;
        ptrDisplay->resumeMainScreen();

        break;
    }
  }
  else {
    if (buttonStateMid == LOW) {
      // turn LED on:

      ptrDisplay->pauseMainScreen();
      calibrationMode = true;
      digitalWrite(ledPin, HIGH);

      // ptrSleep->setRunnable();
    } else {
      // turn LED off:
      digitalWrite(ledPin, LOW);
    }

    if (buttonStateLeft == LOW && !doorStateOpen) {
      // Open door
      for (int i=0; i < distance; i++){
        motor.up();
        delay(refreshInterval);
      }
      motor.brake();
      doorStateOpen = true;
    }
    else if (buttonStateRight == LOW && doorStateOpen) {
      // Close door
      for (int i=0; i < distance; i++){
        motor.down();
        delay(refreshInterval);
      }
      motor.brake();
      doorStateOpen = false;
    }
  }

  checkInactivity(isButtonPressed);
  incRunTime(refreshInterval);
}

void UserInput::checkInactivity(bool isButtonPressed)
{
  if (isButtonPressed) {
  // previousMillis = millis();
  inactivityCounter = 0;
  }
  else {
    inactivityCounter++;

    if (inactivityCounter >= intervalInactive) {
      // previousMillis += intervalInactive;

      inactivityCounter = 0;

      ptrSleep->enableSleepFromUserInactivity();
      ptrSleep->setRunnable();
    }
  }
}

/*************************************************************************/

void setup() {

  //switch-on the on-board led for 1 second for indicating that the sketch is ok and running
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(1000);

  rtc.set_alarm(alarmNumber, 12, 39, 0);
}

void loop() {

  UpdateDisplay updateDisplay;
  SleepMode sleepMode;
  // DoorCalibration doorCalibration;
  UserInput userInput(&sleepMode, &updateDisplay);

  // Order determines task priority
  Task *tasks[] = {
    &userInput,
    &updateDisplay,
    &sleepMode,
    // &doorCalibration
  };

  TaskScheduler scheduler(tasks, NUM_TASKS(tasks));

  scheduler.runTasks();

}

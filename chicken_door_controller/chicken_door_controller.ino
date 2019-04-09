#include <LowPower.h>

#include "EepromExtended.h"
#include "Task.h"
#include "TaskScheduler.h"

#include "ssd1306.h"
#include "rtc_ds3231.h"
#include "motor_tb6612.h"

Rtc rtc;      //we are using the DS3231 RTC
Ssd1306 display;
MotorTb6612 motor;

const uint8_t ALARM_NUMBER = 1;  // two alarms available, but we just need one
const uint8_t ALARM_PIN = 2;    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
const uint8_t LED_PIN = 13;    //use arduino on-board led for indicating sleep or wakeup status

const uint8_t BUTTON_PIN_LEFT = 10;
const uint8_t BUTTON_PIN_MIDDLE = 3;
const uint8_t BUTTON_PIN_RIGHT = 12;

// Pullup resistor used
const uint8_t BUTTON_STATE_DEFAULT = HIGH;
const uint8_t BUTTON_STATE_PRESSED = LOW;

const uint8_t EEPROM_ADDR_DOOR_DISTANCE = 1;

uint32_t DOOR_OPEN_CLOSE_DISTANCE = EEPROMRead32bit(EEPROM_ADDR_DOOR_DISTANCE);
const uint16_t REFRESH_INTERVAL_MS = 100;

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
  pinMode(ALARM_PIN, INPUT);
  rtc.begin();
}

void SleepMode::run(uint32_t now)
{
  if (sleepFromAlarm) {
    powerDown(ALARM_PIN);
    wakeUp(ALARM_PIN);

    sleepFromAlarm = false;
  }

  else if (sleepFromUserInactivity) {
    powerDown(BUTTON_PIN_MIDDLE);
    wakeUp(BUTTON_PIN_MIDDLE);

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

class DoorControl : public TriggeredTask
{
public:
  DoorControl();
  void open();
  void close();
  void setOpen();
  void setClose();
  virtual void run(uint32_t now);
private:
  bool stateDoorOpen = false;
  bool setDoorOpen = false;
  bool setDoorClose = false;
};

DoorControl::DoorControl() : TriggeredTask()
{
}

void DoorControl::run(uint32_t now)
{
  if (setDoorOpen) {
    open();
    setDoorOpen = false;
  }
  else if (setDoorClose) {
    close();
    setDoorClose = false;
  }

  resetRunnable();
}

void DoorControl::setOpen()
{
  setDoorOpen = true;
}

void DoorControl::setClose()
{
  setDoorClose = true;
}

void DoorControl::open()
{
  if (stateDoorOpen) {
    return;
  }
  for (int i=0; i < DOOR_OPEN_CLOSE_DISTANCE; i++){
    motor.up();
    delay(REFRESH_INTERVAL_MS);
  }
  motor.brake();
  stateDoorOpen = true;
}

void DoorControl::close()
{
  if (!stateDoorOpen) {
    return;
  }

  for (int i=0; i < DOOR_OPEN_CLOSE_DISTANCE; i++){
    motor.down();
    delay(REFRESH_INTERVAL_MS);
  }
  motor.brake();
  stateDoorOpen = false;
}

/*************************************************************************/

class UserInput : public TimedTask
{
public:
  UserInput(SleepMode *_ptrSleep, UpdateDisplay *_ptrDisplay, DoorControl *_ptrDoorControl);
  virtual void run(uint32_t now);

private:

  void checkInactivity(bool isButtonPressed);
  void calibrateDoor();

  uint8_t userState = 0;
  bool doorStateOpen = false;

  bool calibrationMode = false;
  bool isButtonPressed = false;

  uint8_t buttonStateLeft = BUTTON_STATE_DEFAULT;
  uint8_t buttonStateMid = BUTTON_STATE_DEFAULT;
  uint8_t buttonStateRight = BUTTON_STATE_DEFAULT;

  uint32_t previousMillis;
  uint32_t inactivityCounter = 0;
  uint32_t calibrationButtonPressedCounter = 0;

  const uint16_t calibrationButtonPressedThreshold = 20;
  const uint16_t intervalInactive = 100;

  // const uint16_t inactivityInterval =

  SleepMode *ptrSleep;
  UpdateDisplay *ptrDisplay;
  DoorControl *ptrDoorControl;
};

UserInput::UserInput(SleepMode *_ptrSleep, UpdateDisplay *_ptrDisplay, DoorControl *_ptrDoorControl) :
  TimedTask(millis()),
  ptrSleep(_ptrSleep),
  ptrDisplay(_ptrDisplay),
  ptrDoorControl(_ptrDoorControl)
{
    pinMode(BUTTON_PIN_LEFT, INPUT);
    pinMode(BUTTON_PIN_MIDDLE, INPUT);
    pinMode(BUTTON_PIN_RIGHT, INPUT);

    previousMillis = millis();
}

void UserInput::run(uint32_t now)
{
  buttonStateLeft = digitalRead(BUTTON_PIN_LEFT);
  buttonStateMid = digitalRead(BUTTON_PIN_MIDDLE);
  buttonStateRight = digitalRead(BUTTON_PIN_RIGHT);

  isButtonPressed = buttonStateLeft == BUTTON_STATE_PRESSED || buttonStateMid == BUTTON_STATE_PRESSED || buttonStateRight == BUTTON_STATE_PRESSED;

  if (calibrationMode == true) {
    calibrateDoor();
  }
  else {
    if (buttonStateMid == BUTTON_STATE_PRESSED) {
      calibrationButtonPressedCounter++;

      if (calibrationButtonPressedCounter >= calibrationButtonPressedThreshold) {
        ptrDisplay->pauseMainScreen();

        calibrationButtonPressedCounter = 0;
        calibrationMode = true;

        display.clear();
        display.println("Calibrate");
        display.show();
        delay(2000);
      }

      // ptrSleep->setRunnable();
    } else {
      // turn LED off:
    }

    if (buttonStateLeft == BUTTON_STATE_PRESSED) {
      ptrDoorControl->setOpen();
      ptrDoorControl->setRunnable();
    }
    else if (buttonStateRight == BUTTON_STATE_PRESSED) {
      ptrDoorControl->setClose();
      ptrDoorControl->setRunnable();
    }
  }

  checkInactivity(isButtonPressed);
  incRunTime(REFRESH_INTERVAL_MS);
}

void UserInput::calibrateDoor()
{
  switch(userState) {
    default:
      display.clear();
      display.println("Move up");
      display.print("Next >");
      display.show();

      userState++;

      break;

    case 1:
      buttonStateLeft = digitalRead(BUTTON_PIN_LEFT);
      buttonStateMid = digitalRead(BUTTON_PIN_MIDDLE);
      buttonStateRight = digitalRead(BUTTON_PIN_RIGHT);

      if (buttonStateLeft == BUTTON_STATE_PRESSED) {
        motor.up();
      }
      else if (buttonStateRight == BUTTON_STATE_PRESSED) {
        motor.down();
      }
      else {
        motor.brake();
      }

      if (buttonStateMid == BUTTON_STATE_PRESSED) {
        DOOR_OPEN_CLOSE_DISTANCE = 0;

        display.clear();
        display.println("Move down");
        display.print("Next >");
        display.show();

        userState++;
      }

      break;

    case 2:
      buttonStateLeft = digitalRead(BUTTON_PIN_LEFT);
      buttonStateMid = digitalRead(BUTTON_PIN_MIDDLE);
      buttonStateRight = digitalRead(BUTTON_PIN_RIGHT);

      if (buttonStateLeft == BUTTON_STATE_PRESSED) {
        motor.up();
        DOOR_OPEN_CLOSE_DISTANCE++;
      }
      else if (buttonStateRight == BUTTON_STATE_PRESSED) {
        if (DOOR_OPEN_CLOSE_DISTANCE > 0) {
          motor.down();
          DOOR_OPEN_CLOSE_DISTANCE--;
        }
        else {
          motor.brake();
        }
      }
      else {
        motor.brake();
      }

      if (buttonStateMid == BUTTON_STATE_PRESSED) {
        display.clear();
        display.println("Complete");
        display.print(DOOR_OPEN_CLOSE_DISTANCE);
        display.show();

        EEPROMWrite32bit(EEPROM_ADDR_DOOR_DISTANCE, DOOR_OPEN_CLOSE_DISTANCE);

        delay(1000);

        doorStateOpen = false;
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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);

  rtc.set_alarm(ALARM_NUMBER, 12, 39, 0);
}

void loop() {

  UpdateDisplay updateDisplay;
  SleepMode sleepMode;
  DoorControl doorControl;
  // DoorCalibration doorCalibration;
  UserInput userInput(&sleepMode, &updateDisplay, &doorControl);

  // Order determines task priority
  Task *tasks[] = {
    &userInput,
    &updateDisplay,
    &sleepMode,
    &doorControl
    // &doorCalibration
  };

  TaskScheduler scheduler(tasks, NUM_TASKS(tasks));

  scheduler.runTasks();

}

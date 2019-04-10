#include <LowPower.h>

#include "EepromExtended.h"
#include "Task.h"
#include "TaskScheduler.h"
#include "SunriseSunsetTimes.h"
#include "Logger.h"

#include "Display_SSD1306.h"
#include "Rtc_DS3231.h"
#include "Motor_TB6612.h"

Rtc rtc;
Ssd1306 display;
MotorTb6612 motor;

const uint8_t ALARM_NUMBER = 1; // two alarms available, but we just need one
const uint8_t ALARM_PIN = 2;    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
const uint8_t LED_PIN = 13;     //use arduino on-board led for indicating sleep or wakeup status

const uint8_t BUTTON_PIN_LEFT = 10;
const uint8_t BUTTON_PIN_MIDDLE = 3;
const uint8_t BUTTON_PIN_RIGHT = 12;

// Pullup resistor used
const uint8_t BUTTON_STATE_DEFAULT = HIGH;
const uint8_t BUTTON_STATE_PRESSED = LOW;

const uint8_t EEPROM_ADDR_DOOR_STATUS = 0;
const uint8_t EEPROM_ADDR_DOOR_DISTANCE = 1;

uint32_t DOOR_OPEN_CLOSE_DISTANCE = EEPROMRead32bit(EEPROM_ADDR_DOOR_DISTANCE);
const uint16_t REFRESH_INTERVAL_MS = 100;

int NEXT_ALARM_HOUR = 0;
int NEXT_ALARM_MINUTE = 0;

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

  // TODO: This buffer is currently causing weird artifacts to show. Reducing to 5
  // causes it to crash (but artifacts are gone), but >6 is functional. Weird.
  char bufferTimestamp[10];
};

UpdateDisplay::UpdateDisplay() : TimedTask(millis())
{
  loggerln("UpdateDisplay: Constructed");
  display.begin();
}

void UpdateDisplay::pauseMainScreen()
{
  loggerln("UpdateDisplay.pauseMainScreen");
  pauseUpdate = true;
}

void UpdateDisplay::resumeMainScreen()
{
  loggerln("UpdateDisplay.resumeMainScreen");
  pauseUpdate = false;
}

void UpdateDisplay::run(uint32_t now)
{
  if (!pauseUpdate)
  {
    display.clear();
    rtc.get_timestamp_str(bufferTimestamp);
    display.println(bufferTimestamp);
    display.print(NEXT_ALARM_HOUR);
    display.print(":");
    display.print(NEXT_ALARM_MINUTE);
    display.print(" ALRM");
    display.show();
  }

  incRunTime(intervalDisplayRefresh);
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
  bool stateDoorOpen = EEPROMRead8bit(EEPROM_ADDR_DOOR_STATUS);
  bool setDoorOpen = false;
  bool setDoorClose = false;
};

DoorControl::DoorControl() : TriggeredTask()
{
  loggerln("DoorControl: Constructed");
}

void DoorControl::run(uint32_t now)
{
  if (setDoorOpen)
  {
    open();
    setDoorOpen = false;
  }
  else if (setDoorClose)
  {
    close();
    setDoorClose = false;
  }

  resetRunnable();
}

void DoorControl::setOpen()
{
  loggerln("DoorControl.setOpen");
  setDoorOpen = true;
}

void DoorControl::setClose()
{
  loggerln("DoorControl.setClose");
  setDoorClose = true;
}

void DoorControl::open()
{
  if (stateDoorOpen)
  {
    loggerln("DoorControl: Already open");
    return;
  }

  for (int i = 0; i < DOOR_OPEN_CLOSE_DISTANCE; i++)
  {
    motor.up();
    delay(REFRESH_INTERVAL_MS);
  }

  motor.brake();
  stateDoorOpen = true;
  EEPROMWrite8bit(EEPROM_ADDR_DOOR_STATUS, stateDoorOpen);
  loggerln("DoorControl: Opened");
}

void DoorControl::close()
{
  if (!stateDoorOpen)
  {
    loggerln("DoorControl: Already closed");
    return;
  }

  for (int i = 0; i < DOOR_OPEN_CLOSE_DISTANCE; i++)
  {
    motor.down();
    delay(REFRESH_INTERVAL_MS);
  }

  motor.brake();
  stateDoorOpen = false;
  EEPROMWrite8bit(EEPROM_ADDR_DOOR_STATUS, stateDoorOpen);
  loggerln("DoorControl: Closed");
}

/*************************************************************************/

class SleepMode : public TriggeredTask
{
public:
  SleepMode(DoorControl *_ptrDoorControl);
  void enableSleepFromAlarm(uint8_t hour, uint8_t minute);
  void enableSleepFromUserInactivity();
  virtual void run(uint32_t now);

private:
  bool isAlarmTriggered = false;
  bool enableSleep = false;
  bool sleepFromAlarm = false;
  bool sleepFromUserInactivity = false;

  uint8_t alarmHour, alarmMinute;

  bool controlDoorFromAlarm();
  void attachPinInterrupt(uint8_t pinNumber);
  void detachPinInterrupt(uint8_t pinNumber);
  void powerDown();
  void wakeUp();
  static void wakeUpInterruptHandler();

  DoorControl *ptrDoorControl;
};

SleepMode::SleepMode(DoorControl *_ptrDoorControl) : TriggeredTask(),
                                                     ptrDoorControl(_ptrDoorControl)

{
  loggerln("SleepMode: Constructed");
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(ALARM_PIN, INPUT);
  rtc.begin();
}

void SleepMode::run(uint32_t now)
{
  // NO LOG STATEMENTS ALLOWED HERE

  if (sleepFromAlarm)
  {
    attachPinInterrupt(ALARM_PIN);

    enableSleep = true;
  }
  if (sleepFromUserInactivity)
  {
    attachPinInterrupt(BUTTON_PIN_MIDDLE);

    enableSleep = true;
  }

  if (enableSleep)
  {
    powerDown();
    wakeUp(); // Resumes here after sleep

    if (sleepFromAlarm)
    {
      isAlarmTriggered = controlDoorFromAlarm();

      if (isAlarmTriggered)
      {
        rtc.clear_alarm(ALARM_NUMBER);
        detachPinInterrupt(ALARM_PIN);

        sleepFromAlarm = false;
      }
    }
    if (sleepFromUserInactivity)
    {
      detachPinInterrupt(BUTTON_PIN_MIDDLE);
      sleepFromUserInactivity = false;
    }
  }

  resetRunnable();
}

void SleepMode::enableSleepFromAlarm(uint8_t hour, uint8_t minute)
{
  logger("SleepMode.enableSleepFromAlarm: ");
  logger(hour);
  logger(":");
  loggerln(minute);

  alarmHour = hour;
  alarmMinute = minute;
  NEXT_ALARM_HOUR = hour;
  NEXT_ALARM_MINUTE = minute;
  sleepFromAlarm = true;

  rtc.set_alarm(ALARM_NUMBER, hour, minute, 0);
}

bool SleepMode::controlDoorFromAlarm()
{
  // NO LOG STATEMENTS ALLOWED HERE

  bool alarmTriggered = false;

  // Check if woken up by alarm
  if (rtc.now().hour() == alarmHour && rtc.now().minute() == alarmMinute)
  {
    alarmTriggered = true;
    // Before noon is morning, so open door
    if (rtc.now().hour() < 12)
    {
      ptrDoorControl->setOpen();
      ptrDoorControl->setRunnable();
    }
    else
    {
      ptrDoorControl->setClose();
      ptrDoorControl->setRunnable();
    }
  }
  return alarmTriggered;
}

void SleepMode::enableSleepFromUserInactivity()
{
  loggerln("SleepMode.enableSleepFromUserInactivity");

  sleepFromUserInactivity = true;
}

void SleepMode::attachPinInterrupt(uint8_t pinNumber)
{
  logger("SleepMode.attachPinInterrupt: ");
  loggerln(pinNumber);

  attachInterrupt(
      digitalPinToInterrupt(pinNumber),
      wakeUpInterruptHandler,
      CHANGE);
}

void SleepMode::detachPinInterrupt(uint8_t pinNumber)
{
  logger("SleepMode.detachPinInterrupt: ");
  loggerln(pinNumber);

  detachInterrupt(digitalPinToInterrupt(pinNumber));
}

void SleepMode::powerDown()
{
  loggerln("SleepMode.powerDown");

  display.sleep();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void SleepMode::wakeUp()
{
  display.wake();
}

void SleepMode::wakeUpInterruptHandler() // here the interrupt is handled after wakeup
{
}

/*************************************************************************/

class SunriseSunsetAlarms : public TimedTask
{
public:
  SunriseSunsetAlarms(SleepMode *_ptrSleep);
  void setAlarm();
  virtual void run(uint32_t now);

private:
  const uint32_t alarmUpdateIntervalMs = 5000;
  const uint8_t sunriseBufferHour = 0;
  const uint8_t sunsetBufferHour = 2;
  SleepMode *ptrSleep;
};

SunriseSunsetAlarms::SunriseSunsetAlarms(SleepMode *_ptrSleep)
    : TimedTask(millis()),
      ptrSleep(_ptrSleep)
{
  loggerln("SunriseSunsetAlarms: Constructed");
}

void SunriseSunsetAlarms::run(uint32_t now)
{
  setAlarm();
  incRunTime(alarmUpdateIntervalMs);
}

void SunriseSunsetAlarms::setAlarm()
{
  // loggerln("SunriseSunsetAlarms.setAlarm");

  // TODO: Handle daylight savings time (since sunrise/sunset times do)
  uint8_t currentMonthIndex = rtc.now().month() - 1;
  uint8_t currentHour = rtc.now().hour();
  uint8_t currentMinute = rtc.now().minute();
  uint8_t alarmHour, alarmMinute;

  bool isBeforeSunrise = (currentHour <= SUNRISE_TIMES[currentMonthIndex].hour) && (currentMinute <= SUNRISE_TIMES[currentMonthIndex].minute);
  bool isAfterSunset = (currentHour >= SUNSET_TIMES[currentMonthIndex].hour) && (currentMinute >= SUNSET_TIMES[currentMonthIndex].minute);

  if (isBeforeSunrise || isAfterSunset)
  {
    alarmHour = SUNRISE_TIMES[currentMonthIndex].hour - sunriseBufferHour;
    alarmMinute = SUNRISE_TIMES[currentMonthIndex].minute;
  }
  else
  {
    alarmHour = SUNSET_TIMES[currentMonthIndex].hour + sunsetBufferHour;
    alarmMinute = SUNSET_TIMES[currentMonthIndex].minute;
  }

  ptrSleep->enableSleepFromAlarm(alarmHour, alarmMinute);
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

UserInput::UserInput(SleepMode *_ptrSleep, UpdateDisplay *_ptrDisplay, DoorControl *_ptrDoorControl) : TimedTask(millis()),
                                                                                                       ptrSleep(_ptrSleep),
                                                                                                       ptrDisplay(_ptrDisplay),
                                                                                                       ptrDoorControl(_ptrDoorControl)
{
  loggerln("UserInput: Constructed");

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

  if (calibrationMode == true)
  {
    calibrateDoor();
  }
  else
  {
    if (buttonStateMid == BUTTON_STATE_PRESSED)
    {
      calibrationButtonPressedCounter++;

      if (calibrationButtonPressedCounter >= calibrationButtonPressedThreshold)
      {
        ptrDisplay->pauseMainScreen();

        calibrationButtonPressedCounter = 0;
        calibrationMode = true;

        display.clear();
        display.println("Calibrate");
        display.show();
        delay(2000);
      }

      // ptrSleep->setRunnable();
    }
    else
    {
      // turn LED off:
    }

    if (buttonStateLeft == BUTTON_STATE_PRESSED)
    {
      ptrDoorControl->setOpen();
      ptrDoorControl->setRunnable();
    }
    else if (buttonStateRight == BUTTON_STATE_PRESSED)
    {
      ptrDoorControl->setClose();
      ptrDoorControl->setRunnable();
    }
  }

  checkInactivity(isButtonPressed);
  incRunTime(REFRESH_INTERVAL_MS);
}

void UserInput::calibrateDoor()
{
  switch (userState)
  {
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

    if (buttonStateLeft == BUTTON_STATE_PRESSED)
    {
      motor.up();
    }
    else if (buttonStateRight == BUTTON_STATE_PRESSED)
    {
      motor.down();
    }
    else
    {
      motor.brake();
    }

    if (buttonStateMid == BUTTON_STATE_PRESSED)
    {
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

    if (buttonStateLeft == BUTTON_STATE_PRESSED)
    {
      motor.up();
      DOOR_OPEN_CLOSE_DISTANCE++;
    }
    else if (buttonStateRight == BUTTON_STATE_PRESSED)
    {
      if (DOOR_OPEN_CLOSE_DISTANCE > 0)
      {
        motor.down();
        DOOR_OPEN_CLOSE_DISTANCE--;
      }
      else
      {
        motor.brake();
      }
    }
    else
    {
      motor.brake();
    }

    if (buttonStateMid == BUTTON_STATE_PRESSED)
    {
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
  if (isButtonPressed)
  {
    inactivityCounter = 0;
  }
  else
  {
    inactivityCounter++;

    if (inactivityCounter >= intervalInactive)
    {
      loggerln("UserInput.checkInactivity: Detected");

      inactivityCounter = 0;

      ptrSleep->enableSleepFromUserInactivity();
      ptrSleep->setRunnable();
    }
  }
}

/*************************************************************************/

void setup()
{
  loggerBegin();
}

void loop()
{

  loggerln("main: Starting");

  UpdateDisplay updateDisplay;
  DoorControl doorControl;
  SleepMode sleepMode(&doorControl);
  SunriseSunsetAlarms sunAlarms(&sleepMode);
  UserInput userInput(&sleepMode, &updateDisplay, &doorControl);

  // Order determines task priority
  Task *tasks[] = {
      &userInput,
      &updateDisplay,
      &sleepMode,
      &doorControl,
      &sunAlarms};

  TaskScheduler scheduler(tasks, NUM_TASKS(tasks));

  loggerln("main: Running tasks");
  scheduler.runTasks();
}

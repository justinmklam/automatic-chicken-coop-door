#include "Rtc_DS3231.h"

Rtc::Rtc()
{
  rtc = new RTC_DS3231();
}

void Rtc::begin()
{
  rtc->begin();

  if (rtc->lostPower())
  {
    //set RTC date and time to COMPILE time
    rtc->adjust(DateTime(__DATE__, __TIME__));
  }

  //Set SQW pin to OFF (in my case it was set by default to 1Hz)
  //The output of the DS3231 INT pin is connected to this pin
  //It must be connected to arduino D2 pin for wake-up
  rtc->writeSqwPinMode(DS3231_OFF);

  clear_alarms();
}

DateTime Rtc::now()
{
  return rtc->now();
}

void Rtc::set_daylight_saving_time(bool state)
{
  uint8_t new_hour;
  current = rtc->now();

  if (state) {
    new_hour = current.hour() + 1;
  }
  else {
    new_hour = current.hour() - 1;
  }

  rtc->adjust(DateTime(
    current.year(),
    current.month(),
    current.day(),
    new_hour,
    current.minute(),
    current.second()
  ));
}

void Rtc::get_datestamp_str(char *buffer)
{
  // Size of string should be no more than 8 characters
  // char *buffer = (char*) malloc(sizeof(char) * 8);

  current = rtc->now();

  year = current.year();
  month = current.month();
  day = current.day();

  sprintf(buffer, "%d-%02d-%02d", year, month, day);

  // return buffer;
}

void Rtc::get_timestamp_str(char *buffer)
{
  // Size of string should be no more than 8 characters
  // char *buffer = (char*) malloc(sizeof(char) * 5);

  current = rtc->now();

  hour = current.hour();
  minute = current.minute();

  sprintf(buffer, "%02d:%02d", hour, minute);

  // return buffer;
}

void Rtc::clear_alarms()
{
  rtc->armAlarm(1, false);
  rtc->clearAlarm(1);
  rtc->alarmInterrupt(1, false);
  rtc->armAlarm(2, false);
  rtc->clearAlarm(2);
  rtc->alarmInterrupt(2, false);
}

void Rtc::set_alarm(uint8_t alarm_number, uint8_t hour, uint8_t minute, uint8_t second)
{
  rtc->setAlarm(ALM1_MATCH_HOURS, minute, hour, second); //set your wake-up time here
  rtc->alarmInterrupt(alarm_number, true);
}

void Rtc::clear_alarm(uint8_t alarm_number)
{
  rtc->armAlarm(alarm_number, false);
  rtc->clearAlarm(alarm_number);
  rtc->alarmInterrupt(alarm_number, false);
}
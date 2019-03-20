#include "rtc_ds3231.h"

Rtc::Rtc()
{
    rtc = new RTC_DS3231();
}

void Rtc::begin()
{
    rtc->begin();

    //set RTC date and time to COMPILE time
    rtc->adjust(DateTime(__DATE__, __TIME__));

    //Set SQW pin to OFF (in my case it was set by default to 1Hz)
    //The output of the DS3231 INT pin is connected to this pin
    //It must be connected to arduino D2 pin for wake-up
    rtc->writeSqwPinMode(DS3231_OFF);
}

void Rtc::datestamp(char *buffer)
{
    now = rtc->now();

    year = now.year();
    month = now.month();
    day = now.day();

    sprintf(buffer, "%d-%d-%d",year,month,day);
}

void Rtc::timestamp(char *buffer)
{
    now = rtc->now();

    hour = now.hour();
    minute = now.minute();

    sprintf(buffer, "%d:%d",hour,minute);
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

void Rtc::set_alarm(uint8_t alarm_number, uint8_t hour, uint8_t minute, uint8_t second) {
    rtc->setAlarm(ALM1_MATCH_HOURS, minute, hour, second);   //set your wake-up time here
    rtc->alarmInterrupt(alarm_number, true);
}

void Rtc::clear_alarm(uint8_t alarm_number) {
    rtc->armAlarm(alarm_number, false);
    rtc->clearAlarm(alarm_number);
    rtc->alarmInterrupt(alarm_number, false);
}
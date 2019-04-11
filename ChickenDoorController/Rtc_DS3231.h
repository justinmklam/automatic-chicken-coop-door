#ifndef RTC_DS3231_H
#define RTC_DS3231_H

#include <RTClibExtended.h>

class Rtc
{
public:
  Rtc();

  void begin();

  DateTime now();
  void get_datestamp_str(char *buffer);
  void get_timestamp_str(char *buffer);
  void set_daylight_saving_time(bool state);

  void clear_alarms();
  void set_alarm(uint8_t alarm_number, uint8_t hour, uint8_t minute, uint8_t second);
  void clear_alarm(uint8_t alarm_number);

private:
  RTC_DS3231 *rtc;
  DateTime current;

  uint8_t month, day, hour, minute;
  uint16_t year;
};

#endif
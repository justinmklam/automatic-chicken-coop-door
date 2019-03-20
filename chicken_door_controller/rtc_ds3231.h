#ifndef RTC_DS3231_H
#define RTC_DS3231_H

#include <RTClibExtended.h>

class Rtc
{
  private:
    RTC_DS3231 *rtc;
    DateTime now;

    uint8_t month, day, hour, minute;
    uint16_t year;

  public:
    Rtc();

    void begin();

    //Writes current datestamp to the given buffer
    void datestamp(char *buffer);

    // Writes current timestamp to the given buffer
    void timestamp(char *buffer);

    void clear_alarms();
    void set_alarm(uint8_t alarm_number, uint8_t hour, uint8_t minute, uint8_t second);
    void clear_alarm(uint8_t alarm_number);
};

#endif
#ifndef RTC_DS3231_H
#define RTC_DS3231_H

#include "RTClib.h"

class Rtc
{
  private:
    RTC_DS3231 *rtc;
    DateTime now;

    uint8_t month, day, hour, minute;
    uint16_t year;

  public:
    Rtc();

    //Writes current datestamp to the given buffer
    void datestamp(char *buffer);

    // Writes current timestamp to the given buffer
    void timestamp(char *buffer);
};

#endif
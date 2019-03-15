#include "rtc_ds3231.h"

Rtc::Rtc()
{
    rtc = new RTC_DS3231();
    rtc->begin();
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
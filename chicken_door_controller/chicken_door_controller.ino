// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <Wire.h>

#include "rtc_ds3231.h"

Rtc rtc;

char buffer[40];

void setup () {

  Serial.begin(9600);

  delay(1000); // wait for console opening

  Serial.println("Starting");
}

void loop () {
    rtc.datestamp(buffer);
    Serial.print(buffer);

    Serial.print(" ");

    rtc.timestamp(buffer);
    Serial.print(buffer);

    Serial.println();
    delay(3000);
}
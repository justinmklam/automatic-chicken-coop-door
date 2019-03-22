#include <Wire.h>
#include <LowPower.h>
#include "ssd1306.h"
#include "rtc_ds3231.h"

#define ALARM_NUMBER 1
#define wakePin 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
#define ledPin 13    //use arduino on-board led for indicating sleep or wakeup status

Rtc rtc;      //we are using the DS3231 RTC
Ssd1306 display;

byte AlarmFlag = 0;
byte ledStatus = 1;

//-------------------------------------------------

void wakeUp()        // here the interrupt is handled after wakeup
{
}

//------------------------------------------------------------

void setup() {
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(wakePin, INPUT);

  //switch-on the on-board led for 1 second for indicating that the sketch is ok and running
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  // delay(1000);

  Serial.begin(9600);

  //Initialize communication with the clock
  Wire.begin();
  rtc.begin();
  display.begin();

  //Set alarm1 every day at 18:33
  rtc.set_alarm(ALARM_NUMBER, 21, 35, 0);

  display.println(rtc.get_datestamp_str());
  display.print(rtc.get_timestamp_str());
  display.print(rtc.get_timestamp_str());

  delay(1000);
}

//------------------------------------------------------------

void loop() {

  //On first loop we enter the sleep mode
  if (AlarmFlag == 0) {
    attachInterrupt(0, wakeUp, LOW);                       //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
    digitalWrite(ledPin, LOW);                             //switch-off the led for indicating that we enter the sleep mode
    ledStatus = 0;                                         //set the led status accordingly
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);   //arduino enters sleep mode here
    detachInterrupt(0);                                    //execution resumes from here after wake-up

    //When exiting the sleep mode we clear the alarm
    rtc.clear_alarm(ALARM_NUMBER);
    AlarmFlag++;
  }

  //cycles the led to indicate that we are no more in sleep mode
  if (ledStatus == 0) {
    ledStatus = 1;
    digitalWrite(ledPin, HIGH);
  }
  else {
    ledStatus = 0;
    digitalWrite(ledPin, LOW);
  }

  delay(500);
}

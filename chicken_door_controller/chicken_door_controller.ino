#include <Wire.h>
#include <LowPower.h>
#include "ssd1306.h"
#include "rtc_ds3231.h"
#include "motor_tb6612.h"

// RTC stuff
#define ALARM_NUMBER 1
#define wakePin 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
#define ledPin 13    //use arduino on-board led for indicating sleep or wakeup status

#define BUTTON_PIN_LEFT 10
#define BUTTON_PIN_MIDDLE 3
#define BUTTON_PIN_RIGHT 12

// Pullup resistor used
#define DEFAULT_BUTTON_STATE HIGH

Rtc rtc;      //we are using the DS3231 RTC
Ssd1306 display;
MotorTb6612 motor;

byte AlarmFlag = 0;
byte ledStatus = 1;

uint8_t buttonStateLeft = DEFAULT_BUTTON_STATE;
uint8_t buttonStateMid = DEFAULT_BUTTON_STATE;
uint8_t buttonStateRight = DEFAULT_BUTTON_STATE;

uint32_t startMillis;
uint32_t currentMillis;
const uint32_t period = 1000;

//-------------------------------------------------

void wakeUp()        // here the interrupt is handled after wakeup
{
}

//------------------------------------------------------------

void setup() {
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(wakePin, INPUT);
  pinMode(BUTTON_PIN_LEFT, INPUT);
  pinMode(BUTTON_PIN_MIDDLE, INPUT);
  pinMode(BUTTON_PIN_RIGHT, INPUT);

  //switch-on the on-board led for 1 second for indicating that the sketch is ok and running
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(1000);

  Serial.begin(9600);

  // Wire.begin();
  rtc.begin();
  display.begin();

  // //Set alarm1 every day at 18:33
  rtc.set_alarm(ALARM_NUMBER, 21, 06, 0);

  display.println(rtc.get_datestamp_str());
  display.print(rtc.get_timestamp_str());

  delay(2000);
}

//------------------------------------------------------------

void loop() {
  currentMillis = millis();

  //On first loop we enter the sleep mode
  if (AlarmFlag == 0) {
    display.turnOff();

    // Wake from either RTC alarm or button press
    attachInterrupt(digitalPinToInterrupt(wakePin), wakeUp, LOW);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_MIDDLE), wakeUp, CHANGE);
    digitalWrite(ledPin, LOW);                             //switch-off the led for indicating that we enter the sleep mode
    ledStatus = 0;                                         //set the led status accordingly
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);   //arduino enters sleep mode here
    detachInterrupt(0);                                    //execution resumes from here after wake-up

    //When exiting the sleep mode we clear the alarm
    rtc.clear_alarm(ALARM_NUMBER);
    AlarmFlag++;
  }

  if (currentMillis - startMillis >= period) {
    display.clear();
    display.println(rtc.get_timestamp_str());

    startMillis = currentMillis;
  }

  buttonStateLeft = digitalRead(BUTTON_PIN_LEFT);
  buttonStateMid = digitalRead(BUTTON_PIN_MIDDLE);
  buttonStateRight = digitalRead(BUTTON_PIN_RIGHT);

  if (buttonStateMid == LOW) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }

  if (buttonStateLeft == LOW) {
    motor.up();
  }
  else if (buttonStateRight == LOW) {
    motor.down();
  }
  else {
    motor.brake();
  }

}

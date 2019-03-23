#include <LowPower.h>
#include "ssd1306.h"
#include "rtc_ds3231.h"
#include "motor_tb6612.h"

Rtc rtc;      //we are using the DS3231 RTC
Ssd1306 display;
MotorTb6612 motor;

// RTC stuff
const uint8_t alarmNumber = 1;
const uint8_t wakePin = 2;    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
const uint8_t ledPin = 13;    //use arduino on-board led for indicating sleep or wakeup status

const uint8_t buttonPinLeft = 10;
const uint8_t buttonPinMiddle = 3;
const uint8_t buttonPinRight = 12;

// Pullup resistor used
const uint8_t defaultButtonState = HIGH;

uint8_t buttonStateLeft = defaultButtonState;
uint8_t buttonStateMid = defaultButtonState;
uint8_t buttonStateRight = defaultButtonState;

uint32_t startMillis;
uint32_t currentMillis;
const uint16_t period = 1000;

uint8_t alarmFlag = 0;
uint8_t ledStatus = 1;

//-------------------------------------------------

void wakeUp()        // here the interrupt is handled after wakeup
{
}

//------------------------------------------------------------

void setup() {
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(wakePin, INPUT);
  pinMode(buttonPinLeft, INPUT);
  pinMode(buttonPinMiddle, INPUT);
  pinMode(buttonPinRight, INPUT);

  //switch-on the on-board led for 1 second for indicating that the sketch is ok and running
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(1000);

  rtc.begin();
  display.begin();

  // //Set alarm1 every day at 18:33
  rtc.set_alarm(alarmNumber, 21, 06, 0);

  display.println(rtc.get_datestamp_str());
  display.print(rtc.get_timestamp_str());

  delay(2000);
}

//------------------------------------------------------------

void loop() {
  currentMillis = millis();

  //On first loop we enter the sleep mode
  if (alarmFlag == 0) {
    display.turnOff();

    // Wake from either RTC alarm or button press
    attachInterrupt(digitalPinToInterrupt(wakePin), wakeUp, LOW);
    attachInterrupt(digitalPinToInterrupt(buttonPinMiddle), wakeUp, CHANGE);
    digitalWrite(ledPin, LOW);                             //switch-off the led for indicating that we enter the sleep mode
    ledStatus = 0;                                         //set the led status accordingly
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);   //arduino enters sleep mode here
    detachInterrupt(0);                                    //execution resumes from here after wake-up

    //When exiting the sleep mode we clear the alarm
    rtc.clear_alarm(alarmNumber);
    alarmFlag++;
  }

  if (currentMillis - startMillis >= period) {
    display.clear();
    display.println(rtc.get_timestamp_str());

    startMillis = currentMillis;
  }

  buttonStateLeft = digitalRead(buttonPinLeft);
  buttonStateMid = digitalRead(buttonPinMiddle);
  buttonStateRight = digitalRead(buttonPinRight);

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

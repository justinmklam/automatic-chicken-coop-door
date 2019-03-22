#ifndef SSD1306_H
#define SSD1306_H

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SSD1306_I2C_ADDRESS 0x3c  // OLED display address for 128x32
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)

#define DEFAULT_TEXT_SIZE 2

class Ssd1306
{
private:
    Adafruit_SSD1306 *display;

public:
    Ssd1306();
    void begin();
    void clear();
    void println(char *text);
    void print(char *text);

};

#endif
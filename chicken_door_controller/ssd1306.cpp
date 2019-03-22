#include "ssd1306.h"

Ssd1306::Ssd1306()
{
    display = new Adafruit_SSD1306();
}

void Ssd1306::begin()
{
    display->begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);

    // Show adafruit splash screen
    display->display();
    display->clearDisplay();

    display->setTextSize(DEFAULT_TEXT_SIZE);
    display->setTextColor(WHITE);
}

void Ssd1306::clear()
{
    display->clearDisplay();
}

void Ssd1306::println(char *text)
{
    display->setCursor(0,0);

    display->println(text);
    display->display();
}

void Ssd1306::print(char *text)
{
    display->println(text);
    display->display();
}
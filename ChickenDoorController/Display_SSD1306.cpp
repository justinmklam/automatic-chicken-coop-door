#include "Display_SSD1306.h"

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
  display->setCursor(0, 0);

  display->println(text);
}

void Ssd1306::println()
{
  display->println();
}

void Ssd1306::print(char *text)
{
  display->print(text);
}

void Ssd1306::print(int value)
{
  display->print(value);
}

void Ssd1306::show()
{
  display->display();
}

void Ssd1306::sleep()
{
  display->clearDisplay();
  display->display();
  display->ssd1306_command(SSD1306_DISPLAYOFF);
}

void Ssd1306::wake()
{
  display->ssd1306_command(SSD1306_DISPLAYON);
}
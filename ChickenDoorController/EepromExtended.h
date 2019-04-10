#include <EEPROM.h>

void EEPROMWrite8bit(uint8_t address, uint8_t value)
{
  EEPROM.write(address, value);
}
uint8_t EEPROMRead8bit(uint8_t address)
{
  return EEPROM.read(address);
}
void EEPROMWrite32bit(uint8_t address, uint32_t value)
{
  uint8_t bytes[4];

  bytes[0] = (value >> 24) & 0xFF;
  bytes[1] = (value >> 16) & 0xFF;
  bytes[2] = (value >> 8) & 0xFF;
  bytes[3] = value & 0xFF;

  for (int i = 0; i < 4; i++)
  {
    EEPROM.write(address + i, bytes[i]);
  }
}

uint32_t EEPROMRead32bit(uint8_t address)
{
  uint32_t bytes[4];
  uint32_t value;

  for (int i = 0; i < 4; i++)
  {
    bytes[i] = EEPROM.read(address + i);
  }

  value = (bytes[3] + (bytes[2] << 8) + (bytes[1] << 16) + (bytes[0] << 24));

  return value;
}
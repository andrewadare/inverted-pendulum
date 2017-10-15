#include <i2c_t3.h>
#include "I2CCom.h"

I2CCom::I2CCom(uint8_t device_address) :
  device_addr(device_address)
{
}

uint8_t I2CCom::readByte(uint8_t register_addr)
{
  Wire.beginTransmission(device_addr);
  Wire.write(register_addr);
  Wire.endTransmission();
  Wire.requestFrom((int)device_addr, 1);
  while (Wire.available() == 0) {}
  return Wire.read();
}

uint16_t I2CCom::readTwoBytes(uint8_t register_addr_low, uint8_t register_addr_high)
{
  uint8_t low = readByte(register_addr_low);
  uint16_t high = readByte(register_addr_high);
  return (high << 8) | low;
}

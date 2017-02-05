#ifndef __I2CCOM_H__
#define __I2CCOM_H__

class I2CCom
{
public:
  I2CCom(uint8_t device_address);
  ~I2CCom() {}

  uint8_t readByte(uint8_t register_address);
  uint16_t readTwoBytes(uint8_t register_addr_low, uint8_t register_addr_high);
  // todo: add write methods when needed

  uint8_t device_addr;
};

#endif // __I2CCOM_H__
#pragma once

#include <inttypes.h>

class SPIAdapter {
public:
  virtual ~SPIAdapter() {}

  virtual uint32_t readTMC4671(uint8_t address) const {};

  virtual void writeTMC4671(uint8_t address, uint32_t data) const {};

  virtual void writeTMC6100(uint8_t address, uint32_t data) const {};

  virtual uint16_t readAS5047P(uint16_t address) const {};
};
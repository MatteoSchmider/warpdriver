#pragma once

class SPIAdapter {
public:
  SPIAdapter();
  uint16_t readAS5047P(uint16_t address);
  uint32_t readTMC4671(uint32_t address);
  void writeTMC4671(uint32_t address, uint32_t data);
  void writeTMC6100(uint32_t address, uint32_t data);
};
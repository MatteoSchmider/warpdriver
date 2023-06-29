#pragma once

#include "SPIAdapter.h"
#include "bcm2835.h"

class RaspiSPI : public SPIAdapter {
public:
  RaspiSPI(uint8_t tmc4671Pin, uint8_t tmc6100Pin, uint8_t as5047pPin);
  ~RaspiSPI();

  uint32_t readTMC4671(uint8_t address) const override;

  void writeTMC4671(uint8_t address, uint32_t data) const override;

  void writeTMC6100(uint8_t address, uint32_t data) const override;

  uint16_t readAS5047P(uint16_t address) const override;

private:
  uint8_t m_tmc4671Pin;
  uint8_t m_tmc6100Pin;
  uint8_t m_as5047pPin;
};
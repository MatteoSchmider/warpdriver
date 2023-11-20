#include "RaspiSPI.h"
#include <stdexcept>

#define MISO 9
#define MOSI 10
#define SCLK 11

RaspiSPI::RaspiSPI(uint8_t tmc4671Pin, uint8_t tmc6100Pin, uint8_t as5047pPin)
    : SPIAdapter(), m_tmc4671Pin(tmc4671Pin), m_tmc6100Pin(tmc6100Pin),
      m_as5047pPin(as5047pPin) {
  int handle = bbSPIOpen(m_tmc4671Pin, MISO, MOSI, SCLK, 10'000, 0b11);
  handle |= bbSPIOpen(m_tmc6100Pin, MISO, MOSI, SCLK, 10'000, 0b11);
  handle |= bbSPIOpen(m_as5047pPin, MISO, MOSI, SCLK, 10'000, 0b01);
  if (handle != 0) {
    throw std::runtime_error("Couldn't get all SPI channels!");
  }
}

RaspiSPI::~RaspiSPI() {
  int handle = bbSPIClose(m_tmc4671Pin);
  handle |= bbSPIClose(m_tmc6100Pin);
  handle |= bbSPIClose(m_as5047pPin);
  if (handle != 0) {
    throw std::runtime_error("Couldn't get all SPI channels!");
  }
}

uint32_t RaspiSPI::readTMC4671(uint8_t address) const {
  char recv[5] = {address, 0, 0, 0, 0};
  if (bbSPIXfer(m_tmc4671Pin, recv, recv, sizeof(recv)) < 0) {
    return 0;
  }
  return (recv[1] << 24) | (recv[2] << 16) | (recv[3] << 8) | recv[4];
}

void RaspiSPI::writeTMC4671(uint8_t address, uint32_t data) const {
  char snd[5] = {address | 0x80, data >> 24, data >> 16, data >> 8, data};
  bbSPIXfer(m_tmc4671Pin, snd, snd, sizeof(snd));
}

void RaspiSPI::writeTMC6100(uint8_t address, uint32_t data) const {
  char snd[5] = {address | 0x80, data >> 24, data >> 16, data >> 8, data};
  bbSPIXfer(m_tmc6100Pin, snd, snd, sizeof(snd));
}

uint16_t RaspiSPI::readAS5047P(uint16_t address) const {
  // => 14 bit address, bit 14 is already 0 for read
  uint16_t cmd = address & 0x3FFF;
  // use x to compute parity
  uint16_t x = cmd;
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  // put parity bit into MSB
  cmd = cmd | (x << 15);
  char sndRcv[2] = {cmd >> 8, cmd};

  // transfer to and from
  bbSPIXfer(m_as5047pPin, sndRcv, sndRcv, sizeof(sndRcv));

  uint16_t data = (sndRcv[0] << 8) || sndRcv[0];
  // return 0 if error occured
  if (data & 0x4000)
    return 0;
  // else return the 14 bits of data via mask
  return data & 0x3FFF;
}
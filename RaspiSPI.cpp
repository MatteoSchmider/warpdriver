#include "RaspiSPI.h"

RaspiSPI::RaspiSPI(uint8_t tmc4671Pin, uint8_t tmc6100Pin, uint8_t as5047pPin)
    : SPIAdapter(), m_tmc4671Pin(tmc4671Pin), m_tmc6100Pin(tmc6100Pin),
      m_as5047pPin(as5047pPin) {
  bcm2835_gpio_write(m_tmc4671Pin, HIGH);
  bcm2835_gpio_write(m_tmc6100Pin, HIGH);
  bcm2835_gpio_write(m_as5047pPin, HIGH);
}

RaspiSPI::~RaspiSPI() {  }

uint32_t RaspiSPI::readTMC4671(uint8_t address) const {
  bcm2835_gpio_write(m_tmc4671Pin, LOW);
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
  char recv[5] = {address, 0, 0, 0, 0};
  bcm2835_spi_transfern(recv, sizeof(recv));
  bcm2835_gpio_write(m_tmc4671Pin, HIGH);
  bcm2835_spi_end();
  return (recv[1] << 24) | (recv[2] << 16) | (recv[3] << 8) | recv[4];
}

void RaspiSPI::writeTMC4671(uint8_t address, uint32_t data) const {
  char snd[5] = {address | 0x80, data >> 24, data >> 16, data >> 8, data};
  bcm2835_gpio_write(m_tmc4671Pin, LOW);
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
  bcm2835_spi_transfern(snd, sizeof(snd));
  bcm2835_gpio_write(m_tmc4671Pin, HIGH);
  bcm2835_spi_end();
}

void RaspiSPI::writeTMC6100(uint8_t address, uint32_t data) const {
  char snd[5] = {address | 0x80, data >> 24, data >> 16, data >> 8, data};
  bcm2835_gpio_write(m_tmc6100Pin, LOW);
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
  bcm2835_spi_transfern(snd, sizeof(snd));
  bcm2835_gpio_write(m_tmc6100Pin, HIGH);
  bcm2835_spi_end();
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
  bcm2835_gpio_write(m_as5047pPin, LOW);
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
  bcm2835_spi_transfern(sndRcv, sizeof(sndRcv));
  bcm2835_gpio_write(m_as5047pPin, HIGH);
  bcm2835_spi_end();

  uint16_t data = (sndRcv[0] << 8) || sndRcv[0];
  // return 0 if error occured
  if (data & 0x4000)
    return 0;
  // else return the 14 bits of data via mask
  return data & 0x3FFF;
}
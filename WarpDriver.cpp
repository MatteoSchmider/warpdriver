#include "WarpDriver.h"

WarpDriver::WarpDriver(const SPIAdapter &device, MotorType motorType,
                       uint16_t numPoles, uint32_t maxPwmFrequency,
                       CalibrationData calibration)
    : m_device(device) {
  initTMC6100();
  initMotorTypeNpolePairs(motorType, numPoles);
  setPwmFrequency(maxPwmFrequency);
  initCalibration(calibration);
}

uint16_t WarpDriver::getAdcRawDataI0() const {
  m_device.writeTMC4671(TMC4671_ADC_RAW_ADDR,
                        ADC_RAW_ADDR_ADC_I1_RAW_ADC_I0_RAW);
  return m_device.readTMC4671(TMC4671_ADC_RAW_DATA);
}

uint16_t WarpDriver::getAdcRawDataI1() const {
  m_device.writeTMC4671(TMC4671_ADC_RAW_ADDR,
                        ADC_RAW_ADDR_ADC_I1_RAW_ADC_I0_RAW);
  return m_device.readTMC4671(TMC4671_ADC_RAW_DATA) >> 16;
}

uint16_t WarpDriver::getAdcRawDataVM() const {
  m_device.writeTMC4671(TMC4671_ADC_RAW_ADDR,
                        ADC_RAW_ADDR_ADC_AGPI_A_RAW_ADC_VM_RAW);
  return m_device.readTMC4671(TMC4671_ADC_RAW_DATA);
}

int16_t WarpDriver::getIux() const {
  return m_device.readTMC4671(TMC4671_ADC_IWY_IUX);
}

int16_t WarpDriver::getIv() const {
  return m_device.readTMC4671(TMC4671_ADC_IV);
}

int16_t WarpDriver::getIwy() const {
  return m_device.readTMC4671(TMC4671_ADC_IWY_IUX) >> 16;
}

int16_t WarpDriver::getTorque() const {
  return m_device.readTMC4671(TMC4671_PID_TORQUE_FLUX_ACTUAL) >> 16;
}

int32_t WarpDriver::getVelocity() const {
  return m_device.readTMC4671(TMC4671_PID_VELOCITY_ACTUAL);
}

int32_t WarpDriver::getPostion() const {
  return m_device.readTMC4671(TMC4671_PID_POSITION_ACTUAL);
}

void WarpDriver::setAdcOffsets(uint16_t i0, uint16_t i1) const {
  m_device.writeTMC4671(TMC4671_ADC_I0_SCALE_OFFSET, i0 | (325 << 16));
  m_device.writeTMC4671(TMC4671_ADC_I1_SCALE_OFFSET, i1 | (325 << 16));
}

void WarpDriver::setSwitchI0I1(bool doSwitch) const {
  if (doSwitch)
    m_device.writeTMC4671(TMC4671_ADC_I_SELECT, 0x240001);
  else
    m_device.writeTMC4671(TMC4671_ADC_I_SELECT, 0x240100);
}

void WarpDriver::setPwmFrequency(uint32_t frequency) const {
  uint16_t maxcnt = (100'000'000 / frequency) - 1;
  uint16_t mdec = ((maxcnt + 1) / 12) - 2;
  m_device.writeTMC4671(TMC4671_dsADC_MDEC_B_MDEC_A,
                        (((uint32_t)mdec) << 16) | mdec);
  m_device.writeTMC4671(TMC4671_PWM_MAXCNT, maxcnt);
}

void WarpDriver::setUseSvpwm(bool on) const {
  if (on)
    m_device.writeTMC4671(TMC4671_PWM_SV_CHOP, 0x107);
  else
    m_device.writeTMC4671(TMC4671_PWM_SV_CHOP, 0x007);
}

void WarpDriver::initEncoder(int16_t offset) const {
  m_device.writeTMC4671(TMC4671_ABN_DECODER_MODE, 0xF);
  m_device.writeTMC4671(TMC4671_ABN_DECODER_PPR, 1000);
  m_device.writeTMC4671(TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET,
                        (((uint32_t)offset) << 16) | ((uint16_t)offset));
}

void WarpDriver::initMotorTypeNpolePairs(MotorType motorType,
                                         uint16_t numPolePairs) const {
  m_device.writeTMC4671(TMC4671_MOTOR_TYPE_N_POLE_PAIRS,
                        (((uint32_t)motorType) << 16) | numPolePairs);
}

void WarpDriver::initTMC6100() const {
  m_device.writeTMC4671(TMC4671_PWM_BBM_H_BBM_L, 0x2828);
  m_device.writeTMC6100(TMC6100_GCONF, 0x44);
  m_device.writeTMC6100(TMC6100_DRV_CONF, 0);
  // maybe also set PWM_POLARITIES if they don't already match
}

WarpDriver::CalibrationData
WarpDriver::autoCalibrate(uint64_t maxNumRetries, uint64_t numRetriesSuccess,
                          uint16_t jitter) const {
  // see TMC4671 Datasheet "3.5.7.2 Bang-Bang Initialization of the Encoder"
  CalibrationData retVal = {.encoderOffset = 0, .i0Offset = 0, .i1Offset = 0};

  setMotionMode(MotionMode::UQ_UD_EXT);
  m_device.writeTMC4671(TMC4671_PHI_E_SELECTION, 2);
  m_device.writeTMC4671(TMC4671_PIDOUT_UQ_UD_LIMITS, 32767);
  m_device.writeTMC4671(TMC4671_UQ_UD_EXT, 32767);

  uint64_t start = 0;
  uint32_t pos = 0;
  uint32_t posDiff = 0;
  bool done = false;
  for (uint64_t i = 0; !done; i++) {
    posDiff = m_device.readAS5047P(0x3FFF) - pos;
    pos += posDiff;
    if (posDiff == 0) {
      if ((i - start) >= numRetriesSuccess) {
        // Bang-Bang successful
        done = true;
        retVal.encoderOffset = pos;
      }
    } else {
      start = i;
    }
    if (i >= maxNumRetries) {
      // Bang-Bang unsuccessful
      return retVal;
    }
  }

  m_device.writeTMC4671(TMC4671_UQ_UD_EXT, 0);
  m_device.writeTMC4671(TMC4671_PHI_E_SELECTION, 3);
  setMotionMode(MotionMode::VELOCITY);

  start = 0;
  uint32_t i0 = 0;
  uint32_t i0Diff = 0;
  bool done = false;
  for (uint64_t i = 0; !done; i++) {
    i0Diff = getAdcRawDataI0();
    i0 += i0Diff;
    if (i0Diff < jitter) {
      if ((i - start) >= numRetriesSuccess) {
        // Bang-Bang successful
        done = true;
        retVal.i0Offset = i0;
      }
    } else {
      start = i;
    }
    if (i >= maxNumRetries) {
      // Bang-Bang unsuccessful
      return retVal;
    }
  }

  start = 0;
  uint32_t i1 = 0;
  uint32_t i1Diff = 0;
  bool done = false;
  for (uint64_t i = 0; !done; i++) {
    i1Diff = getAdcRawDataI0();
    i1 += i1Diff;
    if (i1Diff < jitter) {
      if ((i - start) >= numRetriesSuccess) {
        // Bang-Bang successful
        done = true;
        retVal.i1Offset = i1;
      }
    } else {
      start = i;
    }
    if (i >= maxNumRetries) {
      // Bang-Bang unsuccessful
      return retVal;
    }
  }

  return retVal;
}

void WarpDriver::initCalibration(CalibrationData calibration) const {
  if (calibration.encoderOffset > 0)
    initEncoder(calibration.encoderOffset);
  if ((calibration.i0Offset > 0) && (calibration.i1Offset > 0))
    setAdcOffsets(calibration.i0Offset, calibration.i1Offset);
  m_device.writeTMC4671(TMC4671_PHI_E_SELECTION, 3);
  m_device.writeTMC4671(TMC4671_VELOCITY_SELECTION, 9);
  m_device.writeTMC4671(TMC4671_POSITION_SELECTION, 9);
  setMotionMode(MotionMode::VELOCITY);
}

void WarpDriver::setFluxPI(int16_t P, int16_t I) const {
  m_device.writeTMC4671(TMC4671_PID_FLUX_P_FLUX_I,
                        (((uint32_t)P) << 16) | ((uint16_t)I));
}

void WarpDriver::setTorquePI(int16_t P, int16_t I) const {
  m_device.writeTMC4671(TMC4671_PID_TORQUE_P_TORQUE_I,
                        (((uint32_t)P) << 16) | ((uint16_t)I));
}

void WarpDriver::setVelocityPI(int16_t P, int16_t I) const {
  m_device.writeTMC4671(TMC4671_PID_VELOCITY_P_VELOCITY_I,
                        (((uint32_t)P) << 16) | ((uint16_t)I));
}

void WarpDriver::setPositionPI(int16_t P, int16_t I) const {
  m_device.writeTMC4671(TMC4671_PID_POSITION_P_POSITION_I,
                        (((uint32_t)P) << 16) | ((uint16_t)I));
}

void WarpDriver::setCurrentLimit(int16_t limit) const {
  m_device.writeTMC4671(TMC4671_PIDOUT_UQ_UD_LIMITS, (uint32_t)limit);
}

void WarpDriver::setVelocityLimit(uint32_t limit) const {
  m_device.writeTMC4671(TMC4671_PID_VELOCITY_LIMIT, limit);
}

void WarpDriver::setPositionLimits(int32_t low, int32_t high) const {
  m_device.writeTMC4671(TMC4671_PID_POSITION_LIMIT_LOW, (uint32_t)low);
  m_device.writeTMC4671(TMC4671_PID_POSITION_LIMIT_HIGH, (uint32_t)high);
}

void WarpDriver::setMotionMode(MotionMode mode) const {
  m_device.writeTMC4671(TMC4671_MODE_RAMP_MODE_MOTION,
                        ((uint32_t)mode) | 0x80000000);
}

void WarpDriver::setTorqueTarget(int16_t value) const {
  m_device.writeTMC4671(TMC4671_PID_TORQUE_FLUX_TARGET, ((uint32_t)value)
                                                            << 16);
}

void WarpDriver::setVelocityTarget(int32_t value) const {
  m_device.writeTMC4671(TMC4671_PID_VELOCITY_TARGET, ((uint32_t)value));
}

void WarpDriver::setPositionTarget(int32_t value) const {
  m_device.writeTMC4671(TMC4671_PID_POSITION_TARGET, ((uint32_t)value));
}

bool WarpDriver::getStatus(StatusMask mask) const {
  m_device.writeTMC4671(TMC4671_STATUS_MASK, 0xFFFFFFFF);
  return m_device.readTMC4671(TMC4671_STATUS_FLAGS) & mask;
}


#pragma once

#include "SPIAdapter.h"

#include <inttypes.h>

#include "TMC-API/tmc/ic/TMC4671/TMC4671_Register.h"
#include "TMC-API/tmc/ic/TMC4671/TMC4671_Variants.h"
#include "TMC-API/tmc/ic/TMC6100/TMC6100_Register.h"

class WarpDriver {
public:
  typedef enum MotorType {
    NONE = 0,
    SINGLE_PHASE_DC = 1,
    TWO_PHASE_STEPPER = 2,
    THREE_PHASE_BLDC = 3
  };

  typedef enum MotionMode {
    STOPPED = 0,
    TORQUE = 1,
    VELOCITY = 2,
    POSITION = 3,
    UQ_UD_EXT = 8,
  };

  typedef enum StatusMask {
    PID_X_TARGET_LIMIT = 0x1,
    PID_X_TARGET_DDT_LIMIT = 0x2,
    PID_X_ERRSUM_LIMIT = 0x4,
    PID_X_OUTPUT_LIMIT = 0x8,
    PID_V_TARGET_LIMIT = 0x10,
    PID_V_TARGET_DDT_LIMIT = 0x20,
    PID_V_ERRSUM_LIMIT = 0x40,
    PID_V_OUTPUT_LIMIT = 0x80,
    PID_ID_TARGET_LIMIT = 0x100,
    PID_ID_TARGET_DDT_LIMIT = 0x200,
    PID_ID_ERRSUM_LIMIT = 0x400,
    PID_ID_OUTPUT_LIMIT = 0x800,
    PID_IQ_TARGET_LIMIT = 0x1000,
    PID_IQ_TARGET_DDT_LIMIT = 0x2000,
    PID_IQ_ERRSUM_LIMIT = 0x4000,
    PID_IQ_OUTPUT_LIMIT = 0x8000,
    IPARK_CIRLIM_LIMIT_U_D = 0x10000,
    IPARK_CIRLIM_LIMIT_U_Q = 0x20000,
    IPARK_CIRLIM_LIMIT_U_R = 0x40000,
    NOT_PLL_LOCKED = 0x80000,
    REF_SW_R = 0x100000,
    REF_SW_H = 0x200000,
    REF_SW_L = 0x400000,
    PWM_MIN = 0x1000000,
    PWM_MAX = 0x2000000,
    ADC_I_CLIPPED = 0x4000000,
    ADC_AENC_CLIPPED = 0x8000000,
    ENC_N = 0x10000000,
    ENC2_N = 0x20000000,
    AENC_N = 0x40000000
  };

  typedef struct CalibrationData {
    int16_t encoderOffset;
    int16_t i0Offset;
    int16_t i1Offset;
  };

  WarpDriver(const SPIAdapter &device, MotorType motorType, uint16_t numPoles,
             CalibrationData calibration);

  // TMC4671 section

  // Get raw adc current values for calibration
  uint16_t getAdcRawDataI0();
  uint16_t getAdcRawDataI1();
  uint16_t getAdcRawDataVM();

  // Get phase currents in mA for power consumption monitoring
  int16_t getIux(); // ADC_IWY_IUX
  int16_t getIv();  // ADC_IV
  int16_t getIwy(); // ADC_IWY_IUX

  // Get the actual torque in mA
  int16_t getTorque(); // PID_TORQUE_FLUX_ACTUAL
  // Get the velocity in int32_t driver's format
  int32_t getVelocity(); // PID_VELOCITY_ACTUAL
  // Get the number of revolutions (upper 16 bits) and mechanical encoder angle
  // (lower 16 bits)
  int32_t getPostion(); // PID_POSITION_ACTUAL

  // Switch I0 and I1 ADCs, if they are not in phase with PWM voltages
  void setSwitchI0I1(bool doSwitch); // ADC_I_SELECT

  // Set max pwm frequency in Hz
  void setPwmFrequency(uint32_t frequency); // dsADC_MDEC_B_MDEC_A, PWM_MAXCNT

  // Enable/Disable svpwm, should be default on for 3-phase, before calibration!
  void setUseSvpwm(bool on); // PWM_SV_CHOP

  void initEncoder(int16_t offset); // ABN_DECODER_MODE, ABN_DECODER_PPR,
                                    // ABN_DECODER_PHI_E_PHI_M_OFFSET
  void
  initMotorTypeNpolePairs(MotorType motorType,
                          uint16_t numPolePairs); // MOTOR_TYPE_N_POLE_PAIRS
  void initTMC6100(); // PWM_BBM_H_BBM_L = 400ns, maybe also PWM_POLARITIES if
                      // they don't already match

  CalibrationData autoCalibrate(
      uint64_t maxNumRetries, uint64_t numRetriesSuccess,
      uint16_t jitter); // PHI_E_SELECTION = phi_e_openloop,
                        // PIDOUT_UQ_UD_LIMITS, UQ_UD_EXT, make sure
                        // initEncoder(0) and initMotorTypeNpoles() and
                        // initTMC6100() are called first,
                        // initCalibration() and initPhiePhim() afterwards

  void
  initCalibration(CalibrationData calibration); // PHI_E_SELECTION = phi_e_abn

  void setFluxPI(int16_t P, int16_t I);     // PID_FLUX_P_FLUX_I
  void setTorquePI(int16_t P, int16_t I);   // PID_TORQUE_P_TORQUE_I
  void setVelocityPI(int16_t P, int16_t I); // PID_VELOCITY_P_VELOCITY_I
  void setPositionPI(int16_t P, int16_t I); // PID_POSITION_P_POSITION_I

  void setCurrentLimit(int16_t limit);   // PID_TORQUE_FLUX_LIMITS
  void setVelocityLimit(uint32_t limit); // PID_VELOCITY_LIMIT
  void setPositionLimits(
      int32_t low,
      int32_t high); // PID_POSITION_LIMIT_LOW, PID_POSITION_LIMIT_HIGH

  void setMotionMode(MotionMode mode); // MODE_RAMP_MODE_MOTION

  void setTorqueTarget(int16_t value);   // PID_TORQUE_FLUX_TARGET
  void setVelocityTarget(int32_t value); // PID_VELOCITY_TARGET
  void setPositionTarget(int32_t value); // PID_POSITION_TARGET

  bool getStatus(
      StatusMask mask); // set STATUS_MASK to 0xFFFFFFFF, then read STATUS_FLAGS

private:
  // Set adc scales and offsets
  // set default calculated SCALE to 325
  void setAdcOffsets(uint16_t i0,
                     uint16_t i1); // ADC_I0_SCALE_OFFSET, ADC_I1_SCALE_OFFSET
  SPIAdapter m_device;
};
/*
 * DistanceSensor.c
 *
 *  Created on: 5.02.2019
 *      Author: Marek Wdowiak
 */

#include "DistanceSensor.h"

#define ADDRESS_DEFAULT 0b0101001

// Record the current time to check an upcoming timeout against
#define startTimeout() (self->timeout_start_ms = DistanceSensor_millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (self->io_timeout > 0 && ((uint16_t)DistanceSensor_millis() - self->timeout_start_ms) > self->io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

void DistanceSensor_ctor(DistanceSensor* self, I2C_HandleTypeDef *hi2c)
{
	self->address = 0x52;
	self->hi2c = *hi2c;
	DistanceSensor_counter = 0;
}

void DistanceSensor_dtor(DistanceSensor* self)
{

}

void DistanceSensor_tick()
{
	DistanceSensor_counter++;
}

uint32_t DistanceSensor_millis()
{
	return DistanceSensor_counter;
}

//dopracowac
void DistanceSensor_setAddress(DistanceSensor* self, uint8_t new_addr)
{
	DistanceSensor_writeReg(self, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
	self->address = new_addr;
}

uint8_t DistanceSensor_init(DistanceSensor* self)
{
  // VL53L0X_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	DistanceSensor_writeReg(self, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
		DistanceSensor_readReg(self, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0

  // "Set I2C standard mode"
  DistanceSensor_writeReg(self, 0x88, 0x00);

  DistanceSensor_writeReg(self, 0x80, 0x01);
  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x00, 0x00);
  self->stop_variable = DistanceSensor_readReg(self, 0x91);
  DistanceSensor_writeReg(self, 0x00, 0x01);
  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, 0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  uint8_t signalRate = DistanceSensor_readReg(self, MSRC_CONFIG_CONTROL) | 0x12;
  DistanceSensor_writeReg(self, MSRC_CONFIG_CONTROL, signalRate);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  DistanceSensor_setSignalRateLimit(self, 0.25);

  DistanceSensor_writeReg(self, SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  uint8_t spad_type_is_aperture;
  if (!DistanceSensor_getSpadInfo(self, &spad_count, &spad_type_is_aperture)) { return 0; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  DistanceSensor_readMulti(self, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  DistanceSensor_writeReg(self, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  DistanceSensor_writeMulti(self, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x00, 0x00);

  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, 0x09, 0x00);
  DistanceSensor_writeReg(self, 0x10, 0x00);
  DistanceSensor_writeReg(self, 0x11, 0x00);

  DistanceSensor_writeReg(self, 0x24, 0x01);
  DistanceSensor_writeReg(self, 0x25, 0xFF);
  DistanceSensor_writeReg(self, 0x75, 0x00);

  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x4E, 0x2C);
  DistanceSensor_writeReg(self, 0x48, 0x00);
  DistanceSensor_writeReg(self, 0x30, 0x20);

  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, 0x30, 0x09);
  DistanceSensor_writeReg(self, 0x54, 0x00);
  DistanceSensor_writeReg(self, 0x31, 0x04);
  DistanceSensor_writeReg(self, 0x32, 0x03);
  DistanceSensor_writeReg(self, 0x40, 0x83);
  DistanceSensor_writeReg(self, 0x46, 0x25);
  DistanceSensor_writeReg(self, 0x60, 0x00);
  DistanceSensor_writeReg(self, 0x27, 0x00);
  DistanceSensor_writeReg(self, 0x50, 0x06);
  DistanceSensor_writeReg(self, 0x51, 0x00);
  DistanceSensor_writeReg(self, 0x52, 0x96);
  DistanceSensor_writeReg(self, 0x56, 0x08);
  DistanceSensor_writeReg(self, 0x57, 0x30);
  DistanceSensor_writeReg(self, 0x61, 0x00);
  DistanceSensor_writeReg(self, 0x62, 0x00);
  DistanceSensor_writeReg(self, 0x64, 0x00);
  DistanceSensor_writeReg(self, 0x65, 0x00);
  DistanceSensor_writeReg(self, 0x66, 0xA0);

  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x22, 0x32);
  DistanceSensor_writeReg(self, 0x47, 0x14);
  DistanceSensor_writeReg(self, 0x49, 0xFF);
  DistanceSensor_writeReg(self, 0x4A, 0x00);

  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, 0x7A, 0x0A);
  DistanceSensor_writeReg(self, 0x7B, 0x00);
  DistanceSensor_writeReg(self, 0x78, 0x21);

  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x23, 0x34);
  DistanceSensor_writeReg(self, 0x42, 0x00);
  DistanceSensor_writeReg(self, 0x44, 0xFF);
  DistanceSensor_writeReg(self, 0x45, 0x26);
  DistanceSensor_writeReg(self, 0x46, 0x05);
  DistanceSensor_writeReg(self, 0x40, 0x40);
  DistanceSensor_writeReg(self, 0x0E, 0x06);
  DistanceSensor_writeReg(self, 0x20, 0x1A);
  DistanceSensor_writeReg(self, 0x43, 0x40);

  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, 0x34, 0x03);
  DistanceSensor_writeReg(self, 0x35, 0x44);

  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x31, 0x04);
  DistanceSensor_writeReg(self, 0x4B, 0x09);
  DistanceSensor_writeReg(self, 0x4C, 0x05);
  DistanceSensor_writeReg(self, 0x4D, 0x04);

  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, 0x44, 0x00);
  DistanceSensor_writeReg(self, 0x45, 0x20);
  DistanceSensor_writeReg(self, 0x47, 0x08);
  DistanceSensor_writeReg(self, 0x48, 0x28);
  DistanceSensor_writeReg(self, 0x67, 0x00);
  DistanceSensor_writeReg(self, 0x70, 0x04);
  DistanceSensor_writeReg(self, 0x71, 0x01);
  DistanceSensor_writeReg(self, 0x72, 0xFE);
  DistanceSensor_writeReg(self, 0x76, 0x00);
  DistanceSensor_writeReg(self, 0x77, 0x00);

  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x0D, 0x01);

  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, 0x80, 0x01);
  DistanceSensor_writeReg(self, 0x01, 0xF8);

  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x8E, 0x01);
  DistanceSensor_writeReg(self, 0x00, 0x01);
  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, 0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  DistanceSensor_writeReg(self, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  DistanceSensor_writeReg(self, GPIO_HV_MUX_ACTIVE_HIGH, DistanceSensor_readReg(self, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  DistanceSensor_writeReg(self, SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  self->measurement_timing_budget_us = DistanceSensor_getMeasurementTimingBudget(self);

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  DistanceSensor_writeReg(self, SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  DistanceSensor_setMeasurementTimingBudget(self, self->measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  DistanceSensor_writeReg(self, SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return 0; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  DistanceSensor_writeReg(self, SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return 0; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  DistanceSensor_writeReg(self, SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return 1;
}

void DistanceSensor_writeReg(DistanceSensor *self, uint8_t reg, uint8_t value)
{
	self->last_status = HAL_I2C_Mem_Write(&(self->hi2c), self->address, reg, 1, &value, 1, 100);
}

void DistanceSensor_writeReg16Bit(DistanceSensor* self, uint8_t reg, uint16_t value)
{
	uint8_t data[2];
	data[1] = value >> 8;    // MSB byte of 16bit data
	data[0] = value;       // LSB byte of 16bit data
	self->last_status = HAL_I2C_Mem_Write(&(self->hi2c), self->address, reg, 1, data, 2, 100);
}

void DistanceSensor_writeReg32Bit(DistanceSensor* self, uint8_t reg, uint32_t value)
{
	uint8_t data[4];
	data[3] = value >> 24;
	data[2] = value >> 16;
	data[1] = value >> 8;
	data[0] = value;
	self->last_status = HAL_I2C_Mem_Write(&(self->hi2c), self->address, reg, 1, data, 4, 100);
}

uint8_t DistanceSensor_readReg(DistanceSensor* self, uint8_t reg)
{
  uint8_t value;
  self->last_status = HAL_I2C_Mem_Read(&(self->hi2c), self->address, reg, 1, &value, 1, 100);
  return value;
}

uint16_t DistanceSensor_readReg16Bit(DistanceSensor* self, uint8_t reg)
{
  uint8_t data[2];
  uint16_t value;
  self->last_status = HAL_I2C_Mem_Read(&(self->hi2c), self->address, reg, 1, data, 2, 100);
  value = ((data[1] << 8) | data[0]);
  return value;
}

uint32_t DistanceSensor_readReg32Bit(DistanceSensor* self, uint8_t reg)
{
	  uint8_t data[4];
	  uint32_t value;
	  self->last_status = HAL_I2C_Mem_Read(&(self->hi2c), self->address, reg, 1, data, 4, 100);
	  value = ((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
	  return value;
}

void DistanceSensor_writeMulti(DistanceSensor* self, uint8_t reg, uint8_t* src, uint8_t count)
{
	self->last_status = HAL_I2C_Mem_Write(&(self->hi2c), self->address, reg, 1, src, count, 100);
}

void DistanceSensor_readMulti(DistanceSensor* self, uint8_t reg, uint8_t * dst, uint8_t count)
{
	self->last_status = HAL_I2C_Mem_Read(&(self->hi2c), self->address, reg, 1, dst, count, 100);
}

uint8_t DistanceSensor_setSignalRateLimit(DistanceSensor *self, float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return 0; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  DistanceSensor_writeReg16Bit(self, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return 1;
}

float DistanceSensor_getSignalRateLimit(DistanceSensor *self)
{
  return (float)DistanceSensor_readReg16Bit(self, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

uint8_t DistanceSensor_setMeasurementTimingBudget(DistanceSensor *self, uint32_t budget_us)
{
  DistanceSensor_SequenceStepEnables enables;
  DistanceSensor_SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return 0; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  DistanceSensor_getSequenceStepEnables(self, &enables);
  DistanceSensor_getSequenceStepTimeouts(self, &enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return 0;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
    		DistanceSensor_timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    DistanceSensor_writeReg16Bit(self, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, DistanceSensor_encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    self->measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return 1;
}

uint32_t DistanceSensor_getMeasurementTimingBudget(DistanceSensor *self)
{
  DistanceSensor_SequenceStepEnables enables;
  DistanceSensor_SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  DistanceSensor_getSequenceStepEnables(self, &enables);
  DistanceSensor_getSequenceStepTimeouts(self, &enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range   )
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  self->measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

uint8_t DistanceSensor_setVcselPulsePeriod(DistanceSensor* self, DistanceSensor_vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  DistanceSensor_SequenceStepEnables enables;
  DistanceSensor_SequenceStepTimeouts timeouts;

  DistanceSensor_getSequenceStepEnables(self, &enables);
  DistanceSensor_getSequenceStepTimeouts(self, &enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
    	  DistanceSensor_writeReg(self, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
    	  DistanceSensor_writeReg(self, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
    	  DistanceSensor_writeReg(self, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
    	  DistanceSensor_writeReg(self, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return 0;
    }
    DistanceSensor_writeReg(self, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    DistanceSensor_writeReg(self, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
    		DistanceSensor_timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    DistanceSensor_writeReg16Bit(self, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    		DistanceSensor_encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
    		DistanceSensor_timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    DistanceSensor_writeReg(self, MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
    	  DistanceSensor_writeReg(self, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
    	  DistanceSensor_writeReg(self, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
    	  DistanceSensor_writeReg(self, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
    	  DistanceSensor_writeReg(self, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
    	  DistanceSensor_writeReg(self, 0xFF, 0x01);
    	  DistanceSensor_writeReg(self, ALGO_PHASECAL_LIM, 0x30);
    	  DistanceSensor_writeReg(self, 0xFF, 0x00);
        break;

      case 10:
    	  DistanceSensor_writeReg(self, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
    	  DistanceSensor_writeReg(self, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
    	  DistanceSensor_writeReg(self, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
    	  DistanceSensor_writeReg(self, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
    	  DistanceSensor_writeReg(self, 0xFF, 0x01);
    	  DistanceSensor_writeReg(self, ALGO_PHASECAL_LIM, 0x20);
    	  DistanceSensor_writeReg(self, 0xFF, 0x00);
        break;

      case 12:
    	  DistanceSensor_writeReg(self, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
    	  DistanceSensor_writeReg(self, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
    	  DistanceSensor_writeReg(self, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
    	  DistanceSensor_writeReg(self, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
    	  DistanceSensor_writeReg(self, 0xFF, 0x01);
    	  DistanceSensor_writeReg(self, ALGO_PHASECAL_LIM, 0x20);
    	  DistanceSensor_writeReg(self, 0xFF, 0x00);
        break;

      case 14:
    	  DistanceSensor_writeReg(self, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
    	  DistanceSensor_writeReg(self, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
    	  DistanceSensor_writeReg(self, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
    	  DistanceSensor_writeReg(self, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
    	  DistanceSensor_writeReg(self, 0xFF, 0x01);
    	  DistanceSensor_writeReg(self, ALGO_PHASECAL_LIM, 0x20);
    	  DistanceSensor_writeReg(self, 0xFF, 0x00);
    	  break;

      default:
        // invalid period
        return 0;
    }

    // apply new VCSEL period
    DistanceSensor_writeReg(self, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
    		DistanceSensor_timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    DistanceSensor_writeReg16Bit(self, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    		DistanceSensor_encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return 0;
  }

  // "Finally, the timing budget must be re-applied"

  DistanceSensor_setMeasurementTimingBudget(self, self->measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = DistanceSensor_readReg(self, SYSTEM_SEQUENCE_CONFIG);
  DistanceSensor_writeReg(self, SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  DistanceSensor_writeReg(self, SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return 1;
}

uint8_t DistanceSensor_getVcselPulsePeriod(DistanceSensor *self, DistanceSensor_vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(DistanceSensor_readReg(self, PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(DistanceSensor_readReg(self, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

void DistanceSensor_startContinuous(DistanceSensor *self, uint32_t period_ms)
{
	DistanceSensor_writeReg(self, 0x80, 0x01);
	DistanceSensor_writeReg(self, 0xFF, 0x01);
	DistanceSensor_writeReg(self, 0x00, 0x00);
	DistanceSensor_writeReg(self, 0x91, self->stop_variable);
	DistanceSensor_writeReg(self, 0x00, 0x01);
	DistanceSensor_writeReg(self, 0xFF, 0x00);
	DistanceSensor_writeReg(self, 0x80, 0x00);

	if (period_ms != 0)
	{
	    // continuous timed mode

	    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

	    uint16_t osc_calibrate_val = DistanceSensor_readReg16Bit(self, OSC_CALIBRATE_VAL);

	    if (osc_calibrate_val != 0)
	    {
	      period_ms *= osc_calibrate_val;
	    }

	    DistanceSensor_writeReg32Bit(self, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

	    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

	    DistanceSensor_writeReg(self, SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
	}
	else
	{
	    // continuous back-to-back mode
		DistanceSensor_writeReg(self, SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}
}

void DistanceSensor_stopContinuous(DistanceSensor *self)
{
	DistanceSensor_writeReg(self, SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

	DistanceSensor_writeReg(self, 0xFF, 0x01);
	DistanceSensor_writeReg(self, 0x00, 0x00);
	DistanceSensor_writeReg(self, 0x91, 0x00);
	DistanceSensor_writeReg(self, 0x00, 0x01);
	DistanceSensor_writeReg(self, 0xFF, 0x00);
}

uint16_t DistanceSensor_readRangeContinuousMillimeters(DistanceSensor *self)
{
	startTimeout();
	while ((DistanceSensor_readReg(self, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
	{
		if (checkTimeoutExpired())
		{
			self->did_timeout = 1;
			return 65535;
		}
	}

	// assumptions: Linearity Corrective Gain is 1000 (default);
	// fractional ranging is not enabled
	uint16_t range = DistanceSensor_readReg16Bit(self, RESULT_RANGE_STATUS + 10);

	DistanceSensor_writeReg(self, SYSTEM_INTERRUPT_CLEAR, 0x01);

	return range;
}

uint16_t DistanceSensor_readRangeSingleMillimeters(DistanceSensor *self)
{
	DistanceSensor_writeReg(self, 0x80, 0x01);
	DistanceSensor_writeReg(self, 0xFF, 0x01);
	DistanceSensor_writeReg(self, 0x00, 0x00);
	DistanceSensor_writeReg(self, 0x91, self->stop_variable);
	DistanceSensor_writeReg(self, 0x00, 0x01);
	DistanceSensor_writeReg(self, 0xFF, 0x00);
	DistanceSensor_writeReg(self, 0x80, 0x00);

	DistanceSensor_writeReg(self, SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  while (DistanceSensor_readReg(self, SYSRANGE_START) & 0x01)
  {
    if (checkTimeoutExpired())
    {
      self->did_timeout = 1;
      return 65535;
    }
  }

  return DistanceSensor_readRangeContinuousMillimeters(self);
}

uint8_t DistanceSensor_timeoutOccurred(DistanceSensor *self)
{
  uint8_t tmp = self->did_timeout;
  self->did_timeout = 0;
  return tmp;
}

//PRIVATE
uint8_t DistanceSensor_getSpadInfo(DistanceSensor *self, uint8_t * count, uint8_t * type_is_aperture)
{
  uint8_t tmp;

  DistanceSensor_writeReg(self, 0x80, 0x01);
  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x00, 0x00);

  DistanceSensor_writeReg(self, 0xFF, 0x06);
  DistanceSensor_writeReg(self, 0x83, DistanceSensor_readReg(self, 0x83) | 0x04);
  DistanceSensor_writeReg(self, 0xFF, 0x07);
  DistanceSensor_writeReg(self, 0x81, 0x01);

  DistanceSensor_writeReg(self, 0x80, 0x01);

  DistanceSensor_writeReg(self, 0x94, 0x6b);
  DistanceSensor_writeReg(self, 0x83, 0x00);
  startTimeout();
  while (DistanceSensor_readReg(self, 0x83) == 0x00)
  {
    if (checkTimeoutExpired()) { return 0; }
  }
  DistanceSensor_writeReg(self, 0x83, 0x01);
  tmp = DistanceSensor_readReg(self, 0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  DistanceSensor_writeReg(self, 0x81, 0x00);
  DistanceSensor_writeReg(self, 0xFF, 0x06);
  DistanceSensor_writeReg(self, 0x83, DistanceSensor_readReg(self, 0x83)  & ~0x04);
  DistanceSensor_writeReg(self, 0xFF, 0x01);
  DistanceSensor_writeReg(self, 0x00, 0x01);

  DistanceSensor_writeReg(self, 0xFF, 0x00);
  DistanceSensor_writeReg(self, 0x80, 0x00);

  return 1;
}

void getSequenceStepEnables(DistanceSensor *self, DistanceSensor_SequenceStepEnables * enables)
{
  uint8_t sequence_config = DistanceSensor_readReg(self, SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

void getSequenceStepTimeouts(DistanceSensor *self, DistanceSensor_SequenceStepEnables const * enables,
		DistanceSensor_SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = DistanceSensor_getVcselPulsePeriod(self, VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = DistanceSensor_readReg(self, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
		  DistanceSensor_timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
		  DistanceSensor_decodeTimeout(DistanceSensor_readReg16Bit(self, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
		  DistanceSensor_timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = DistanceSensor_getVcselPulsePeriod(self, VcselPeriodFinalRange);

  timeouts->final_range_mclks =
		  DistanceSensor_decodeTimeout(DistanceSensor_readReg16Bit(self, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
		  DistanceSensor_timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

uint8_t DistanceSensor_performSingleRefCalibration(DistanceSensor *self, uint8_t vhv_init_byte)
{
	DistanceSensor_writeReg(self, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  while ((DistanceSensor_readReg(self, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired()) { return 0; }
  }

  DistanceSensor_writeReg(self, SYSTEM_INTERRUPT_CLEAR, 0x01);

  DistanceSensor_writeReg(self, SYSRANGE_START, 0x00);

  return 1;
}

static uint16_t DistanceSensor_decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t DistanceSensor_encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

static uint32_t DistanceSensor_timeoutMclksToMicroseconds(
		uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t DistanceSensor_timeoutMicrosecondsToMclks(
		uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

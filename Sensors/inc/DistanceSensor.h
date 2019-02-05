/*
 * DistanceSensor.h
 *
 *  Created on: 5.02.2019
 *      Author: Marek Wdowiak
 */

#ifndef INC_DISTANCESENSOR_H_
#define INC_DISTANCESENSOR_H_

#include <stdint.h>
#include "stm32l0xx.h"

//PUBLIC

typedef enum {
	SYSRANGE_START = 0x00,

	SYSTEM_THRESH_HIGH = 0x0C,
	SYSTEM_THRESH_LOW = 0x0E,

	SYSTEM_SEQUENCE_CONFIG = 0x01,
	SYSTEM_RANGE_CONFIG = 0x09,
	SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

	SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

	GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

	SYSTEM_INTERRUPT_CLEAR = 0x0B,

	RESULT_INTERRUPT_STATUS = 0x13,
	RESULT_RANGE_STATUS = 0x14,

	RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
	RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
	RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
	RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
	RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

	ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

	I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

	MSRC_CONFIG_CONTROL = 0x60,

	PRE_RANGE_CONFIG_MIN_SNR = 0x27,
	PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
	PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
	PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

	FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
	FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
	FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
	FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

	PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
	PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

	PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

	SYSTEM_HISTOGRAM_BIN = 0x81,
	HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
	HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

	FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
	CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

	MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

	SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
	IDENTIFICATION_MODEL_ID = 0xC0,
	IDENTIFICATION_REVISION_ID = 0xC2,

	OSC_CALIBRATE_VAL = 0xF8,

	GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
	GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

	GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
	DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
	DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
	POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

	VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

	ALGO_PHASECAL_LIM = 0x30,
	ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
} DistanceSensor_regAddr;

typedef enum {
	VcselPeriodPreRange, VcselPeriodFinalRange
} DistanceSensor_vcselPeriodType;

typedef struct {
	uint8_t address;
	I2C_HandleTypeDef hi2c;
	uint16_t io_timeout;
	uint8_t did_timeout;
	uint16_t timeout_start_ms;

	uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
	uint32_t measurement_timing_budget_us;
	uint8_t last_status;
	DistanceSensor_regAddr regAddr;
	DistanceSensor_vcselPeriodType vcselPeriodType;

} DistanceSensor;

void DistanceSensor_ctor(DistanceSensor* self, I2C_HandleTypeDef *hi2c);
void DistanceSensor_dtor(DistanceSensor* self);

volatile uint32_t DistanceSensor_counter;

void DistanceSensor_tick();
uint32_t DistanceSensor_millis();

void DistanceSensor_setAddress(DistanceSensor* self, uint8_t new_addr);
inline uint8_t DistanceSensor_getAddress(DistanceSensor* self) {
	return self->address;
}

uint8_t DistanceSensor_init(DistanceSensor* self);

void DistanceSensor_writeReg(DistanceSensor* self, uint8_t reg, uint8_t value);
void DistanceSensor_writeReg16Bit(DistanceSensor* self, uint8_t reg, uint16_t value);
void DistanceSensor_writeReg32Bit(DistanceSensor* self, uint8_t reg, uint32_t value);
uint8_t DistanceSensor_readReg(DistanceSensor* self, uint8_t reg);
uint16_t DistanceSensor_readReg16Bit(DistanceSensor* self, uint8_t reg);
uint32_t DistanceSensor_readReg32Bit(DistanceSensor* self, uint8_t reg);

void DistanceSensor_writeMulti(DistanceSensor* self, uint8_t reg, uint8_t * src, uint8_t count);
void DistanceSensor_readMulti(DistanceSensor* self, uint8_t reg, uint8_t * dst, uint8_t count);

uint8_t DistanceSensor_setSignalRateLimit(DistanceSensor *self, float limit_Mcps);
float DistanceSensor_getSignalRateLimit(DistanceSensor *self);

uint8_t DistanceSensor_setMeasurementTimingBudget(DistanceSensor *self, uint32_t budget_us);
uint32_t DistanceSensor_getMeasurementTimingBudget(DistanceSensor *self);

uint8_t DistanceSensor_setVcselPulsePeriod(DistanceSensor *self, DistanceSensor_vcselPeriodType type,
		uint8_t period_pclks);
uint8_t DistanceSensor_getVcselPulsePeriod(DistanceSensor *self, DistanceSensor_vcselPeriodType type);

void DistanceSensor_startContinuous(DistanceSensor *self, uint32_t period_ms);
void DistanceSensor_stopContinuous(DistanceSensor *self);
uint16_t DistanceSensor_readRangeContinuousMillimeters(DistanceSensor *self);
uint16_t DistanceSensor_readRangeSingleMillimeters(DistanceSensor *self);

inline void DistanceSensor_setTimeout(DistanceSensor* self, uint16_t timeout) {
	self->io_timeout = timeout;
}
inline uint16_t DistanceSensor_getTimeout(DistanceSensor* self) {
	return self->io_timeout;
}
uint8_t DistanceSensor_timeoutOccurred(DistanceSensor *self);

//PRIVATE

typedef struct {
	uint8_t tcc, msrc, dss, pre_range, final_range;
} DistanceSensor_SequenceStepEnables;

typedef struct {
	uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

	uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
	uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} DistanceSensor_SequenceStepTimeouts;

uint8_t DistanceSensor_getSpadInfo(DistanceSensor *self, uint8_t * count, uint8_t * type_is_aperture);

void DistanceSensor_getSequenceStepEnables(DistanceSensor *self, DistanceSensor_SequenceStepEnables * enables);
void DistanceSensor_getSequenceStepTimeouts(DistanceSensor *self, DistanceSensor_SequenceStepEnables const * enables,
		DistanceSensor_SequenceStepTimeouts * timeouts);

uint8_t performSingleRefCalibration(uint8_t vhv_init_byte);

static uint16_t DistanceSensor_decodeTimeout(uint16_t reg_val);
static uint16_t DistanceSensor_encodeTimeout(uint16_t timeout_mclks);
static uint32_t DistanceSensor_timeoutMclksToMicroseconds(
		uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t DistanceSensor_timeoutMicrosecondsToMclks(
		uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

#endif /* INC_DISTANCESENSOR_H_ */

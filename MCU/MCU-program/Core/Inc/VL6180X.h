/*
 * VL6180X.h
 *
 *  Created on: Apr 2022
 *      Author: Miłosz Gajewski
 */

#ifndef INC_VL6180X_H_
#define INC_VL6180X_H_

#include "main.h"
#include "i2c.h"
//
// Defines
//
// Default address
#define ADDRESS_DEFAULT_VL6180X (0x29)
// RANGE_SCALER values for 1x, 2x, 3x scaling
static const uint16_t ScalerValues[] = {0, 253, 127, 84};
// bool
typedef enum{
	TRUE = 1,
	FALSE = 0,
}bool;
// Registers
#define       IDENTIFICATION__MODEL_ID               0x000
#define       IDENTIFICATION__MODEL_REV_MAJOR        0x001
#define       IDENTIFICATION__MODEL_REV_MINOR        0x002
#define       IDENTIFICATION__MODULE_REV_MAJOR       0x003
#define       IDENTIFICATION__MODULE_REV_MINOR       0x004
#define       IDENTIFICATION__DATE_HI                0x006
#define       IDENTIFICATION__DATE_LO                0x007
#define       IDENTIFICATION__TIME                   0x008 // 16-bit
#define       SYSTEM__MODE_GPIO0                     0x010
#define       SYSTEM__MODE_GPIO1                     0x011
#define       SYSTEM__HISTORY_CTRL                   0x012
#define       SYSTEM__INTERRUPT_CONFIG_GPIO          0x014
#define       SYSTEM__INTERRUPT_CLEAR                0x015
#define       SYSTEM__FRESH_OUT_OF_RESET             0x016
#define       SYSTEM__GROUPED_PARAMETER_HOLD         0x017
#define       SYSRANGE__START                        0x018
#define       SYSRANGE__THRESH_HIGH                  0x019
#define       SYSRANGE__THRESH_LOW                   0x01A
#define       SYSRANGE__INTERMEASUREMENT_PERIOD      0x01B
#define       SYSRANGE__MAX_CONVERGENCE_TIME         0x01C
#define       SYSRANGE__CROSSTALK_COMPENSATION_RATE  0x01E // 16-bit
#define       SYSRANGE__CROSSTALK_VALID_HEIGHT       0x021
#define      SYSRANGE__EARLY_CONVERGENCE_ESTIMATE    0x022 // 16-bit
#define       SYSRANGE__PART_TO_PART_RANGE_OFFSET    0x024
#define       SYSRANGE__RANGE_IGNORE_VALID_HEIGHT    0x025
#define       SYSRANGE__RANGE_IGNORE_THRESHOLD       0x026 // 16-bit
#define       SYSRANGE__MAX_AMBIENT_LEVEL_MULT       0x02C
#define       SYSRANGE__RANGE_CHECK_ENABLES        	 0x02D
#define       SYSRANGE__VHV_RECALIBRATE              0x02E
#define       SYSRANGE__VHV_REPEAT_RATE              0x031
#define       SYSALS__START                          0x038
#define       SYSALS__THRESH_HIGH                    0x03A
#define       SYSALS__THRESH_LOW                     0x03C
#define       SYSALS__INTERMEASUREMENT_PERIOD        0x03E
#define       SYSALS__ANALOGUE_GAIN                  0x03F
#define       SYSALS__INTEGRATION_PERIOD             0x040
#define       RESULT__RANGE_STATUS                   0x04D
#define       RESULT__ALS_STATUS                     0x04E
#define       RESULT__INTERRUPT_STATUS_GPIO          0x04F
#define       RESULT__ALS_VAL                        0x050 // 16-bit
#define       RESULT__HISTORY_BUFFER_0               0x052 // 16-bit
#define       RESULT__HISTORY_BUFFER_1               0x054 // 16-bit
#define       RESULT__HISTORY_BUFFER_2               0x056 // 16-bit
#define       RESULT__HISTORY_BUFFER_3               0x058 // 16-bit
#define       RESULT__HISTORY_BUFFER_4               0x05A // 16-bit
#define       RESULT__HISTORY_BUFFER_5               0x05C // 16-bit
#define       RESULT__HISTORY_BUFFER_6               0x05E // 16-bit
#define       RESULT__HISTORY_BUFFER_7               0x060 // 16-bit
#define       RESULT__RANGE_VAL                      0x062
#define       RESULT__RANGE_RAW                      0x064
#define       RESULT__RANGE_RETURN_RATE              0x066 // 16-bit
#define       RESULT__RANGE_REFERENCE_RATE           0x068 // 16-bit
#define       RESULT__RANGE_RETURN_SIGNAL_COUNT      0x06C // 32-bit
#define       RESULT__RANGE_REFERENCE_SIGNAL_COUNT   0x070 // 32-bit
#define       RESULT__RANGE_RETURN_AMB_COUNT         0x074 // 32-bit
#define       RESULT__RANGE_REFERENCE_AMB_COUNT      0x078 // 32-bit
#define       RESULT__RANGE_RETURN_CONV_TIME         0x07C // 32-bit
#define       RESULT__RANGE_REFERENCE_CONV_TIME      0x080 // 32-bit
#define       RANGE_SCALER                           0x096 // 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h
#define       READOUT__AVERAGING_SAMPLE_PERIOD       0x10A
#define       FIRMWARE__BOOTUP                       0x119
#define       FIRMWARE__RESULT_SCALER                0x120
#define       I2C_SLAVE__DEVICE_ADDRESS              0x212
#define       INTERLEAVED_MODE__ENABLE               0x2A3
//
// Structure
//
typedef struct{
	I2C_HandleTypeDef* i2cHandle;
	uint8_t address;
	uint8_t scaling;
	uint8_t ptp_offset;
	uint16_t io_timeout;
	HAL_StatusTypeDef last_status;
	bool did_timeout;
}VL6180X_;
//
// Functions
//
// Write to register
void writeReg(VL6180X_* VL6180x,uint16_t reg, uint8_t value);
void writeReg16Bit(VL6180X_* VL6180x, uint16_t reg, uint16_t value);
// Read from register
uint8_t readReg(VL6180X_* VL6180x, uint16_t reg);
uint16_t readReg16Bit(VL6180X_* VL6180x, uint16_t reg);
// Initialize (constructor)
void VL6180X_Init(VL6180X_* VL6180x, I2C_HandleTypeDef* i2cHandle);
// I2C setter and getter
void setI2C_VL6180X(VL6180X_* VL6180x, I2C_HandleTypeDef* i2cHandle); //setBUS
I2C_HandleTypeDef* getBus_VL6180X(VL6180X_* VL6180x);
// Address setter and getter
void setAddress_VL6180X(VL6180X_* VL6180x,uint8_t new_address);
uint8_t getAddress_VL6180X(VL6180X_* VL6180x);
// Scaling setter and getter
void setScaling_VL6180X(VL6180X_* VL6180x, uint8_t new_scaling);
uint8_t getScaling_VL6180X(VL6180X_* VL6180x);
// Arduino constrain method
int16_t constrain_VL6180X(int16_t l1,int16_t l2, int16_t l3);
// Continuous functions
void startRangeContinuous_VL6180X(VL6180X_* VL6180x, uint16_t period);
void startAmbientContinuous_VL6180X(VL6180X_* VL6180x, uint16_t period);
void startInterleavedContinuous_VL6180X(VL6180X_* VL6180x, uint16_t period);
void stopContinuous_VL6180X(VL6180X_* VL6180x);
// Read continuous
uint8_t readRangeContinuous_VL6180X(VL6180X_* VL6180x);
uint16_t readRangeContinuousMillimeters_VL6180X(VL6180X_* VL6180x);
uint16_t readAmbientContinuous_VL6180X(VL6180X_* VL6180x);
// Read single
uint8_t readRangeSingle_VL6180X(VL6180X_* VL6180x);
uint16_t readRangeSingleMillimeters_VL6180X(VL6180X_* VL6180x);
uint16_t readAmbientSingle_VL6180X(VL6180X_* VL6180x);
// Default configuration
void configureDefault_VL6180X(VL6180X_* VL6180x);
// Timeout setter and getter
void setTimeout_VL6180X(VL6180X_* VL6180x, uint16_t timeout);
uint16_t getTimeout_VL6180X(VL6180X_* VL6180x);
bool timeoutOccured_VL6180X(VL6180X_* VL6180x);

#endif /* INC_VL6180X_H_ */

#ifndef _TSD305_REG_H_
#define _TSD305_REG_H_

#include "stm32fxxx_hal.h"
/* Replace the #include above for your specific STM32 family */

#include <string.h>
#include <stdio.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>

typedef union {
	uint16_t i16Value[2];
	uint32_t i32Value;
	float fValue;
}int_float;

/* Enumerate all status codes for TSD305 sensor */
typedef struct{
	uint16_t lot_number;
	uint16_t serial_number;

	int16_t min_ambient_temperature;
	int16_t max_ambient_temperature;
	int16_t min_object_temperature;
	int16_t max_object_temperature;

	float temp_coeff;
	float reff_temp;

	float k4_comp;
	float k3_comp;
	float k2_comp;
	float k1_comp;
	float k0_comp;

	float k4_obj;
	float k3_obj;
	float k2_obj;
	float k1_obj;
	float k0_obj;

	int16_t adc_calibration_factor;
}eeprom_coeff;

typedef enum {
	TSD305_STATUS_OK = 0x0U,
	TSD305_STATUS_BUSY = 0x1U,
	TSD305_I2C_ANOTHER_MASTER = 0x2U,
	TSD305_STATUS_I2C_TRANSF_ERR = 0x20U,
	TSD305_STATUS_NO_I2C_ACK = 0x4U,
	TSD305_STATUS_OUT_OF_RANGE = 0x5U
} TSD305_STATUS ;

typedef struct{
	int32_t i2c_status;
	TSD305_STATUS tsd305_status;
}status;


#define TSD305_ADDR 0x0

/* Sensor commands */
#define TSD305_CONVERT_ADCS_COMMAND 0xaf

#define TSD305_STATUS_BUSY_MASK 0x20
#define TSD305_STATUS_MEMOTY_ERROR_MASK 0x04

#define TSD305_CONVERSION_TIME 60 /* From datasheet, the worst time is 59.2ms */

/* Those functions must be placed in order to enable the communication */
int32_t tsd305_read(I2C_HandleTypeDef *i2cHandle, uint8_t *RxData, uint16_t len);
int32_t tsd305_write(I2C_HandleTypeDef *i2cHandle, uint8_t *TxData, uint16_t len);

/**
* \brief Reads the tsd305 EEPROM coefficient stored at address provided.
*
* \param[in] uint8_t : Address of coefficient in EEPROM
* \param[out] uint16_t* : Value read in EEPROM
*
* \return status: A struct that contains both I2C's and sensor's statuses
*/
status tsd305_read_eeprom_coeff(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint16_t *coeff);


/**
* \brief Reads the temperature and pressure ADC value and compute the
* compensated values.
*
* \param[out] float* : Celsius Degree temperature value
* \param[out] float* : mbar pressure value
*
* \return A struct that contains both I2C's and sensor's statuses
*/
status tsd305_read_temperature_and_object_temperature(I2C_HandleTypeDef *i2cHandle, float *temperature, float *object_temperature);

/**
* \brief Reads the tsd305 EEPROM coefficient stored at address provided.
*
* \param[in] uint8_t : Address of coefficient in EEPROM
* \param[out] float* : IEEE-745 Value read in EEPROM
*
* \return A struct that contains both I2C's and sensor's statuses
*/
status tsd305_read_eeprom_float(I2C_HandleTypeDef *i2cHandle, uint8_t address, float *value);


/**
* \brief Triggers conversion and read ADC value
*
* \param[in] uint8_t : Command used for conversion (will determine
* Temperature vs Pressure and osr)
* \param[out] uint32_t* : ADC value.
*
* \return A struct that contains both I2C's and sensor's statuses
*/
status tsd305_conversion_and_read_adcs(I2C_HandleTypeDef *i2cHandle, uint32_t *adc_object, uint32_t *adc_ambient);

/**
* \brief Reads the tsd305 EEPROM coefficients to store them for computation.
*
* \return A struct that contains both I2C's and sensor's statuses
*/
status tsd305_read_eeprom(I2C_HandleTypeDef *i2cHandle);

#endif /* SRC_TSD305_REG_H_ */

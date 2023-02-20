#include "tsd305_reg.h"

eeprom_coeff Eeprom_Coeff = {'\0'};
bool tsd305_coeff_read = false;

int32_t tsd305_write(I2C_HandleTypeDef *i2cHandle, uint8_t *TxData, uint16_t len) {
	HAL_I2C_Master_Transmit(i2cHandle, TSD305_ADDR, TxData, len, 1);
	return (HAL_I2C_GetError(i2cHandle));
}


int32_t tsd305_read(I2C_HandleTypeDef *i2cHandle, uint8_t *RxData, uint16_t len) {
	HAL_I2C_Master_Receive(i2cHandle, TSD305_ADDR, RxData, len, 1);
	return (HAL_I2C_GetError(i2cHandle));
}

status tsd305_read_eeprom_coeff(I2C_HandleTypeDef *i2cHandle, uint8_t address, uint16_t *coeff) {
	uint8_t buffer[3];
	status Status;

	Status.i2c_status = tsd305_write(i2cHandle, &address, 1);
	HAL_Delay(1);

	Status.i2c_status = tsd305_read(i2cHandle, buffer, 3);
	Status.tsd305_status = buffer[0];

	*coeff = (buffer[1] << 8) | buffer[2];

	return(Status);
}

status tsd305_read_eeprom_float(I2C_HandleTypeDef *i2cHandle, uint8_t address, float *value) {
	uint16_t h_word, l_word;
	int_float word;
	status Status;

	Status = tsd305_read_eeprom_coeff(i2cHandle, address, &h_word);
	if (Status.i2c_status != HAL_I2C_ERROR_NONE) {
		return(Status);
	}
	Status = tsd305_read_eeprom_coeff(i2cHandle, address + 1, &l_word);
	if (Status.i2c_status != HAL_I2C_ERROR_NONE) {
		return(Status);
	}

	word.i16Value[1] = h_word;
	word.i16Value[0] = l_word;

	*value = word.fValue;

	return(Status);
}

status tsd305_read_eeprom(I2C_HandleTypeDef *i2cHandle) {
	status Status;

	Status = tsd305_read_eeprom_coeff(i2cHandle, 0x00, &Eeprom_Coeff.lot_number);
	Status = tsd305_read_eeprom_coeff(i2cHandle, 0x01, &Eeprom_Coeff.serial_number);

	Status = tsd305_read_eeprom_coeff(i2cHandle, 0x1a,(uint16_t *)&Eeprom_Coeff.min_ambient_temperature);
	Status = tsd305_read_eeprom_coeff(i2cHandle, 0x1b,(uint16_t *)&Eeprom_Coeff.max_ambient_temperature);
	Status = tsd305_read_eeprom_coeff(i2cHandle, 0x1c,(uint16_t *)&Eeprom_Coeff.min_object_temperature);
	Status = tsd305_read_eeprom_coeff(i2cHandle, 0x1d,(uint16_t *)&Eeprom_Coeff.max_object_temperature);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x1e,&Eeprom_Coeff.temp_coeff);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x20,&Eeprom_Coeff.reff_temp);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x22,&Eeprom_Coeff.k4_comp);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x24,&Eeprom_Coeff.k3_comp);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x26,&Eeprom_Coeff.k2_comp);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x28,&Eeprom_Coeff.k1_comp);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x2a,&Eeprom_Coeff.k0_comp);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x2e,&Eeprom_Coeff.k4_obj);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x30,&Eeprom_Coeff.k3_obj);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x32,&Eeprom_Coeff.k2_obj);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x34,&Eeprom_Coeff.k1_obj);
	Status = tsd305_read_eeprom_float(i2cHandle, 0x36,&Eeprom_Coeff.k0_obj);

	tsd305_coeff_read = true;

	return(Status);
}

status tsd305_conversion_and_read_adcs(I2C_HandleTypeDef *i2cHandle, uint32_t *adc_object, uint32_t *adc_ambient) {
	uint8_t buffer[7] = {'\0'};
	status Status;
	uint8_t command = TSD305_CONVERT_ADCS_COMMAND;

	Status.i2c_status = tsd305_write(i2cHandle, &command, 1);
	HAL_Delay(TSD305_CONVERSION_TIME);

	Status.i2c_status = tsd305_read(i2cHandle, buffer, 7);

	if (Status.i2c_status != HAL_I2C_ERROR_NONE) {
		return(Status);
	}

	Status.tsd305_status = buffer[0];

	*adc_object = ((uint32_t)buffer[1]<<16) | ((uint32_t)buffer[2]<<8) | ((uint32_t)buffer[3]);
	*adc_ambient = ((uint32_t)buffer[4]<<16) | ((uint32_t)buffer[5]<<8) | ((uint32_t)buffer[6]);

	return(Status);
}

status tsd305_read_temperature_and_object_temperature(I2C_HandleTypeDef *i2cHandle, float *temperature, float *object_temperature) {
	int32_t adc_object, adc_ambient;
	float TCFactor=0;
	float offset_tc=0;
	int32_t adc_comp = 0;
	float adc_comp_tc = 0, t_object=0;
	status Status;

	if (!tsd305_coeff_read) {
		Status = tsd305_read_eeprom(i2cHandle);
		if (Status.i2c_status != HAL_I2C_ERROR_NONE) {
			return (Status);
		}
	}

	Status = tsd305_conversion_and_read_adcs(i2cHandle, (uint32_t*)&adc_object, (uint32_t*)&adc_ambient);
	if (Status.i2c_status != HAL_I2C_ERROR_NONE) {
		return (Status);
	}

	*temperature = adc_ambient/ (float)16777216 *
			((float)Eeprom_Coeff.max_ambient_temperature -
					(float)Eeprom_Coeff.min_ambient_temperature) +
					(float)Eeprom_Coeff.min_ambient_temperature;

	TCFactor = (*temperature - Eeprom_Coeff.reff_temp) * Eeprom_Coeff.temp_coeff;
	TCFactor++;

	offset_tc = Eeprom_Coeff.k4_comp * *temperature * *temperature * *temperature * *temperature
			+ Eeprom_Coeff.k3_comp * *temperature * *temperature * *temperature
			+ Eeprom_Coeff.k2_comp * *temperature * *temperature
			+ Eeprom_Coeff.k1_comp * *temperature
			+ Eeprom_Coeff.k0_comp;
	offset_tc = offset_tc * TCFactor;

	adc_comp = (int32_t) offset_tc + adc_object - 8388608;

	adc_comp_tc = (float)adc_comp / TCFactor;

	t_object = Eeprom_Coeff.k4_obj * adc_comp_tc * adc_comp_tc * adc_comp_tc * adc_comp_tc
			+ Eeprom_Coeff.k3_obj * adc_comp_tc * adc_comp_tc * adc_comp_tc
			+ Eeprom_Coeff.k2_obj * adc_comp_tc * adc_comp_tc
			+ Eeprom_Coeff.k1_obj * adc_comp_tc
			+ Eeprom_Coeff.k0_obj;

	*object_temperature = t_object;
	return(Status);
}

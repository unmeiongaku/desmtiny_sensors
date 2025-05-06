/*
 * im_bme680.h
 *
 *  Created on: Oct 31, 2023
 *      Author: nguye
 */

#ifndef APPLICATION_SENSOR_BME680_BME680_COMMON_H_
#define APPLICATION_SENSOR_BME680_BME680_COMMON_H_

#include <stdbool.h>
#include "bme68x.h"
#include "bme68x_defs.h"

typedef enum {
	MODE_SEQUENTIAL,
	PARALLEL_MODE,
	FORCED_MODE,
	NO_MODE_SELECTED
}bme680_mode_init_t;

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in] bme      : Structure instance of bme68x_dev
 *  @param[in] intf     : Interface selection parameter
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BME68X_INTF_RET_SUCCESS -> Success
 *  @retval != BME68X_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose value is to be written.
 *  @param[in] len          : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BME68X_INTF_RET_SUCCESS -> Success
 *  @retval != BME68X_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);

void bme68x_check_rslt(const char api_name[], int8_t rslt);

typedef struct{
	struct bme68x_dev gas_sensor;
	float temperature;
	float pressure;
	float humidity;
	uint32_t gas_resistance;
	uint16_t meas_period;
	float gas_weighting;
	float gas_reference;
	uint8_t getgasreference_count;
	float hum_weighting;
	float hum_reference;
	int32_t gas_lower_limit;
	int32_t gas_upper_limit;
}bme680_t;


#endif /* APPLICATION_SENSOR_BME680_BME680_COMMON_H_ */

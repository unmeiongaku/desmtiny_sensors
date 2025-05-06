/*
 * bme680_initialize.h
 *
 *  Created on: Oct 31, 2023
 *      Author: nguye
 */

#ifndef APPLICATION_SENSOR_BME680_BME680_INITIALIZE_H_
#define APPLICATION_SENSOR_BME680_BME680_INITIALIZE_H_

#include <stdio.h>
#include "bme68x.h"
#include "bme680_common.h"
#include "bme680_callback.h"


//typedef enum {
//	MODE_SEQUENTIAL,
//	PARALLEL_MODE,
//	FORCED_MODE,
//	NO_MODE_SELECTED
//}bme680_mode_init_t;


int8_t bme680_init(struct bme68x_dev *bme, struct bme68x_conf *conf, struct bme68x_heatr_conf *heatr_conf, bme680_mode_init_t mode_t);

#endif /* APPLICATION_SENSOR_BME680_BME680_INITIALIZE_H_ */

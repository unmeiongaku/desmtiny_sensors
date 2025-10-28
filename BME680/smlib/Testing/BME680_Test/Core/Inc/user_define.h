/*
 * user_define.h
 *
 *  Created on: May 14, 2025
 *      Author: nguye
 */

#include "i2c.h"

#ifndef INC_USER_DEFINE_H_
#define INC_USER_DEFINE_H_

#define BME680_STATE_MACHINE_PEROID 10
#define BME_REPORT_PEROID_MS 10

#define BME680_I2C 			hi2c1
#define I2C_TIMEOUT_MS			100

#define BME680_STATUS 									0
//HIGH 				1
//LOW 				0


/*BME680_DEFINE_FUNC*/
#if BME680_STATUS == 0
#define BME680_I2C_ADDR			(0x76 << 1)
#elif BME680_STATUS == 1
#define BME680_I2C_ADDR			(0x77 << 1)
#endif

#endif /* INC_USER_DEFINE_H_ */

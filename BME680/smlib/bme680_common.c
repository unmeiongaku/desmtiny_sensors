/*
 * im_bme680.c
 *
 *  Created on: Oct 31, 2023
 *      Author: nguye
 */

#include "user_define.h"
#include "bme680_common.h"
#include "bme68x.h"
#include "delay_us.h"

#define SAMPLE_COUNT  UINT8_C(300)

static uint8_t dev_addr;



/*User I2C communication*/
BME68X_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length){
	if(HAL_I2C_Mem_Write(&BME680_I2C,BME680_I2C_ADDR, reg_addr , I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, (uint16_t)length, I2C_TIMEOUT_MS)!= HAL_OK)
		return -1;
	return 0;
}

BME68X_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length){
	if(HAL_I2C_Mem_Read(&BME680_I2C,BME680_I2C_ADDR, reg_addr , I2C_MEMADD_SIZE_8BIT, reg_data, (uint16_t)length, I2C_TIMEOUT_MS)!= HAL_OK){
		*reg_data = 0;
		return -1;
	}
	return 0;
}

void bme68x_delay_us(uint32_t period)
{
    delay_us(period);
}


void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
#if ENABLE_TERMINAL == 1
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
#elif ENABLE_TERMINAL == 0
#endif
            break;
        case BME68X_E_COM_FAIL:
#if ENABLE_TERMINAL == 1
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
#elif ENABLE_TERMINAL == 0
#endif
            break;
        case BME68X_E_INVALID_LENGTH:
#if ENABLE_TERMINAL == 1
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
#elif ENABLE_TERMINAL == 0
#endif
            break;
        case BME68X_E_DEV_NOT_FOUND:
#if ENABLE_TERMINAL == 1
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
#elif ENABLE_TERMINAL == 0
#endif
            break;
        case BME68X_E_SELF_TEST:
#if ENABLE_TERMINAL == 1
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
#elif ENABLE_TERMINAL == 0
#endif
            break;
        case BME68X_W_NO_NEW_DATA:
#if ENABLE_TERMINAL == 1
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
#elif ENABLE_TERMINAL == 0
#endif
            break;
        default:
#if ENABLE_TERMINAL == 1
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
#elif ENABLE_TERMINAL == 0
#endif
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf){
	 int8_t rslt = BME68X_OK;
	 if (bme != NULL){
	 /* Bus configuration : I2C */
#if BME680_STATUS == 0
	 dev_addr = BME68X_I2C_ADDR_LOW;
#elif BME680_STATUS == 1
	 dev_addr = BME68X_I2C_ADDR_HIGH;
#endif
	 	 if (intf == BME68X_I2C_INTF){
#if ENABLE_TERMINAL == 1
	 		 printf("I2C Interface\n");
#elif ENABLE_TERMINAL == 0
#endif
	 		 bme->read = user_i2c_read;
	 		 bme->write = user_i2c_write;
	 		 bme->intf = BME68X_I2C_INTF;
	 	 }
 		 bme->delay_us = bme68x_delay_us;
 		 bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
	 }
	 else{
		 rslt = BME68X_E_NULL_PTR;
	 }
	 return rslt;
}



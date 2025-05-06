/*
 * bme680_initialize.c
 *
 *  Created on: Oct 31, 2023
 *      Author: nguye
 */

#include "bme680_initialize.h"

#define ENABLE_TERMINAL 0

int8_t bme680_init(struct bme68x_dev *bme, struct bme68x_conf *conf, struct bme68x_heatr_conf *heatr_conf, bme680_mode_init_t mode_t){
	int8_t rslt;
	if(mode_t == MODE_SEQUENTIAL){
		/* Heater temperature in degree Celsius */
		uint16_t temp_prof[10] = { 200, 240, 280, 320, 360, 360, 320, 280, 240, 200 };

		/* Heating duration in milliseconds */
		uint16_t dur_prof[10] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };
		/* Interface preference is updated as a parameter
		 * For I2C : BME68X_I2C_INTF
		 * For SPI : BME68X_SPI_INTF
		 */
		rslt = bme68x_interface_init(bme, BME68X_I2C_INTF);
		bme68x_check_rslt("bme68x_interface_init", rslt);

		rslt = bme68x_init(bme);
		bme68x_check_rslt("bme68x_init", rslt);
		/* Check if rslt == BME68X_OK, report or handle if otherwise */

		rslt = bme68x_get_conf(conf, bme);
		bme68x_check_rslt("bme68x_get_conf", rslt);

		conf->filter = BME68X_FILTER_OFF;
		conf->odr = BME68X_ODR_NONE; /* This parameter defines the sleep duration after each profile */
		conf->os_hum = BME68X_OS_16X;
		conf->os_pres = BME68X_OS_1X;
		conf->os_temp = BME68X_OS_2X;
		rslt = bme68x_set_conf(conf, bme);
		bme68x_check_rslt("bme68x_set_conf", rslt);

		/* Check if rslt == BME68X_OK, report or handle if otherwise */
		heatr_conf->enable = BME68X_ENABLE;
		heatr_conf->heatr_temp_prof = temp_prof;
		heatr_conf->heatr_dur_prof = dur_prof;
		heatr_conf->profile_len = 10;
		rslt = bme68x_set_heatr_conf(BME68X_SEQUENTIAL_MODE, heatr_conf, bme);
		bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

		/* Check if rslt == BME68X_OK, report or handle if otherwise */
		rslt = bme68x_set_op_mode(BME68X_SEQUENTIAL_MODE, bme);
		bme68x_check_rslt("bme68x_set_op_mode", rslt);
	#if ENABLE_TERMINAL == 1
		printf("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status, Profile index, Measurement index\n");
	#elif ENABLE_TERMINAL == 0
	#endif
	}
	else if(mode_t == PARALLEL_MODE){
	    /* Heater temperature in degree Celsius */
	    uint16_t temp_prof[10] = { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 };

	    /* Multiplier to the shared heater duration */
	    uint16_t mul_prof[10] = { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 };

	    /* Interface preference is updated as a parameter
	     * For I2C : BME68X_I2C_INTF
	     * For SPI : BME68X_SPI_INTF
	     */
	    rslt = bme68x_interface_init(bme, BME68X_I2C_INTF);
	    bme68x_check_rslt("bme68x_interface_init", rslt);

	    rslt = bme68x_init(bme);
	    bme68x_check_rslt("bme68x_init", rslt);

	    /* Check if rslt == BME68X_OK, report or handle if otherwise */
	    rslt = bme68x_get_conf(conf, bme);
	    bme68x_check_rslt("bme68x_get_conf", rslt);

	    /* Check if rslt == BME68X_OK, report or handle if otherwise */
	    conf->filter = BME68X_FILTER_OFF;
	    conf->odr = BME68X_ODR_NONE;
	    conf->os_hum = BME68X_OS_1X;
	    conf->os_pres = BME68X_OS_16X;
	    conf->os_temp = BME68X_OS_2X;
	    rslt = bme68x_set_conf(conf, bme);
	    bme68x_check_rslt("bme68x_set_conf", rslt);

	    /* Check if rslt == BME68X_OK, report or handle if otherwise */
	       heatr_conf->enable = BME68X_ENABLE;
	       heatr_conf->heatr_temp_prof = temp_prof;
	       heatr_conf->heatr_dur_prof = mul_prof;

	       /* Shared heating duration in milliseconds */
	       heatr_conf->shared_heatr_dur = (uint16_t)(140 - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, conf, bme) / 1000));

	       heatr_conf->profile_len = 10;
	       rslt = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, heatr_conf, bme);
	       bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

	       /* Check if rslt == BME68X_OK, report or handle if otherwise */
	       rslt = bme68x_set_op_mode(BME68X_PARALLEL_MODE, bme);
	       bme68x_check_rslt("bme68x_set_op_mode", rslt);
		#if ENABLE_TERMINAL == 1
	       printf(
	           "Print parallel mode data if mask for new data(0x80), gas measurement(0x20) and heater stability(0x10) are set\n\n");
	    #elif ENABLE_TERMINAL == 0
	    #endif
	    #if ENABLE_TERMINAL == 1
	       /* Check if rslt == BME68X_OK, report or handle if otherwise */
	       printf(
	           "Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status, Gas index, Meas index\n");
		#elif ENABLE_TERMINAL == 0
		#endif
	}
	else if(mode_t == FORCED_MODE){
	    /* Interface preference is updated as a parameter
	     * For I2C : BME68X_I2C_INTF
	     * For SPI : BME68X_SPI_INTF
	     */
	    rslt = bme68x_interface_init(bme, BME68X_I2C_INTF);
	    bme68x_check_rslt("bme68x_interface_init", rslt);

	    rslt = bme68x_init(bme);
	    bme68x_check_rslt("bme68x_init", rslt);

	    /* Check if rslt == BME68X_OK, report or handle if otherwise */
	    conf->filter = BME68X_FILTER_OFF;
	    conf->odr = BME68X_ODR_NONE;
	    conf->os_hum = BME68X_OS_16X;
	    conf->os_pres = BME68X_OS_1X;
	    conf->os_temp = BME68X_OS_2X;
	    rslt = bme68x_set_conf(conf, bme);
	    bme68x_check_rslt("bme68x_set_conf", rslt);

	    /* Check if rslt == BME68X_OK, report or handle if otherwise */
	    heatr_conf->enable = BME68X_ENABLE;
	    heatr_conf->heatr_temp = 300;
	    heatr_conf->heatr_dur = 100;
	    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, heatr_conf, bme);
	    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
	#if ENABLE_TERMINAL == 1
	    printf("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status\n");
	#elif ENABLE_TERMINAL == 0
	#endif
	}
	else{
		return -1;
	}
    return 0;
}


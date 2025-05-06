/*
 * bme680_callback.h
 *
 *  Created on: Nov 1, 2023
 *      Author: nguye
 */

#ifndef APPLICATION_SENSOR_BME680_BME680_CALLBACK_H_
#define APPLICATION_SENSOR_BME680_BME680_CALLBACK_H_

#include "bme680_initialize.h"
#include "bme680_common.h"

typedef struct{
	bool bme_isCurr_Delay;
	bool saveData;
	uint32_t bme_cnt;
	uint32_t bme_current_cnt;
	uint8_t delay_cnt;
}bme_cb_function_t;

typedef enum{
	SEQUENTIAL_REGISTER,
	FORCED_REGISTER,
	PARALLEL_REGISTER,
	CALIBRATION_REGISTER,
}bme_register_t;

typedef enum{
	SEQUENTIAL_UNREGISTER,
	FORCED_UNREGISTER,
	PARALLEL_UNREGISTER,
	CALIBRATION_UNREGISTER,
	ALL_UNREGISTER,
}bme_unregister_t;

typedef enum{
	TEMPERATURE_REGISTER,
	HUMIDITY_REGISTER,
	AIR_QUALITY_REGISTER,
	PRESSURE_REGISTER,
	ALTITUTE_REGISTER,
	BME_ALL_REPORT_CALLBACK_REGISTER,
}bme_register_report_callback_t;

typedef enum{
	TEMPERATURE_UNREGISTER,
	HUMIDITY_UNREGISTER,
	AIR_QUALITY_UNREGISTER,
	PRESSURE_UNREGISTER,
	ALTITUTE_UNREGISTER,
	BME_ALL_REPORT_CALLBACK_UNREGISTER,
}bme_unregister_report_callback_t;

int bme680_init_callback(bme680_mode_init_t init_mode);
void bme680_get_calib_data(uint16_t *par_temperature_1,int16_t *par_temperature_2,int8_t *par_temperature_3 , uint16_t par_humidity_1_2[2],int8_t par_humidity_3_4_5_7[4],uint8_t *par_humidity_6 ,int8_t par_gas_sensor[3],uint16_t *par_pressure_1,int16_t par_pressure_2_4_5_8_9[5],int8_t par_pressure_3_6_7[3],uint8_t *par_pressure_10);

//void bme680_get_data_mode_sequential();
//void bme680_get_data_mode_forced();
//void bme680_get_data_mode_parallel();
void bme_register_callback(bme_register_t bme_register);
void bme_unregister_callback(bme_unregister_t bme_unregister);

void bme_register_report_callback(bme_register_report_callback_t bme_register);
void bme_unregister_report_callback(bme_unregister_report_callback_t bme_unregister);

float bme_get_temp();
float bme_get_pressure();
float bme_get_humidity();
float bme_get_gas_resistance();
float bme_get_altitude_from_body_to_base();
float bme_get_altitude_from_sea_level();

uint8_t bme_get_status();
uint8_t bme_get_gas_index();
uint8_t bme_get_meas_index();
uint8_t bme_get_res_heat();

uint16_t bme_get_par_temperature_1();
int16_t bme_get_par_temperature_2();
int8_t bme_get_par_temperature_3();

uint16_t bme_get_par_humidity_1();
uint16_t bme_get_par_humidity_2();
int8_t bme_get_par_humidity_3();
int8_t bme_get_par_humidity_4();
int8_t bme_get_par_humidity_5();
uint8_t bme_get_par_humidity_6();
int8_t bme_get_par_humidity_7();

int8_t bme_get_par_gas_sensor_1();
int8_t bme_get_par_gas_sensor_2();
int8_t bme_get_par_gas_sensor_3();

uint16_t bme_get_par_pressure_1();
int16_t bme_get_par_pressure_2();
int8_t bme_get_par_pressure_3();
int16_t bme_get_par_pressure_4();
int16_t bme_get_par_pressure_5();
int8_t bme_get_par_pressure_6();
int16_t bme_get_par_pressure_8();
int8_t bme_get_par_pressure_7();
int16_t bme_get_par_pressure_9();
uint8_t bme_get_par_pressure_10();

void update_cnt_bme(bme_cb_function_t *bme_cb, uint32_t sys_tick);

#endif /* APPLICATION_SENSOR_BME680_BME680_CALLBACK_H_ */

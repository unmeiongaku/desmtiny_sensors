/*
 * bme680_callback.c
 *
 *  Created on: Nov 1, 2023
 *      Author: nguye
 */

#include "bme680_callback.h"
#include "bme68x.h"
#include "math.h"
#include "params.h"
#include "mav.h"
#include "stm32h7xx_it.h"

#define BME68X_VALID_DATA  UINT8_C(0xB0)
bme_cb_function_t bme_cb;

TIDD(gtid_bme_mode_sequential_callback);
TIDD(gtid_bme_mode_force_callback);
TIDD(gtid_bme_mode_parallel_callback);
TID(gtid_bme_calibration_callback);

TID(gtid_bme_temperature_report_callback);
TID(gtid_bme_pressure_report_callback);
TID(gtid_bme_air_quality_report_callback);
TID(gtid_bme_humidity_report_callback);
TID(gtid_bme_altitute_report_callback);

static struct bme68x_dev bme;
static struct bme68x_conf conf;
static struct bme68x_heatr_conf heatr_conf;
static struct bme68x_calib_data data_calib;
static struct bme68x_data data_parallel[3];
static struct bme68x_data data_sequential[3];
static struct bme68x_data data_forced;

//uint32_t time_ms = 0;
//static uint32_t time_ms_cb = 0;

/*! Temperature in degree celsius */
static float temperature_cb;

/*! Pressure in Pascal */
static float pressure_cb;

/*! Humidity in % relative humidity x1000 */
static float humidity_cb;

/*! Gas resistance in Ohms */
static float gas_resistance_cb;

/*! Contains new_data, gasm_valid & heat_stab */
static uint8_t status_cb;

/*! The index of the heater profile used */
static uint8_t gas_index_cb;

/*! Measurement index to track order */
static uint8_t meas_index_cb;

/*! Heater resistance */
static uint8_t res_heat_cb;

static float altitude_sea_level,altitude_sea_level_base;

static float altitude_body,pressure_base;

static bool get_1st_press = false;


/*calibration static bme680*/
static uint16_t par_temperature_1;
static int16_t par_temperature_2;
static int8_t par_temperature_3 ;
static uint16_t par_humidity_1_2[2];
static int8_t par_humidity_3_4_5_7[4];
static uint8_t par_humidity_6 ;
static int8_t par_gas_sensor[3];
static uint16_t par_pressure_1;
static int16_t par_pressure_2_4_5_8_9[5];
static int8_t par_pressure_3_6_7[3];
static uint8_t par_pressure_10;

uint32_t del_period;
static uint8_t n_fields;
int8_t rslt;

void buzzer_nofti_bme_success(uint8_t time_t);

void buzzer_nofti_bme_false(uint16_t time_t);

void update_cnt_bme(bme_cb_function_t *bme_cb, uint32_t sys_tick){
	bme_cb->bme_current_cnt = sys_tick;
}

void buzzer_nofti_bme_false(uint16_t time_t){
	HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin, SET);
	delay_ms(time_t);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin, RESET);
	delay_ms(time_t);
}

void buzzer_nofti_bme_success(uint8_t time_t){
	HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin, SET);
	delay_ms(time_t);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin, RESET);
	delay_ms(time_t);
}

int bme680_init_callback(bme680_mode_init_t init_mode){
	if(bme680_init(&bme,&conf,&heatr_conf,init_mode)!=0){
	buzzer_nofti_bme_false(1000);
#if ENABLE_TERMINAL == 1
	printf("BME680 Initialize false\n");
#elif ENABLE_TERMINAL == 0
#endif
	}
	else{
		bme_cb.bme_isCurr_Delay = false;
		bme_cb.saveData = false;
		bme_cb.delay_cnt = 15;
		buzzer_nofti_bme_success(100);
	}
	return 0;
}

static void bme680_get_data_mode_sequential_callback(void* ctx){
	if(bme_cb.bme_isCurr_Delay == false){
		del_period = bme68x_get_meas_dur(BME68X_SEQUENTIAL_MODE, &conf, &bme) + (heatr_conf.heatr_dur_prof[0] * 1000);
		bme.delay_us(del_period); // 10ms is enough;
		bme_cb.bme_isCurr_Delay = true;
		bme_cb.bme_cnt = getsyscnt();
	}
	if(bme_cb.bme_current_cnt >= bme_cb.bme_cnt && bme_cb.bme_current_cnt - bme_cb.bme_cnt >= bme_cb.delay_cnt){
		bme_cb.saveData = true;
	}
	if(bme_cb.saveData == true){
		rslt = bme68x_get_data(BME68X_SEQUENTIAL_MODE, data_sequential, &n_fields, &bme);
		bme68x_check_rslt("bme68x_get_data", rslt);
		for (uint8_t i = 0; i < n_fields; i++){
	#ifdef BME68X_USE_FPU
			//time_ms_cb = (long unsigned int)time_ms + (i * (del_period / 2000));
			temperature_cb = data_sequential[i].temperature;
			pressure_cb = data_sequential[i].pressure;
			humidity_cb = data_sequential[i].humidity;
			gas_resistance_cb = data_sequential[i].gas_resistance;
			status_cb = data_sequential[i].status;
			gas_index_cb = data_sequential[i].gas_index;
			meas_index_cb = data_sequential[i].meas_index;
			res_heat_cb = data_sequential[i].res_heat;
	#else
			temperature_cb = data_sequential[i].temperature/100;
			pressure_cb = (long unsigned int)data_sequential[i].pressure;
			humidity_cb = (long unsigned int)data_sequential[i].humidity/1000;
			gas_resistance_cb = (long unsigned int)data_sequential[i].gas_resistance;
			status_cb = data_sequential[i].status;
			gas_index_cb = data_sequential[i].gas_index;
			meas_index_cb = data_sequential[i].meas_index;
			res_heat_cb = data_sequential[i].res_heat;
	#endif
			bme_cb.saveData = false;
			bme_cb.bme_isCurr_Delay = false;
		}
    }
}

static void bme680_get_data_mode_parallel_callback(void* ctx){
	del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) + (heatr_conf.shared_heatr_dur * 1000);
	bme.delay_us(del_period);

	rslt = bme68x_get_data(BME68X_PARALLEL_MODE, data_parallel, &n_fields, &bme);
	bme68x_check_rslt("bme68x_get_data", rslt);

	 for (uint8_t i = 0; i < n_fields; i++){
		 if (data_parallel[i].status == BME68X_VALID_DATA){
#ifdef BME68X_USE_FPU
		temperature_cb = data_parallel[i].temperature;
		pressure_cb = data_parallel[i].pressure;
		humidity_cb = data_parallel[i].humidity;
		gas_resistance_cb = data_parallel[i].gas_resistance;
		status_cb = data_parallel[i].status;
		gas_index_cb = data_parallel[i].gas_index;
		meas_index_cb = data_parallel[i].meas_index;
		res_heat_cb = data_parallel[i].res_heat;
#else
	    temperature_cb = data_parallel[i].temperature/100;
	    pressure_cb = (long unsigned int)data_parallel[i].pressure;
	    humidity_cb = (long unsigned int)data_parallel[i].humidity/1000;
	    gas_resistance_cb = (long unsigned int)data_parallel[i].gas_resistance;
	    status_cb = data_parallel[i].status;
	    gas_index_cb = data_parallel[i].gas_index;
	    meas_index_cb = data_parallel[i].meas_index;
	    res_heat_cb = data_parallel[i].res_heat;
#endif
		 }
	 }
}

static void bme680_get_data_mode_forced_callback(void* ctx){
	if(bme_cb.bme_isCurr_Delay == false){
		rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
		bme68x_check_rslt("bme68x_set_op_mode", rslt);
		del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
		//bme.delay_us(del_period);
		bme_cb.bme_isCurr_Delay = true;
	}
	else if(bme_cb.bme_isCurr_Delay == true){
		bme_cb.saveData = true;
	}
	if(bme_cb.bme_isCurr_Delay == true && bme_cb.saveData == true){
    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_get_data(BME68X_FORCED_MODE,&data_forced, &n_fields, &bme);
    bme68x_check_rslt("bme68x_get_data", rslt);
		if (n_fields){
#ifdef BME68X_USE_FPU
			temperature_cb = data_forced.temperature;
			pressure_cb = data_forced.pressure;
			humidity_cb = data_forced.humidity;
			gas_resistance_cb = data_forced.gas_resistance;
			status_cb = data_forced.status;
			gas_index_cb = data_forced.gas_index;
			meas_index_cb = data_forced.meas_index;
			res_heat_cb = data_forced.res_heat;
#else
			temperature_cb = data_forced.temperature/100;
			pressure_cb = (long unsigned int)data_forced.pressure;
			humidity_cb = (long unsigned int)data_forced.humidity/1000;
			gas_resistance_cb = (long unsigned int)data_forced.gas_resistance;
			status_cb = data_forced.status;
			gas_index_cb = data_forced.gas_index;
			meas_index_cb = data_forced.meas_index;
			res_heat_cb = data_forced.res_heat;
#endif
			bme_cb.bme_isCurr_Delay = false;
			bme_cb.saveData = false;
		}
    }
}

void bme680_get_calib_data(uint16_t *par_temperature_1,int16_t *par_temperature_2,int8_t *par_temperature_3 , uint16_t par_humidity_1_2[2],int8_t par_humidity_3_4_5_7[4],uint8_t *par_humidity_6 ,int8_t par_gas_sensor[3],uint16_t *par_pressure_1,int16_t par_pressure_2_4_5_8_9[5],int8_t par_pressure_3_6_7[3],uint8_t *par_pressure_10){

/*Calibration data from par_temperature*/
	*par_temperature_1 = data_calib.par_t1;
	*par_temperature_2 = data_calib.par_t2;
	*par_temperature_3 = data_calib.par_t3;
/*Calibration data from par_humidity*/
	par_humidity_1_2[0] = data_calib.par_h1;
	par_humidity_1_2[1] = data_calib.par_h2;
	par_humidity_3_4_5_7[0] = data_calib.par_h3;
	par_humidity_3_4_5_7[1] = data_calib.par_h4;
	par_humidity_3_4_5_7[2] = data_calib.par_h5;
	*par_humidity_6 = data_calib.par_h6;
	par_humidity_3_4_5_7[3] = data_calib.par_h7;
/*Calibration data from par_gas_sensor*/
	par_gas_sensor[0] = data_calib.par_gh1;
	par_gas_sensor[1] = data_calib.par_gh2;
	par_gas_sensor[2] = data_calib.par_gh2;
/*Calibration data from par_gas_sensor*/
	*par_pressure_1 = data_calib.par_p1;
	par_pressure_2_4_5_8_9[0] = data_calib.par_p2;
	par_pressure_3_6_7[0] = data_calib.par_p3;
	par_pressure_2_4_5_8_9[1] = data_calib.par_p4;
	par_pressure_2_4_5_8_9[2] = data_calib.par_p5;
	par_pressure_3_6_7[1] = data_calib.par_p6;
	par_pressure_3_6_7[2] = data_calib.par_p7;
	par_pressure_2_4_5_8_9[3] = data_calib.par_p8;
	par_pressure_2_4_5_8_9[4] = data_calib.par_p9;
	*par_pressure_10 = data_calib.par_p10;
}

static void bme680_get_calibration_data_callback(void* ctx){
//	uint16_t par_temperature_1;
//	int16_t par_temperature_2;
//	int8_t par_temperature_3 ;
//	uint16_t par_humidity_1_2[2];
//	int8_t par_humidity_3_4_5_7[4];
//	uint8_t par_humidity_6 ;
//	int8_t par_gas_sensor[3];
//	uint16_t par_pressure_1;
//	int16_t par_pressure_2_4_5_8_9[5];
//	int8_t par_pressure_3_6_7[3];
//	uint8_t par_pressure_10;
	bme680_get_calib_data(&par_temperature_1, &par_temperature_2, &par_temperature_3, par_humidity_1_2, par_humidity_3_4_5_7, &par_humidity_6, par_gas_sensor, &par_pressure_1, par_pressure_2_4_5_8_9, par_pressure_3_6_7, &par_pressure_10);

}

float bme_get_altitude_from_sea_level(){
	altitude_sea_level = (params.T0_temp/params.lapse_rate)*(1.0f-pow((pressure_cb/100)/params.SEA_Level_Pressure,0.1903));
	return altitude_sea_level;
}

float bme_get_altitude_from_body_to_base(){
	if(get_1st_press == false){
		if(pressure_cb != 0){
		pressure_base = pressure_cb;
		altitude_sea_level_base = (params.T0_temp/params.lapse_rate)*(1.0f-pow((pressure_base/100)/params.SEA_Level_Pressure,0.1903));
		get_1st_press = true;
		}
		else{
			return 0;
		}
	}
	/*Copmute new altitude*/
	altitude_body = altitude_sea_level - altitude_sea_level_base;
	return altitude_body;
}

void bme_register_callback(bme_register_t bme_register){
	switch(bme_register){
	case SEQUENTIAL_REGISTER:
		gtid_bme_mode_sequential_callback =timer_register_delay_callback(bme680_get_data_mode_sequential_callback, SEQUENTIAL_PEROID_MS, 0, TIMER_MODE_REPEAT);
		break;
	case FORCED_REGISTER:
		gtid_bme_mode_force_callback = timer_register_delay_callback(bme680_get_data_mode_forced_callback, FORCED_PEROID_MS, 0, TIMER_MODE_REPEAT);
		break;
	case PARALLEL_REGISTER:
		gtid_bme_mode_parallel_callback = timer_register_delay_callback(bme680_get_data_mode_parallel_callback, PARALLEL_PEROID_MS, 0, TIMER_MODE_REPEAT);
		break;
	case CALIBRATION_REGISTER:
		gtid_bme_calibration_callback = timer_register_callback(bme680_get_calibration_data_callback, CALIBRATION_PEROID_MS, 0, TIMER_MODE_ONE_SHOT);
		break;
	}
}

void bme_unregister_callback(bme_unregister_t bme_unregister){
	switch(bme_unregister){
	case SEQUENTIAL_UNREGISTER:
		timer_unregister_delay_callback(gtid_bme_mode_sequential_callback);
		break;
	case FORCED_UNREGISTER:
		timer_unregister_delay_callback(gtid_bme_mode_force_callback);
		break;
	case PARALLEL_UNREGISTER:
		timer_unregister_delay_callback(gtid_bme_mode_parallel_callback);
		break;
	case CALIBRATION_UNREGISTER:
		timer_unregister_callback(gtid_bme_calibration_callback);
		break;
	case ALL_UNREGISTER:
		timer_unregister_callback(gtid_bme_mode_sequential_callback);
		timer_unregister_callback(gtid_bme_mode_sequential_callback);
		timer_unregister_callback(gtid_bme_mode_parallel_callback);
		timer_unregister_callback(gtid_bme_calibration_callback);
		break;
	}
}

float bme_get_temp(){
	return temperature_cb;
}

float bme_get_pressure(){
	return pressure_cb;
}

float bme_get_humidity(){
	return humidity_cb;
}

float bme_get_gas_resistance(){
	return gas_resistance_cb;
}

uint8_t bme_get_status(){
	return status_cb;
}

uint8_t bme_get_gas_index(){
	return gas_index_cb;
}

uint8_t bme_get_meas_index(){
	return meas_index_cb;
}

uint8_t bme_get_res_heat(){
	return res_heat_cb;
}

uint16_t bme_get_par_temperature_1(){
	return par_temperature_1;
	}
int16_t bme_get_par_temperature_2(){
	return par_temperature_2;
}
int8_t bme_get_par_temperature_3(){
	return par_temperature_3;
}

uint16_t bme_get_par_humidity_1(){
	return par_humidity_1_2[0];
}

uint16_t bme_get_par_humidity_2(){
	return par_humidity_1_2[1];
}

int8_t bme_get_par_humidity_3(){
	return par_humidity_3_4_5_7[0];
}
int8_t bme_get_par_humidity_4(){
	return par_humidity_3_4_5_7[1];
}
int8_t bme_get_par_humidity_5(){
	return par_humidity_3_4_5_7[2];
}
int8_t bme_get_par_humidity_7(){
	return par_humidity_3_4_5_7[3];
}

uint8_t bme_get_par_humidity_6(){
	return par_humidity_6;
}

int8_t bme_get_par_gas_sensor_1(){
	return par_gas_sensor[0];
}
int8_t bme_get_par_gas_sensor_2(){
	return par_gas_sensor[1];
}
int8_t bme_get_par_gas_sensor_3(){
	return par_gas_sensor[2];
}

uint16_t bme_get_par_pressure_1(){
	return par_pressure_1;
}
int16_t bme_get_par_pressure_2(){
	return par_pressure_2_4_5_8_9[0];
}
int16_t bme_get_par_pressure_4(){
	return par_pressure_2_4_5_8_9[1];
}

int16_t bme_get_par_pressure_5(){
	return par_pressure_2_4_5_8_9[2];
}
int16_t bme_get_par_pressure_8(){
	return par_pressure_2_4_5_8_9[3];
}
int16_t bme_get_par_pressure_9(){
	return par_pressure_2_4_5_8_9[4];
}

int8_t bme_get_par_pressure_3(){
	return par_pressure_3_6_7[0];
}
int8_t bme_get_par_pressure_6(){
	return par_pressure_3_6_7[1];
}
int8_t bme_get_par_pressure_7(){
	return par_pressure_3_6_7[2];
}

uint8_t bme_get_par_pressure_10(){
	return par_pressure_10;
}

static void bme_temperature_report_callback(void *ctx){
	mavlink_message_t msg;
	float temp = 0.0;
	temp = bme_get_temp();
	mavlink_msg_temperature_bm680_pack(0, 0, &msg, temp);
	mav_send_msg(&msg);
}

static void bme_pressure_report_callback(void *ctx){
	mavlink_message_t msg;
	float pressure;
	pressure = bme_get_pressure();
	mavlink_msg_pressure_pack(0, 0, &msg, pressure);
	mav_send_msg(&msg);
}

static void bme_air_quality_report_callback(void *ctx){
	mavlink_message_t msg;
	float air_quality;
	air_quality = bme_get_gas_resistance();
	mavlink_msg_air_quality_pack(0, 0, &msg, air_quality);
	mav_send_msg(&msg);
}

static void bme_humidity_report_callback(void *ctx){
	mavlink_message_t msg;
	float humidity;
	humidity = bme_get_humidity();
	mavlink_msg_humidity_pack(0,0, &msg, humidity);
	mav_send_msg(&msg);
}

static void bme_altitute_report_callback(void *ctx){
	mavlink_message_t msg;
	float bme_altitute_to_sea_level;
	float bme_altitute_set_point;
	float bme_altitute;
	float temperature_sea_level;
	bme_altitute_to_sea_level = params.SEA_Level_Pressure;
	bme_altitute_set_point = altitude_sea_level_base;
	bme_altitute = bme_get_altitude_from_body_to_base();
	temperature_sea_level = params.T0_temp;
	mavlink_msg_bme_altitude_pack(0, 0, &msg, bme_altitute_to_sea_level, bme_altitute_set_point, bme_altitute, temperature_sea_level);
	mav_send_msg(&msg);
}

void bme_register_report_callback(bme_register_report_callback_t bme_register){
	switch(bme_register){
	case TEMPERATURE_REGISTER:
	{
		gtid_bme_temperature_report_callback = timer_register_callback(bme_temperature_report_callback, BME_TEMP_PEROID_MS,0, TIMER_MODE_REPEAT);
	}
		break;
	case HUMIDITY_REGISTER:
	{
		gtid_bme_humidity_report_callback = timer_register_callback(bme_humidity_report_callback, BME_HUMIDITY_PEROID_MS,0, TIMER_MODE_REPEAT);
	}
		break;
	case AIR_QUALITY_REGISTER:
	{
		gtid_bme_air_quality_report_callback = timer_register_callback(bme_air_quality_report_callback, BME_AIR_QUALITY_PEROID_MS,0, TIMER_MODE_REPEAT);
	}
		break;
	case PRESSURE_REGISTER:
	{
		gtid_bme_pressure_report_callback = timer_register_callback(bme_pressure_report_callback, BME_PRESSURE_PEROID_MS,0, TIMER_MODE_REPEAT);
	}
		break;
	case ALTITUTE_REGISTER:
	{
		gtid_bme_altitute_report_callback = timer_register_callback(bme_altitute_report_callback, BME_ALTITUTE_PEROID_MS, 0, TIMER_MODE_REPEAT);
	}
		break;
	case BME_ALL_REPORT_CALLBACK_REGISTER:
	{
		gtid_bme_temperature_report_callback = timer_register_callback(bme_temperature_report_callback, BME_TEMP_PEROID_MS,0, TIMER_MODE_REPEAT);
		gtid_bme_humidity_report_callback = timer_register_callback(bme_humidity_report_callback, BME_HUMIDITY_PEROID_MS,0, TIMER_MODE_REPEAT);
		gtid_bme_air_quality_report_callback = timer_register_callback(bme_air_quality_report_callback, BME_AIR_QUALITY_PEROID_MS,0, TIMER_MODE_REPEAT);
		gtid_bme_pressure_report_callback = timer_register_callback(bme_pressure_report_callback, BME_PRESSURE_PEROID_MS,0, TIMER_MODE_REPEAT);
		gtid_bme_altitute_report_callback = timer_register_callback(bme_altitute_report_callback, BME_ALTITUTE_PEROID_MS, 0, TIMER_MODE_REPEAT);
	}
		break;
	}
}
void bme_unregister_report_callback(bme_unregister_report_callback_t bme_unregister){
	switch(bme_unregister){
	case TEMPERATURE_UNREGISTER:
	{
		timer_unregister_callback(gtid_bme_temperature_report_callback);
	}
		break;
	case HUMIDITY_UNREGISTER:
	{
		timer_unregister_callback(gtid_bme_humidity_report_callback);
	}
		break;
	case AIR_QUALITY_UNREGISTER:
	{
		timer_unregister_callback(gtid_bme_air_quality_report_callback);
	}
		break;
	case PRESSURE_UNREGISTER:
	{
		timer_unregister_callback(gtid_bme_pressure_report_callback);
	}
		break;
	case ALTITUTE_UNREGISTER:
	{
		timer_unregister_callback(gtid_bme_altitute_report_callback);
	}
		break;
	case BME_ALL_REPORT_CALLBACK_UNREGISTER:
	{
		timer_unregister_callback(gtid_bme_temperature_report_callback);
		timer_unregister_callback(gtid_bme_humidity_report_callback);
		timer_unregister_callback(gtid_bme_air_quality_report_callback);
		timer_unregister_callback(gtid_bme_pressure_report_callback);
		timer_unregister_callback(gtid_bme_altitute_report_callback);

	}
		break;
	}
}

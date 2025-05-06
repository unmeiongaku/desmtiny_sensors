/*
 * bme_active_object.c
 *
 *  Created on: Nov 18, 2024
 *      Author: nguye
 */

#include "bme68x.h"
#include "bme680_common.h"
#include "user_define.h"
#include "bme_active_object.h"
#include "math.h"
#include "params.h"
#include "bme680_initialize.h"
#include "delay_us.h"

#define BME68X_VALID_DATA  UINT8_C(0xB0)

static bme_event_status_t bme_proobject_state_handle_BME_HEATING_SENSORS_SM(bme_proobject_t *const mobj, bme_event_t const *const e);
static bme_event_status_t bme_proobject_state_handle_BME_READING_SENSORS_SM(bme_proobject_t *const mobj, bme_event_t const *const e);

static uint8_t bme_internal_signal;

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


/*static saving data*/
static bme_calib_value_t bme_calib;


///*calibration static bme680*/
//static uint16_t par_temperature_1;
//static int16_t par_temperature_2;
//static int8_t par_temperature_3 ;
//static uint16_t par_humidity_1_2[2];
//static int8_t par_humidity_3_4_5_7[4];
//static uint8_t par_humidity_6 ;
//static int8_t par_gas_sensor[3];
//static uint16_t par_pressure_1;
//static int16_t par_pressure_2_4_5_8_9[5];
//static int8_t par_pressure_3_6_7[3];
//static uint8_t par_pressure_10;


void led_nofti_bme_false(uint16_t time_t){
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, SET);
	delay_ms(time_t);
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, RESET);
	delay_ms(time_t);
}

void led_nofti_bme_success(uint8_t time_t){
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, SET);
	delay_ms(time_t);
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, RESET);
	delay_ms(time_t);
}

void bme_proobject_init(bme_proobject_t *const mobj){
	bme_event_t ee;
	ee.sig = BME_ENTRY;
	mobj->active_state = BME_HEATING_SENSORS_SM;
	mobj->bme_mode = FORCED_MODE;
	mobj->ticksigcnt = 0;
	mobj->IsGetCalibrationDataYet = false;
	if(bme680_init(&mobj->bme,&mobj->conf,&mobj->heatr_conf, mobj->bme_mode)==0){
		led_nofti_bme_success(100);
	}
	bme_proobject_state_machine(mobj,&ee);
}
bme_event_status_t bme_proobject_state_machine(bme_proobject_t *const mobj, bme_event_t const * const e){
	switch(mobj->active_state){
	case BME_HEATING_SENSORS_SM:
	{
		return bme_proobject_state_handle_BME_HEATING_SENSORS_SM(mobj, e);
	}
		break;
	case BME_READING_SENSORS_SM:
	{
		bme_proobject_state_handle_BME_READING_SENSORS_SM(mobj, e);
	}
		break;
	}
	return BME_EVENT_IGNORED;
}

static bme_event_status_t bme_proobject_state_handle_BME_HEATING_SENSORS_SM(bme_proobject_t *const mobj, bme_event_t const *const e){
	switch(e->sig){
		case BME_ENTRY:
		{
			if(mobj->bme_mode ==MODE_SEQUENTIAL){
				mobj->del_period = bme68x_get_meas_dur(BME68X_SEQUENTIAL_MODE,&mobj->conf,&mobj->bme) + (mobj->heatr_conf.heatr_dur_prof[0]*1000);
			}
			else if(mobj->bme_mode ==PARALLEL_MODE){
				bme68x_set_op_mode(BME68X_PARALLEL_MODE, &mobj->bme);
				mobj->del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE,&mobj->conf,&mobj->bme) + (mobj->heatr_conf.shared_heatr_dur*1000);
			}
			else if(mobj->bme_mode ==FORCED_MODE){
				bme68x_set_op_mode(BME68X_FORCED_MODE, &mobj->bme);
				mobj->del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE,&mobj->conf,&mobj->bme) + (mobj->heatr_conf.heatr_dur*1000);
			}
		}
			return BME_EVENT_HANDLED;
		case BME_EXIT:
		{
			mobj->ticksigcnt = 0;
		}
			return BME_EVENT_HANDLED;
		case BME_TICK_SIG:
		{
			/*Delay 30ms before going next state*/
			if(mobj->ticksigcnt >= BME680_STATE_MACHINE_PEROID*3){
				mobj->ticksigcnt = 0;
				bme_internal_signal = BME_GET_DATA_DEFINE;
			}
			else{
				mobj->ticksigcnt+=10;
			}
		}
			return BME_EVENT_HANDLED;
		case BME_HEATING_SIG:
		{
			mobj->active_state = BME_READING_SENSORS_SM;
			return BME_EVENT_TRANSITION;
		}
	}
	return BME_EVENT_IGNORED;
}

static bme_event_status_t bme_proobject_state_handle_BME_READING_SENSORS_SM(bme_proobject_t *const mobj, bme_event_t const *const e){
	switch(e->sig){
	case BME_ENTRY:
	{
		if(mobj->bme_mode ==MODE_SEQUENTIAL){
			bme68x_get_data(BME68X_SEQUENTIAL_MODE,mobj->data_sequential, &mobj->n_fields, &mobj->bme);
#ifdef BME68X_USE_FPU
		//time_ms_cb = (long unsigned int)time_ms + (i * (del_period / 2000));
			for (uint8_t i = 0; i < mobj->n_fields; i++){
				temperature_cb = mobj->data_sequential[i].temperature;
				pressure_cb = mobj->data_sequential[i].pressure;
				humidity_cb = mobj->data_sequential[i].humidity;
				gas_resistance_cb = mobj->data_sequential[i].gas_resistance;
				status_cb = mobj->data_sequential[i].status;
				gas_index_cb = mobj->data_sequential[i].gas_index;
				meas_index_cb = mobj->data_sequential[i].meas_index;
				res_heat_cb = mobj->data_sequential[i].res_heat;
#else
		temperature_cb = mobj->data_sequential[i].temperature/100;
		pressure_cb = (long unsigned int)mobj->data_sequential[i].pressure;
		humidity_cb = (long unsigned int)mobj->data_sequential[i].humidity/1000;
		gas_resistance_cb = (long unsigned int)mobj->data_sequential[i].gas_resistance;
		status_cb = mobj->data_sequential[i].status;
		gas_index_cb = mobj->data_sequential[i].gas_index;
		meas_index_cb = mobj->data_sequential[i].meas_index;
		res_heat_cb = mobj->data_sequential[i].res_heat;
#endif
			}
		}
		else if(mobj->bme_mode ==PARALLEL_MODE){
			bme68x_get_data(BME68X_PARALLEL_MODE,mobj->data_parallel, &mobj->n_fields, &mobj->bme);
			for (uint8_t i = 0; i < mobj->n_fields; i++){
					 if (mobj->data_parallel[i].status == BME68X_VALID_DATA){
			#ifdef BME68X_USE_FPU
					temperature_cb = mobj->data_parallel[i].temperature;
					pressure_cb = mobj->data_parallel[i].pressure;
					humidity_cb = mobj->data_parallel[i].humidity;
					gas_resistance_cb = mobj->data_parallel[i].gas_resistance;
					status_cb = mobj->data_parallel[i].status;
					gas_index_cb = mobj->data_parallel[i].gas_index;
					meas_index_cb = mobj->data_parallel[i].meas_index;
					res_heat_cb = mobj->data_parallel[i].res_heat;
			#else
				    temperature_cb = mobj->data_parallel[i].temperature/100;
				    pressure_cb = (long unsigned int)mobj->data_parallel[i].pressure;
				    humidity_cb = (long unsigned int)mobj->data_parallel[i].humidity/1000;
				    gas_resistance_cb = (long unsigned int)mobj->data_parallel[i].gas_resistance;
				    status_cb = mobj->data_parallel[i].status;
				    gas_index_cb = mobj->data_parallel[i].gas_index;
				    meas_index_cb = mobj->data_parallel[i].meas_index;
				    res_heat_cb = mobj->data_parallel[i].res_heat;
			#endif
					 }
				 }
		}
		else if(mobj->bme_mode ==FORCED_MODE){
			bme68x_get_data(BME68X_FORCED_MODE,&mobj->data_forced, &mobj->n_fields, &mobj->bme);
			if(mobj->n_fields){
#ifdef BME68X_USE_FPU
			temperature_cb = mobj->data_forced.temperature;
			pressure_cb = mobj->data_forced.pressure;
			humidity_cb = mobj->data_forced.humidity;
			gas_resistance_cb = mobj->data_forced.gas_resistance;
			status_cb = mobj->data_forced.status;
			gas_index_cb = mobj->data_forced.gas_index;
			meas_index_cb = mobj->data_forced.meas_index;
			res_heat_cb = mobj->data_forced.res_heat;
#else
			temperature_cb = mobj->data_forced.temperature/100;
			pressure_cb = (long unsigned int)mobj->data_forced.pressure;
			humidity_cb = (long unsigned int)mobj->data_forced.humidity/1000;
			gas_resistance_cb = (long unsigned int)mobj->data_forced.gas_resistance;
			status_cb = mobj->data_forced.status;
			gas_index_cb = mobj->data_forced.gas_index;
			meas_index_cb = mobj->data_forced.meas_index;
			res_heat_cb = mobj->data_forced.res_heat;
#endif
			}
		}
		if(mobj->IsGetCalibrationDataYet == false){
			bme_calib.par_temperature_1  = mobj->data_calib.par_t1;
			bme_calib.par_temperature_2  = mobj->data_calib.par_t2;
			bme_calib.par_temperature_3  = mobj->data_calib.par_t3;
			/*Calibration data from par_humidity*/
			bme_calib.par_humidity_1_2[0]  = mobj->data_calib.par_h1;
			bme_calib.par_humidity_1_2[1]  = mobj->data_calib.par_h2;
			bme_calib.par_humidity_3_4_5_7[0]  = mobj->data_calib.par_h3;
			bme_calib.par_humidity_3_4_5_7[1]  = mobj->data_calib.par_h4;
			bme_calib.par_humidity_3_4_5_7[2]  = mobj->data_calib.par_h5;
			bme_calib.par_humidity_6 = mobj->data_calib.par_h6;
			bme_calib.par_humidity_3_4_5_7[3]  = mobj->data_calib.par_h7;
			/*Calibration data from par_gas_sensor*/
			bme_calib.par_gas_sensor[0] = mobj->data_calib.par_gh1;
			bme_calib.par_gas_sensor[1] = mobj->data_calib.par_gh2;
			bme_calib.par_gas_sensor[2] = mobj->data_calib.par_gh2;
			/*Calibration data from par_gas_sensor*/
			bme_calib.par_pressure_1 = mobj->data_calib.par_p1;
			bme_calib.par_pressure_2_4_5_8_9[0] = mobj->data_calib.par_p2;
			bme_calib.par_pressure_3_6_7[0] = mobj->data_calib.par_p3;
			bme_calib.par_pressure_2_4_5_8_9[1] = mobj->data_calib.par_p4;
			bme_calib.par_pressure_2_4_5_8_9[2] = mobj->data_calib.par_p5;
			bme_calib.par_pressure_3_6_7[1] = mobj->data_calib.par_p6;
			bme_calib.par_pressure_3_6_7[2] = mobj->data_calib.par_p7;
			bme_calib.par_pressure_2_4_5_8_9[3] = mobj->data_calib.par_p8;
			bme_calib.par_pressure_2_4_5_8_9[4] = mobj->data_calib.par_p9;
			bme_calib.par_pressure_10 = mobj->data_calib.par_p10;
			mobj->IsGetCalibrationDataYet = true;
		}
		return BME_EVENT_HANDLED;
	}
	case BME_EXIT:
	{
		return BME_EVENT_HANDLED;
	}
	case BME_TICK_SIG:
	{
		return BME_EVENT_HANDLED;
	}
	case BME_HEATING_SIG:
		mobj->active_state = BME_HEATING_SENSORS_SM;
		return BME_EVENT_TRANSITION;
	}
	return BME_EVENT_IGNORED;
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
	return bme_calib.par_temperature_1;
	}
int16_t bme_get_par_temperature_2(){
	return bme_calib.par_temperature_2;
}
int8_t bme_get_par_temperature_3(){
	return bme_calib.par_temperature_3;
}

uint16_t bme_get_par_humidity_1(){
	return bme_calib.par_humidity_1_2[0];
}

uint16_t bme_get_par_humidity_2(){
	return bme_calib.par_humidity_1_2[1];
}

int8_t bme_get_par_humidity_3(){
	return bme_calib.par_humidity_3_4_5_7[0];
}
int8_t bme_get_par_humidity_4(){
	return bme_calib.par_humidity_3_4_5_7[1];
}
int8_t bme_get_par_humidity_5(){
	return bme_calib.par_humidity_3_4_5_7[2];
}
int8_t bme_get_par_humidity_7(){
	return bme_calib.par_humidity_3_4_5_7[3];
}

uint8_t bme_get_par_humidity_6(){
	return bme_calib.par_humidity_6;
}

int8_t bme_get_par_gas_sensor_1(){
	return bme_calib.par_gas_sensor[0];
}
int8_t bme_get_par_gas_sensor_2(){
	return bme_calib.par_gas_sensor[1];
}
int8_t bme_get_par_gas_sensor_3(){
	return bme_calib.par_gas_sensor[2];
}

uint16_t bme_get_par_pressure_1(){
	return bme_calib.par_pressure_1;
}
int16_t bme_get_par_pressure_2(){
	return bme_calib.par_pressure_2_4_5_8_9[0];
}
int16_t bme_get_par_pressure_4(){
	return bme_calib.par_pressure_2_4_5_8_9[1];
}

int16_t bme_get_par_pressure_5(){
	return bme_calib.par_pressure_2_4_5_8_9[2];
}
int16_t bme_get_par_pressure_8(){
	return bme_calib.par_pressure_2_4_5_8_9[3];
}
int16_t bme_get_par_pressure_9(){
	return bme_calib.par_pressure_2_4_5_8_9[4];
}

int8_t bme_get_par_pressure_3(){
	return bme_calib.par_pressure_3_6_7[0];
}
int8_t bme_get_par_pressure_6(){
	return bme_calib.par_pressure_3_6_7[1];
}
int8_t bme_get_par_pressure_7(){
	return bme_calib.par_pressure_3_6_7[2];
}

uint8_t bme_get_par_pressure_10(){
	return bme_calib.par_pressure_10;
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

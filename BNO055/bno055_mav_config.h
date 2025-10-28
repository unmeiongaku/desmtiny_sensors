/*
 * bno055_mav_config.h
 *
 *  Created on: Oct 15, 2025
 *      Author: nguye
 */

#ifndef BNO055_BNO055_MAV_CONFIG_H_
#define BNO055_BNO055_MAV_CONFIG_H_

#include "bno055_register_map.h"
#include "mav.h"
#include "mavlink.h"
#include "stdbool.h"

typedef enum{
	BN0055_SYS_RECONFIG = 1,
	BN0055_SYS_CALIBRATION = 2,
	BN0055_SYS_INIT = 3,
	BN0055_SYS_SUSPEND = 4,
	BN0055_SYS_LOW_POWER = 5,
	BN0055_SYS_DEFAULT = 6,
	BN0055_SYS_READ_DATA = 7,
}bno55_system_config_type_t;

typedef struct{
	bno55_system_config_type_t systype;
	uint8_t sys_v;
}bno55_sys_config_global_type_t;

typedef struct{
	bno055_opr_mode_t oprmode;
	uint8_t oprmode_v;
}bno55_oprmode_global_t;

typedef struct{
	fusion_data_output_systems_t fusiondata;
	uint8_t fusion_data_v;
}bno55_fusion_output_data_global_t;

typedef struct{
	bno055_axis_remap_config_t axis_remap_config;
	uint8_t remap_config_data_v;
}bno55_axis_remap_config_global_t;

typedef struct{
	bno055_axis_remap_sign_t axis_remap_sign;
	uint8_t remap_sign_data_v;
}bno55_axis_remap_sign_global_t;

typedef struct{
	power_mode_t pwr_mode;
	uint8_t pwr_mode_v;
}bno55_pwr_mode_global_t;

typedef struct{
	temp_source_t temp_src;
	uint8_t temp_src_v;
}bno55_temp_src_global_t;

typedef struct{
	uint8_t acc_g_range;
	uint8_t acc_bandwidth;
	uint8_t acc_mode;
	uint8_t gyr_range;
	uint8_t gyr_bandwidth;
	uint8_t gyr_mode;
	uint8_t mag_data_rate;
	uint8_t mag_operation_mode;
	uint8_t mag_pwr_mode;
	uint8_t acc_linearacc_gravityvector_unit;
	uint8_t angular_rate_gyr_unit;
	uint8_t euler_angles_unit;
	uint8_t temp_unit;
	uint8_t fusion_dof;
}bno055_acc_gyr_mag_valuation_t;

typedef struct{
	uint8_t sys;
	uint8_t gyr;
	uint8_t acc;
	uint8_t mag;
	bool  indicatesFullyCalibrated;
}bno055_calib_stat_t;

typedef struct{
	float x;
	float y;
	float z;
}bno055_acceleration_t;

typedef struct{
	float x;
	float y;
	float z;
}bno055_gyroscope_t;

typedef struct{
	float x;
	float y;
	float z;
}bno055_magnetometer_t;

typedef struct{
	float x;
	float y;
	float z;
}bno055_gravity_vector_t;

typedef struct{
	float x;
	float y;
	float z;
}bno055_linear_acceleration_t;

typedef struct{
	float x;
	float y;
	float z;
	float w;
}bno055_quaternion_t;

typedef struct{
	float roll;
	float pitch;
	float heading;
}bno055_euler_angles_t;

typedef struct{
	bno055_calib_stat_t sts;
	float magradius;
	float accradius;

	float accxoffset;
	float accyoffset;
	float acczoffset;

	float magxoffset;
	float magyoffset;
	float magzoffset;

	float gyrxoffset;
	float gyryoffset;
	float gyrzoffset;

	bool isOnCalibration;
}bno055_calibration_t;

typedef struct{
	uint8_t lastpageid;
	uint8_t bl_rev_id;
	uint8_t SW_REV_ID;
	uint8_t GYR_ID;
	uint8_t MAG_ID;
	uint8_t ACC_ID;
	uint8_t CHIP_ID;
}bno055_id_t;

typedef enum{
	PAGE_ID_0 = 0,
	PAGE_ID_1 = 1,
}bno055_page_id_t;

typedef struct{
	float acc_lia_grv;
	float angular_rate_gyr;
	float mag;
	float euler;
	float qua;
	float temp;
}bno055_scale_t;

typedef struct{
	bno055_page_id_t pageid;
	bno55_sys_config_global_type_t sys;
	bno55_oprmode_global_t oprmode;
	bno55_fusion_output_data_global_t fusionod;
	bno55_axis_remap_config_global_t axisremapconfig;
	bno55_axis_remap_sign_global_t axisremapsign;
	bno55_pwr_mode_global_t pwrmode;
	bno55_temp_src_global_t temp_src;
	bno055_acc_gyr_mag_valuation_t acc_gyr_mag_valuation;
	bno055_scale_t scale;
	uint8_t tmp;
}mav_bno055_config_t;

typedef struct{
	int8_t sys_error;
	int8_t sys_status;
	int8_t sys_clk_status;
	int8_t int_sta_acc_nm;
	int8_t int_sta_acc_am;
	int8_t int_sta_acc_high_g;
	int8_t int_sta_gyro_high_rate;
	int8_t int_sta_gyro_am;
	int8_t ST_RESULT_ST_MCU;
	int8_t ST_RESULT_ST_GYR;
	int8_t ST_RESULT_ST_MAG;
	int8_t ST_RESULT_ST_ACC;
}bno_sys_status_t;

typedef struct{
	/*DeviceOk*/
	bool isDeviceWasFound;
	bno_sys_status_t sts;
	/*Configuration*/
	mav_bno055_config_t config;
	mav_bno055_config_t mav_config;
	/*Variable*/
	bno055_id_t id;
	bno055_acceleration_t acc;
	bno055_gyroscope_t gyr;
	bno055_magnetometer_t mag;
	bno055_calibration_t calib;
	float temperature;
	bno055_gravity_vector_t grv;
	bno055_linear_acceleration_t lia;
	bno055_quaternion_t qua;
	bno055_euler_angles_t eul;
}bno_global_var_t;

uint8_t bno_default_initization(bno_global_var_t *bno);

void bno_mav_do();
void bno_mav_configuration_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config);
void bno_sys_decode_type(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config);
void bno_opr_mode_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config);
void bno_fusion_data_output_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config);
void bno_axis_remap_config_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config);
void bno_axis_remap_sign_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config);
void bno_pwr_mode_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config);
void bno_temp_src_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config);
void bno_acc_gyro_mag_valuation_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config);

uint8_t bno_axis_remap_sign(mav_bno055_config_t *config, bno055_axis_remap_sign_t axis_remap_sign);
uint8_t bno_axis_remap_config(mav_bno055_config_t *config, bno055_axis_remap_config_t axis_remap);
uint8_t bno_acc_config(mav_bno055_config_t *config,uint8_t g_range,uint8_t Bandwidth,uint8_t OPRMode);
uint8_t bno_gyro_config(mav_bno055_config_t *config,uint8_t g_range,uint8_t Bandwidth,uint8_t OPRMode);
uint8_t bno_mag_config(mav_bno055_config_t *config,uint8_t data_rate,uint8_t OPRMode,uint8_t PWRMode);
uint8_t bno_set_pwr_mode(mav_bno055_config_t *config, power_mode_t pwr_mode);
uint8_t bno_set_operation_mode(mav_bno055_config_t *config, bno055_opr_mode_t mode);
void bno_set_page_id(mav_bno055_config_t *config, bno055_page_id_t id);
uint8_t bno_read_chip_id(bno_global_var_t *bno);
uint8_t bno_reset(mav_bno055_config_t *config);
uint8_t bno_set_unit(mav_bno055_config_t *config,uint8_t acc,uint8_t angular,uint8_t euler, uint8_t temp, uint8_t  fusion_dof);
uint8_t bno_set_temperature_src(mav_bno055_config_t *config,temp_source_t src);

int8_t bno_calib_stat(bno_global_var_t *bno);
int8_t bno_get_calibration_offset(bno_global_var_t *bno);
int8_t bno_get_accel_gyro(bno_global_var_t *bno);
int8_t bno_get_mag(bno_global_var_t *bno);
int8_t bno_get_temp(bno_global_var_t *bno);
int8_t bno_get_elu_data(bno_global_var_t *bno);
int8_t bno_get_qua_data(bno_global_var_t *bno);
int8_t bno_get_lia_data(bno_global_var_t *bno);
int8_t bno_get_grv_data(bno_global_var_t *bno);

int8_t bno_read_sys_error(bno_global_var_t *bno);
int8_t bno_read_sys_status(bno_global_var_t *bno);
int8_t bno_read_clk_status(bno_global_var_t *bno);
int8_t bno_read_int_sta(bno_global_var_t *bno);
int8_t bno_read_st_result(bno_global_var_t *bno);

#endif /* BNO055_BNO055_MAV_CONFIG_H_ */

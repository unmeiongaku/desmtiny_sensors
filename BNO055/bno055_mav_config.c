/*
 * bno055_mav_config.c
 *
 *  Created on: Oct 15, 2025
 *      Author: nguye
 */

#define MAV_ENABLE 0

#include "bno055_register_map.h"
#include "bno055_mav_config.h"

#if  MAV_ENABLE == 0
#include "mav.h"
#include "mavlink.h"
#endif

static int8_t bno_read(uint8_t addr, uint8_t *value){
	if(HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDR, addr , I2C_MEMADD_SIZE_8BIT, value, 1, I2C_TIMEOUT_MS)!= HAL_OK){
		*value = 0;
		return -1;
	}
	return 0;
}

static int8_t bno_write(uint8_t addr, uint8_t *value){
	if(HAL_I2C_Mem_Write(&BNO055_I2C, BNO055_I2C_ADDR, addr , I2C_MEMADD_SIZE_8BIT, value, 1, I2C_TIMEOUT_MS)!= HAL_OK)
		return -1;
	return 0;
}

static int8_t bno_read_bytes(uint8_t addr, uint8_t *value, uint8_t len){
	if(HAL_I2C_Mem_Read(&BNO055_I2C, BNO055_I2C_ADDR, addr , I2C_MEMADD_SIZE_8BIT, value, len, I2C_TIMEOUT_MS)!= HAL_OK){
		memset(value,0,len);
		return -1;
	}
	return 0;
}

void  bno_isDeviceReady(bno_global_var_t *bno) {
	if(HAL_I2C_IsDeviceReady(&BNO055_I2C, BNO055_I2C_ADDR, 1, 100) == HAL_OK){
		bno->isDeviceWasFound = true;
	}
	else{
		bno->isDeviceWasFound = false;
	}
}

void bno_set_default_init(mav_bno055_config_t *config){
	config->sys.systype = BN0055_SYS_DEFAULT;
	/*default id page is 0*/
	uint8_t tmp;
	config->pageid = PAGE_ID_0;
	tmp = (uint8_t)config->pageid;
	bno_write(PAGE_ID,&tmp);
}

uint8_t bno_default_initization(bno_global_var_t *bno){
	bno_isDeviceReady(bno);
	if(bno->isDeviceWasFound){
		/*Default init*/
		bno_set_default_init(&bno->config);
		/*Reset CHIP*/
		bno_reset(&bno->config);
		/*Set Operation Mode to CONFIG Mode*/
		bno_set_operation_mode(&bno->config,BNO055_OPERATION_CONFIG_MODE);
		/*Read Chip ID*/
		bno_read_chip_id(bno);
		/*Set Axis Remap Config*/
		bno_axis_remap_config(&bno->config, REMAP_CONFIG_P1_2_4_7);
		/*Set Axis Remap Sign*/
		bno_axis_remap_sign(&bno->config, REMAP_SIGN_P1);
		/*Set Power Mode*/
		bno_set_pwr_mode(&bno->config, NORMAL_MODE);
		/*acceleration configuration*/
		bno_acc_config(&bno->config, ACC_4G_RANGE, ACC_BANDWIDTH_62_5HZ, ACC_OPERATION_MODE_NORMAL);
		/*gyroscope configuration*/
		bno_gyro_config(&bno->config, GYR_2000DPS_RANGE, GYR_BANDWIDTH_32HZ, GYR_OPERATION_MODE_NORMAL);
		/*mag configuration*/
		bno_mag_config(&bno->config, MAG_20HZ_RANGE, MAG_OPERATION_MODE_REGULAR, MAG_PWR_MODE_FORCE_MODE);
		/*Set unit*/
		bno_set_unit(&bno->config, ACC_LIA_GRVVector_MS2, ANGULAR_RATE_GYR_DPS, EULER_ANGLES_DEGREES, TEMP_C, FUSION_DATA_OUTPUT_FORMAT_ANDROID);
		/*Set Temperature Source*/
		bno_set_temperature_src(&bno->config,TEMP_SRC_ACCELEROMETER);
		/*Move to page 0 for reading*/
		bno_set_page_id(&bno->config, PAGE_ID_0);
		/*Delay*/
		delay_ms(100);
		/*Set Operation Mode*/
		bno_set_operation_mode(&bno->config,BNO055_OPERATION_MODE_NDOF);
		delay_ms(50);
		return 1;
	}
	return 0;
}

uint8_t bno_mav_initization(bno_global_var_t *bno){
	bno_isDeviceReady(bno);
	if(bno->isDeviceWasFound){

		return 1;
	}
	return 0;
}

uint8_t bno_axis_remap_sign(mav_bno055_config_t *config, bno055_axis_remap_sign_t axis_remap_sign){
	bno_set_page_id(config, PAGE_ID_0);
	uint8_t tmp;
	switch(axis_remap_sign){
	case REMAP_SIGN_P0:
	tmp = REMAP_SIGN_P0,
	bno_write(AXIS_MAP_SIGN,&tmp);
	break;
	case REMAP_SIGN_P1:
	tmp = REMAP_SIGN_P1,
	bno_write(AXIS_MAP_SIGN,&tmp);
	break;
	case REMAP_SIGN_P2:
	tmp = REMAP_SIGN_P2,
	bno_write(AXIS_MAP_SIGN,&tmp);
	break;
	case REMAP_SIGN_P3:
	tmp = REMAP_SIGN_P3,
	bno_write(AXIS_MAP_SIGN,&tmp);
	break;
	case REMAP_SIGN_P4:
	tmp = REMAP_SIGN_P4,
	bno_write(AXIS_MAP_SIGN,&tmp);
	break;
	case REMAP_SIGN_P5:
	tmp = REMAP_SIGN_P5,
	bno_write(AXIS_MAP_SIGN,&tmp);
	break;
	case REMAP_SIGN_P6:
	tmp = REMAP_SIGN_P6,
	bno_write(AXIS_MAP_SIGN,&tmp);
	break;
	case REMAP_SIGN_P7:
	tmp = REMAP_SIGN_P7,
	bno_write(AXIS_MAP_SIGN,&tmp);
	break;
	}
	return 1;
}

uint8_t bno_axis_remap_config(mav_bno055_config_t *config, bno055_axis_remap_config_t axis_remap){
	bno_set_page_id(config, PAGE_ID_0);
	uint8_t tmp;
	switch(axis_remap){
	case REMAP_CONFIG_P0_3_5_6:
	config->axisremapconfig.axis_remap_config = REMAP_CONFIG_P0_3_5_6;
	tmp = REMAP_CONFIG_P0_3_5_6,
	bno_write(AXIS_MAP_CONFIG,&tmp);
	break;
	case REMAP_CONFIG_P1_2_4_7:
	config->axisremapconfig.axis_remap_config = REMAP_CONFIG_P1_2_4_7;
	tmp = REMAP_CONFIG_P1_2_4_7,
	bno_write(AXIS_MAP_CONFIG,&tmp);
	break;
	}
	return 1;
}


uint8_t bno_acc_config(mav_bno055_config_t *config,uint8_t g_range,uint8_t Bandwidth,uint8_t OPRMode){
	bno_set_page_id(config, PAGE_ID_1);
	config->acc_gyr_mag_valuation.acc_g_range = g_range;
	config->acc_gyr_mag_valuation.acc_bandwidth = Bandwidth;
	config->acc_gyr_mag_valuation.acc_mode = OPRMode;
	uint8_t tmp;
	tmp = (config->acc_gyr_mag_valuation.acc_mode | config->acc_gyr_mag_valuation.acc_bandwidth)| config->acc_gyr_mag_valuation.acc_g_range;
	bno_write(ACC_CONFIG,&tmp);
	return 1;
}

uint8_t bno_gyro_config(mav_bno055_config_t *config,uint8_t g_range,uint8_t Bandwidth,uint8_t OPRMode){
	bno_set_page_id(config, PAGE_ID_1);
	config->acc_gyr_mag_valuation.gyr_range = g_range;
	config->acc_gyr_mag_valuation.gyr_bandwidth = Bandwidth;
	config->acc_gyr_mag_valuation.gyr_mode = OPRMode;
	uint8_t tmp_cf0,tmp_cf1;
	tmp_cf0 =  config->acc_gyr_mag_valuation.gyr_bandwidth|config->acc_gyr_mag_valuation.gyr_range;
	bno_write(GYR_CONFIG_0,&tmp_cf0);
	tmp_cf1 = config->acc_gyr_mag_valuation.gyr_mode;
	bno_write(GYR_CONFIG_1,&tmp_cf1);
	return 1;
}

uint8_t bno_mag_config(mav_bno055_config_t *config,uint8_t data_rate,uint8_t OPRMode,uint8_t PWRMode){
	bno_set_page_id(config, PAGE_ID_1);
	config->acc_gyr_mag_valuation.mag_data_rate = data_rate;
	config->acc_gyr_mag_valuation.mag_operation_mode = OPRMode;
	config->acc_gyr_mag_valuation.mag_pwr_mode = PWRMode;
	uint8_t tmp;
	tmp =  (config->acc_gyr_mag_valuation.mag_pwr_mode|config->acc_gyr_mag_valuation.mag_operation_mode) | config->acc_gyr_mag_valuation.mag_data_rate;
	bno_write(GYR_CONFIG_0,&tmp);
	return 1;
}

uint8_t bno_set_unit(mav_bno055_config_t *config,uint8_t acc,uint8_t angular,uint8_t euler, uint8_t temp, uint8_t  fusion_dof){
	bno_set_page_id(config, PAGE_ID_0);

	config->acc_gyr_mag_valuation.acc_linearacc_gravityvector_unit = acc;
	config->acc_gyr_mag_valuation.angular_rate_gyr_unit = angular;
	config->acc_gyr_mag_valuation.euler_angles_unit = euler;
	config->acc_gyr_mag_valuation.temp_unit = temp;
	config->acc_gyr_mag_valuation.fusion_dof = fusion_dof;
	uint8_t tmp;
	tmp = config->acc_gyr_mag_valuation.fusion_dof |
	      config->acc_gyr_mag_valuation.temp_unit |
	      config->acc_gyr_mag_valuation.euler_angles_unit |
	      config->acc_gyr_mag_valuation.angular_rate_gyr_unit |
	      config->acc_gyr_mag_valuation.acc_linearacc_gravityvector_unit;
	bno_write(UNIT_SEL,&tmp);
	/*Set Scale*/
	config->scale.acc_lia_grv = (acc == ACC_LIA_GRVVector_MS2) ? 100.0f : 1.0f;
	config->scale.angular_rate_gyr = (angular == ANGULAR_RATE_GYR_DPS) ? 16.0f : 900.0f;
	config->scale.mag = 16.0f;
	config->scale.euler = (euler == EULER_ANGLES_DEGREES) ? 16.0f : 900.0f;
	config->scale.qua = 16384.0f;
	config->scale.temp = (temp == TEMP_C) ? 1.0f : 2.0f;
	return 1;
}

uint8_t bno_set_temperature_src(mav_bno055_config_t *config,temp_source_t src){
	uint8_t tmp;
	bno_set_page_id(config, PAGE_ID_0);
	if(src == TEMP_SRC_ACCELEROMETER){
		config->temp_src.temp_src = TEMP_SRC_ACCELEROMETER;
		tmp = TEMP_SRC_ACCELEROMETER;
	}
	else if(src == TEMP_SRC_GYROSCOPE){
		tmp = TEMP_SRC_GYROSCOPE;
	}
	bno_write(TEMP_SOURCE,&tmp);
	return 1;
}

uint8_t bno_set_pwr_mode(mav_bno055_config_t *config, power_mode_t pwr_mode){
	bno_set_page_id(config, PAGE_ID_0);
	uint8_t tmp;
	switch(pwr_mode){
	case NORMAL_MODE:
		tmp = NORMAL_MODE;
		bno_write(PWR_MODE,&tmp);
		config->pwrmode.pwr_mode = NORMAL_MODE;
	break;
	case LOW_POWER_MODE:
		tmp = LOW_POWER_MODE;
		bno_write(PWR_MODE,&tmp);
		config->pwrmode.pwr_mode = LOW_POWER_MODE;
	break;
	case SUSPEND_MODE:
		tmp = SUSPEND_MODE;
		bno_write(PWR_MODE,&tmp);
		config->pwrmode.pwr_mode = SUSPEND_MODE;
	break;
	}
	return 1;
}

uint8_t bno_set_operation_mode(mav_bno055_config_t *config, bno055_opr_mode_t mode)
{
    uint8_t cur_mode;
    /*check current opr mode*/
    bno_read(OPR_MODE, &cur_mode);
    if (cur_mode == mode) {
        return 1;
    }
    // If target CONFIGMODE -> Moving to CONFIGMODE
    if (cur_mode != BNO055_OPERATION_CONFIG_MODE) {
        uint8_t cfg_mode = BNO055_OPERATION_CONFIG_MODE;
        bno_write(OPR_MODE, &cfg_mode);
        delay_ms(20);
    }

    // if target Mode is CONFIGMODE
    if (mode == BNO055_OPERATION_CONFIG_MODE) {
        config->oprmode.oprmode = BNO055_OPERATION_CONFIG_MODE;
        return 1;
    }

    // Moving To Target Mode
    uint8_t target_mode = mode;
    bno_write(OPR_MODE, &target_mode);
    delay_ms(20);

    // Retest
    bno_read(OPR_MODE, &cur_mode);
    if (cur_mode == mode) {
        config->oprmode.oprmode = mode;
        return 1;
    }
    return 0;
}

void bno_set_page_id(mav_bno055_config_t *config, bno055_page_id_t id){
	/*checking current id*/
	uint8_t pageid;
	bno_read(PAGE_ID,&pageid);
	if(pageid != id){
		uint8_t tmp;
		tmp = id;
		bno_write(PAGE_ID,&tmp);
		config->pageid = id;
	}
}

uint8_t bno_read_chip_id(bno_global_var_t *bno){
	bno_set_page_id(&bno->config, PAGE_ID_0);
	/*Read Chip ID*/
	bno_read(CHIP_ID_PAGE_0,&bno->id.CHIP_ID);
	if(bno->id.CHIP_ID == BNO055_ID){
		 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
		/*Read ACC_ID*/
	bno_read( ACC_ID,&bno->id.ACC_ID);
	/*Read GYR_ID*/
	bno_read( GYR_ID,&bno->id.GYR_ID);
	/*Read MAG_ID*/
	bno_read( MAG_ID,&bno->id.MAG_ID);
	/*Read SW_REV_ID*/
	uint8_t sw_rev_id_buff[2];
	bno_read_bytes(SW_REV_ID_LSB, sw_rev_id_buff, 2);
	bno->id.SW_REV_ID = (int16_t)((int16_t)sw_rev_id_buff[1]  << 8) | sw_rev_id_buff[0];
	return 1;
}

uint8_t bno_reset(mav_bno055_config_t *config){
	bno_set_page_id(config, PAGE_ID_0);
	  /*Reset System */
	 uint8_t tmp =RESET_SYSTEM;
	 bno_write(SYS_TRIGGER,&tmp);
	 /*Wait sys restart*/
	 delay_ms(600);
	//set BNO055 SYS_TRIGGER TO 0x00
	tmp = DEFAULT_SYSTEM;
	bno_write(SYS_TRIGGER,&tmp);
	return 0;
}

#if  MAV_ENABLE == 0
void bno_mav_do(bno_global_var_t *bno){
	switch(bno->config.sys.systype){
	case BN0055_SYS_RECONFIG:
	{

	}
		break;
	case BN0055_SYS_CALIBRATION:
	{

	}
		break;
	case BN0055_SYS_INIT:
	{

	}
		break;
	case BN0055_SYS_SUSPEND:
	{

	}
		break;
	case BN0055_SYS_LOW_POWER:
	{

	}
		break;
	case BN0055_SYS_DEFAULT:
	{

	}
		break;
	case BN0055_SYS_READ_DATA:
	{

	}
		break;
	}
}

void bno_mav_configuration_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config){
	bno_sys_decode_type(mav_bno055_mode_config,config);
	bno_opr_mode_decode(mav_bno055_mode_config,config);
	bno_fusion_data_output_decode(mav_bno055_mode_config,config);
	bno_axis_remap_config_decode(mav_bno055_mode_config,config);
	bno_axis_remap_sign_decode(mav_bno055_mode_config,config);
	bno_pwr_mode_decode(mav_bno055_mode_config,config);
	bno_temp_src_decode(mav_bno055_mode_config,config);
	bno_acc_gyro_mag_valuation_decode(mav_bno055_mode_config,config);
}

void bno_sys_decode_type(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config){
	switch(mav_bno055_mode_config->sys_config){
	case SYS_RECONFIQ:
	{
		config->sys.systype = BN0055_SYS_RECONFIG;

	}
		break;
	case SYS_CALIBRATION:
	{
		config->sys.systype = BN0055_SYS_CALIBRATION;
	}
		break;
	case SYS_INIT:
	{
		config->sys.systype = BN0055_SYS_INIT;
	}
		break;
	case SYS_SUSPEND:
	{
		config->sys.systype = BN0055_SYS_SUSPEND;
	}
		break;
	case SYS_LOW_POWER:
	{
		config->sys.systype = BN0055_SYS_LOW_POWER;
	}
		break;
	case SYS_DEFAULT:
	{
		config->sys.systype = BN0055_SYS_DEFAULT;
	}
		break;
	case SYS_READ_DATA:
	{
		config->sys.systype = BN0055_SYS_READ_DATA;
	}
		break;
	}
	config->sys.sys_v = (uint8_t)config->sys.systype;
}

void bno_opr_mode_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config){
	switch(mav_bno055_mode_config->opr_mode){
	case MAV_OPERATION_CONFIG_MODE:
	{
		config->oprmode.oprmode = BNO055_OPERATION_CONFIG_MODE;
	}
		break;
	case MAV_OPERATION_MODE_ACCONLY:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_ACCONLY;
	}
		break;
	case MAV_OPERATION_MODE_MAGONLY:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_MAGONLY;
	}
		break;
	case MAV_OPERATION_MODE_GYROONLY:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_GYROONLY;
	}
		break;
	case MAV_OPERATION_MODE_ACCMAG:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_ACCMAG;
	}
		break;
	case MAV_OPERATION_MODE_ACCGYRO:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_ACCGYRO;
	}
		break;
	case MAV_OPERATION_MODE_MAGGYRO:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_MAGGYRO;
	}
		break;
	case MAV_OPERATION_MODE_AMG:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_AMG;
	}
		break;
	case MAV_OPERATION_MODE_IMU:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_IMU;
	}
		break;
	case MAV_OPERATION_MODE_COMPASS:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_COMPASS;
	}
		break;
	case MAV_OPERATION_MODE_M4G:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_M4G;
	}
		break;
	case MAV_OPERATION_MODE_NDOF_FMC_OFF:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_NDOF_FMC_OFF;
	}
		break;
	case MAV_OPERATION_MODE_NDOF:
	{
		config->oprmode.oprmode = BNO055_OPERATION_MODE_NDOF;
	}
		break;
	}
	config->oprmode.oprmode_v = (uint8_t)config->oprmode.oprmode;
}

void bno_fusion_data_output_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config){
	switch(mav_bno055_mode_config->fusion_data){
	case MAV_WINDOWS_FUSION_DATA_OUTPUT:
	{
		config->fusionod.fusiondata = WINDOWS_FUSION_DATA_OUTPUT;
	}
		break;
	case MAV_ANDROID_FUSION_DATA_OUTPUT:
	{
		config->fusionod.fusiondata = ANDROID_FUSION_DATA_OUTPUT;
	}
		break;
	}
	config->fusionod.fusion_data_v = (uint8_t)config->fusionod.fusiondata;
}

void bno_axis_remap_config_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config){
	switch(mav_bno055_mode_config->axis_remap_config){
	case MAV_REMAP_CONFIG_P0_3_5_6:
	{
		config->axisremapconfig.axis_remap_config = REMAP_CONFIG_P0_3_5_6;
	}
		break;
	case	MAV_REMAP_CONFIG_P1_2_4_7:
	{
		config->axisremapconfig.axis_remap_config = REMAP_CONFIG_P1_2_4_7;
	}
		break;
	}
	config->axisremapconfig.remap_config_data_v = (uint8_t)config->axisremapconfig.axis_remap_config;
}

void bno_axis_remap_sign_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config){
	switch(mav_bno055_mode_config->axis_remap_sign){
	case MAV_REMAP_SIGN_P0:
	{
		config->axisremapsign.axis_remap_sign = REMAP_SIGN_P0;
	}
		break;
	case	MAV_REMAP_SIGN_P1:
	{
		config->axisremapsign.axis_remap_sign = REMAP_SIGN_P1;
	}
		break;
	case MAV_REMAP_SIGN_P2:
	{
		config->axisremapsign.axis_remap_sign = REMAP_SIGN_P2;
	}
		break;
	case	MAV_REMAP_SIGN_P3:
	{
		config->axisremapsign.axis_remap_sign = REMAP_SIGN_P3;
	}
		break;
	case	MAV_REMAP_SIGN_P4:
	{
		config->axisremapsign.axis_remap_sign = REMAP_SIGN_P4;
	}
		break;
	case MAV_REMAP_SIGN_P5:
	{
		config->axisremapsign.axis_remap_sign = REMAP_SIGN_P5;
	}
		break;
	case	MAV_REMAP_SIGN_P6:
	{
		config->axisremapsign.axis_remap_sign = REMAP_SIGN_P6;
	}
		break;
	case	MAV_REMAP_SIGN_P7:
	{
		config->axisremapsign.axis_remap_sign = REMAP_SIGN_P7;
	}
		break;
	}
	config->axisremapsign.remap_sign_data_v = (uint8_t)config->axisremapsign.axis_remap_sign;
}

void bno_pwr_mode_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config){
	switch(mav_bno055_mode_config->pwr_mode){
	case MAV_NORMAL_MODE:
	{
		config->pwrmode.pwr_mode = NORMAL_MODE;
	}
		break;
	case MAV_LOW_POWER_MODE:
	{
		config->pwrmode.pwr_mode = LOW_POWER_MODE;
	}
		break;
	case MAV_SUSPEND_MODE:
	{
		config->pwrmode.pwr_mode = SUSPEND_MODE;
	}
		break;
	}
	config->pwrmode.pwr_mode_v = (uint8_t)config->pwrmode.pwr_mode;
}


void bno_temp_src_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config){
	switch(mav_bno055_mode_config->temp_src){
	case MAV_TEMP_SRC_ACCELEROMETER:
	{
		config->temp_src.temp_src = TEMP_SRC_ACCELEROMETER;
	}
		break;
	case MAV_TEMP_SRC_GYROSCOPE:
	{
		config->temp_src.temp_src = TEMP_SRC_GYROSCOPE;
	}
		break;
	}
	config->temp_src.temp_src_v = (uint8_t)config->temp_src.temp_src;
}

void bno_acc_gyro_mag_valuation_decode(mavlink_mav_bno055_mode_config_t* mav_bno055_mode_config,mav_bno055_config_t *config){
	config->acc_gyr_mag_valuation.acc_g_range = mav_bno055_mode_config->acc_g_range;
	config->acc_gyr_mag_valuation.acc_bandwidth = mav_bno055_mode_config->acc_bandwidth;
	config->acc_gyr_mag_valuation.acc_mode = mav_bno055_mode_config->acc_mode;

	config->acc_gyr_mag_valuation.gyr_range = mav_bno055_mode_config->gyr_range;
	config->acc_gyr_mag_valuation.gyr_bandwidth = mav_bno055_mode_config->gyr_bandwidth;
	config->acc_gyr_mag_valuation.gyr_mode = mav_bno055_mode_config->gyr_mode;

	config->acc_gyr_mag_valuation.mag_data_rate = mav_bno055_mode_config->mag_data_rate;
	config->acc_gyr_mag_valuation.mag_operation_mode = mav_bno055_mode_config->mag_operation_mode;
	config->acc_gyr_mag_valuation.mag_pwr_mode = mav_bno055_mode_config->mag_pwr_mode;

	config->acc_gyr_mag_valuation.acc_linearacc_gravityvector_unit = mav_bno055_mode_config->acc_linearacc_gravityvector_unit;
	config->acc_gyr_mag_valuation.angular_rate_gyr_unit = mav_bno055_mode_config->angular_rate_unit;
	config->acc_gyr_mag_valuation.euler_angles_unit = mav_bno055_mode_config->euler_angles_unit;
	config->acc_gyr_mag_valuation.temp_unit = mav_bno055_mode_config->temp_unit;
}
#endif

/*Read Register Data*/
int8_t bno_calib_stat(bno_global_var_t *bno){
	uint8_t buffer_calib_sta[1] ={};
	if(bno_read(CALIB_STAT,buffer_calib_sta)<0){
		return -1;
	}
	bno->calib.sts.sys = (int16_t)((int8_t)buffer_calib_sta[0])>>6 & 0x03;
	bno->calib.sts.gyr = (int16_t)((int8_t)buffer_calib_sta[0])>>4 & 0x03;
	bno->calib.sts.acc = (int16_t)((int8_t)buffer_calib_sta[0])>>2 & 0x03;
	bno->calib.sts.mag = (int16_t)((int8_t)buffer_calib_sta[0]) & 0x03;
	if(bno->calib.sts.sys == 3 && bno->calib.sts.gyr == 3 && bno->calib.sts.acc == 3 && bno->calib.sts.mag){
		bno->calib.sts.indicatesFullyCalibrated = true;
	}
	else{
		bno->calib.sts.indicatesFullyCalibrated = false;
	}
	return 0;
}

int8_t bno_get_calibration_offset(bno_global_var_t *bno){
	if(bno->config.oprmode.oprmode == BNO055_OPERATION_CONFIG_MODE && bno->calib.sts.indicatesFullyCalibrated == true){
		uint8_t buffer_calibrations [22] = {};
		if(bno_read_bytes(ACC_OFFSET_X_lSB,buffer_calibrations,18)<0){
			return -1;
		}
		bno->calib.accxoffset = (int16_t)((int16_t)buffer_calibrations[1]) << 8 | buffer_calibrations[0];
		bno->calib.accyoffset = (int16_t)((int16_t)buffer_calibrations[3]) << 8 | buffer_calibrations[2];
		bno->calib.acczoffset = (int16_t)((int16_t)buffer_calibrations[5]) << 8 | buffer_calibrations[4];
		bno->calib.magxoffset = (int16_t)((int16_t)buffer_calibrations[7]) << 8 | buffer_calibrations[6];
		bno->calib.magyoffset = (int16_t)((int16_t)buffer_calibrations[9]) << 8 | buffer_calibrations[8];
		bno->calib.magzoffset = (int16_t)((int16_t)buffer_calibrations[11]) << 8 | buffer_calibrations[10];
		bno->calib.gyrxoffset = (int16_t)((int16_t)buffer_calibrations[13]) << 8 | buffer_calibrations[12];
		bno->calib.gyryoffset = (int16_t)((int16_t)buffer_calibrations[15]) << 8 | buffer_calibrations[14];
		bno->calib.gyrzoffset = (int16_t)((int16_t)buffer_calibrations[17]) << 8 | buffer_calibrations[16];
		bno->calib.accradius = (int16_t)((int16_t)buffer_calibrations[19]) << 8 | buffer_calibrations[18];
		bno->calib.magradius = (int16_t)((int16_t)buffer_calibrations[21]) << 8 | buffer_calibrations[20];
	}
	else{
		return -1;
	}
	return 0;
}

int8_t bno_get_accel_gyro(bno_global_var_t *bno){
	uint8_t buffer_accel[6] = {};
	uint8_t buffer_gyro[6] = {};
	if(bno_read_bytes(ACC_DATA_X_LSB,buffer_accel,6)<0){
		return -1;
	}
	bno->acc.x = (int16_t)((int16_t)buffer_accel[1]  << 8) | buffer_accel[0]; bno->acc.x /= bno->config.scale.acc_lia_grv;
	bno->acc.y = (int16_t)((int16_t)buffer_accel[3]  << 8) | buffer_accel[2]; bno->acc.y /= bno->config.scale.acc_lia_grv;
	bno->acc.z = (int16_t)((int16_t)buffer_accel[5]  << 8) | buffer_accel[4]; bno->acc.z /= bno->config.scale.acc_lia_grv;
	if(bno_read_bytes(GYR_DATA_X_LSB,buffer_gyro,6)<0){
			return -1;
		}
	bno->gyr.x = (int16_t)((int16_t)buffer_gyro[1]  << 8) | buffer_gyro[0]; bno->gyr.x /= bno->config.scale.angular_rate_gyr;
	bno->gyr.y = (int16_t)((int16_t)buffer_gyro[3]  << 8) | buffer_gyro[2]; bno->gyr.y /= bno->config.scale.angular_rate_gyr;
	bno->gyr.z = (int16_t)((int16_t)buffer_gyro[5]  << 8) | buffer_gyro[4]; bno->gyr.z /= bno->config.scale.angular_rate_gyr;
	return 0;
}

int8_t bno_get_mag(bno_global_var_t *bno){
	uint8_t buffer_mag[6] = {};
	if(bno_read_bytes(MAG_DATA_X_LSB,buffer_mag,6)<0){
			return -1;
		}
	bno->mag.x = (int16_t)((int16_t)buffer_mag[1]  << 8) | buffer_mag[0]; bno->mag.x /= bno->config.scale.mag;
	bno->mag.y = (int16_t)((int16_t)buffer_mag[3]  << 8) | buffer_mag[2]; bno->mag.y /= bno->config.scale.mag;
	bno->mag.z = (int16_t)((int16_t)buffer_mag[5]  << 8) | buffer_mag[4]; bno->mag.z /= bno->config.scale.mag;
	return 0;
}

int8_t bno_get_temp(bno_global_var_t *bno){
	uint8_t buffer_temp[1] = {};
	if(bno_read_bytes(TEMP,buffer_temp,1)<0){
				return -1;
	}
	bno->temperature = buffer_temp[0] * bno->config.scale.temp;
	return 0;
}

int8_t bno_get_elu_data(bno_global_var_t *bno){
	uint8_t buffer_eul[6] = {};
	if(bno_read_bytes(EUL_HEADING_LSB,buffer_eul,6)<0){
					return -1;
	}
	bno->eul.heading = (int16_t)((int16_t)buffer_eul[1]  << 8) | buffer_eul[0]; bno->eul.heading /= bno->config.scale.euler;
	bno->eul.roll = (int16_t)((int16_t)buffer_eul[3]  << 8) | buffer_eul[2]; bno->eul.roll /= bno->config.scale.euler;
	bno->eul.pitch = (int16_t)((int16_t)buffer_eul[5]  << 8) | buffer_eul[4]; bno->eul.pitch /= bno->config.scale.euler;
	return 0;
}

int8_t bno_get_qua_data(bno_global_var_t *bno){
	uint8_t buffer_qua[8] = {};
	if(bno_read_bytes(QUA_DATA_W_LSB,buffer_qua,8)<0){
					return -1;
			}
	bno->qua.w = (int16_t)((int16_t)buffer_qua[1]  << 8) | buffer_qua[0];bno->qua.w /= bno->config.scale.qua;
	bno->qua.x = (int16_t)((int16_t)buffer_qua[3]  << 8) | buffer_qua[2]; bno->qua.x /= bno->config.scale.qua;
	bno->qua.y = (int16_t)((int16_t)buffer_qua[5]  << 8) | buffer_qua[4]; bno->qua.y /= bno->config.scale.qua;
	bno->qua.z = (int16_t)((int16_t)buffer_qua[7]  << 8) | buffer_qua[6]; bno->qua.z /= bno->config.scale.qua;
	return 0;
}

int8_t bno_get_lia_data(bno_global_var_t *bno){
	uint8_t buffer_lia[6] = {};
	if(bno_read_bytes(LIA_DATA_X_LSB,buffer_lia,6)<0){
				return -1;
			}
	bno->lia.x = (int16_t)((int16_t)buffer_lia[1]  << 8) | buffer_lia[0]; bno->lia.x /= bno->config.scale.acc_lia_grv;
	bno->lia.y = (int16_t)((int16_t)buffer_lia[3]  << 8) | buffer_lia[2]; bno->lia.y /= bno->config.scale.acc_lia_grv;
	bno->lia.z = (int16_t)((int16_t)buffer_lia[5]  << 8) | buffer_lia[4]; bno->lia.z /= bno->config.scale.acc_lia_grv;
	return 0;
}

int8_t bno_get_grv_data(bno_global_var_t *bno){
	uint8_t buffer_grv[6] = {};
	if(bno_read_bytes(GRV_DATA_X_LSB,buffer_grv,6)<0){
				return -1;
			}
	bno->grv.x = (int16_t)((int16_t)buffer_grv[1]  << 8) | buffer_grv[0]; bno->grv.x/= bno->config.scale.acc_lia_grv;
	bno->grv.y = (int16_t)((int16_t)buffer_grv[3]  << 8) | buffer_grv[2]; bno->grv.y/= bno->config.scale.acc_lia_grv;
	bno->grv.z = (int16_t)((int16_t)buffer_grv[5]  << 8) | buffer_grv[4]; bno->grv.z/= bno->config.scale.acc_lia_grv;
	return 0;
}

/*BNO055 read Sys*/
int8_t bno_read_sys_error(bno_global_var_t *bno){
	uint8_t buffer_sys_error;
	if(bno_read(SYS_ERR,&buffer_sys_error)<0){
				return -1;
	}
	bno->sts.sys_error = buffer_sys_error;
	return 1;
}

int8_t bno_read_sys_status(bno_global_var_t *bno){
	uint8_t buffer_sys_status;
	if(bno_read(SYS_STATUS,&buffer_sys_status)<0){
				return -1;
	}
	bno->sts.sys_status = buffer_sys_status;
	return 1;
}

int8_t bno_read_clk_status(bno_global_var_t *bno){
	uint8_t buffer_clk_status;
	if(bno_read(SYS_CLK_STATUS,&buffer_clk_status)<0){
				return -1;
	}
	bno->sts.sys_clk_status = buffer_clk_status;
	return 1;
}

int8_t bno_read_int_sta(bno_global_var_t *bno){
	uint8_t buffer_int_sta;
	if(bno_read(INT_STA,&buffer_int_sta)<0){
				return -1;
	}
	bno->sts.int_sta_acc_nm = (int16_t)((int8_t)buffer_int_sta)>>7 & 0x01;
	bno->sts.int_sta_acc_am = (int16_t)((int8_t)buffer_int_sta)>>6 & 0x01;
	bno->sts.int_sta_acc_high_g = (int16_t)((int8_t)buffer_int_sta)>>5 & 0x01;
	bno->sts.int_sta_gyro_high_rate = (int16_t)((int8_t)buffer_int_sta)>>3 & 0x01;
	bno->sts.int_sta_gyro_am = (int16_t)((int8_t)buffer_int_sta)>>2 & 0x01;
	return 1;
}

int8_t bno_read_st_result(bno_global_var_t *bno){
	uint8_t buffer_st_result;
	if(bno_read(ST_RESULT,&buffer_st_result)<0){
				return -1;
	}
	bno->sts.ST_RESULT_ST_ACC = (int16_t)((int8_t)buffer_st_result) & 0x01;
	bno->sts.ST_RESULT_ST_MAG = (int16_t)((int8_t)buffer_st_result)>>1 & 0x01;
	bno->sts.ST_RESULT_ST_GYR = (int16_t)((int8_t)buffer_st_result)>>2 & 0x01;
	bno->sts.ST_RESULT_ST_MCU = (int16_t)((int8_t)buffer_st_result)>>3 & 0x01;
	return 1;
}

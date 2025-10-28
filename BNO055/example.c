#include "bno055_mav_config.h"

static bno_global_var_t bno;

int imu_get_sys_gyr_acc_mag_status(uint8_t status[4]);

float imu_get_accel(float acc[3]);
float imu_get_gyro(float gyr[3]);
int imu_get_mag(float mag[3]);

float imu_get_roll_euler(void);
float imu_get_pitch_euler(void);
float imu_get_yaw_euler(void);

int imu_get_quaternions(float qua[4]);
int imu_get_lia(float lia[3]);
int imu_get_grv(float grv[3]);

int imu_get_acc_calib_bias(float acc_bias[3]);
int imu_get_gyro_calib_bias(float gyro_bias[3]);
int imu_get_mag_calib_bias(float mag_bias[3]);
int imu_get_mag_acc_radius(float mag_acc_radius[2]);


int main(void){
	bno_default_initization(&bno);
	while(1){
		bno_get_accel_gyro(&bno);
		bno_get_mag(&bno);
		bno_get_elu_data(&bno);
		bno_get_qua_data(&bno);
		bno_get_grv_data(&bno);
		bno_get_lia_data(&bno)
	HAL_Delay(200);
	}
	return 0;
}

float imu_get_accel(float acc[3]){
	acc[0] = bno.acc.x;
	acc[1] = bno.acc.y;
	acc[2] = bno.acc.z;
	return 0;
}

float imu_get_gyro(float gyr[3]){
	gyr[0] = bno.gyr.x;
	gyr[1] = bno.gyr.y;
	gyr[2] = bno.gyr.z;
	return 0;
}

float imu_get_roll_euler(void){
	return bno.eul.roll;
}

float imu_get_pitch_euler(void){
	return bno.eul.pitch;
}

float imu_get_yaw_euler(void){
	return bno.eul.heading;
}

int imu_get_mag(float mag[3]){
	mag[0] = bno.mag.x;
	mag[1] = bno.mag.y;
	mag[2] = bno.mag.z;
	return 0;
}

int imu_get_quaternions(float qua[4]){
	qua[0] = bno.qua.w;
	qua[1] = bno.qua.x;
	qua[2] = bno.qua.y;
	qua[3] = bno.qua.z;
	return 0;
}

int imu_get_lia(float lia[3]){
	lia[0] = bno.lia.x;
	lia[1] = bno.lia.y;
	lia[2] = bno.lia.z;
	return 0;
}

int imu_get_grv(float grv[3]){
	grv[0] = bno.grv.x;
	grv[1] = bno.grv.y;
	grv[2] = bno.grv.z;
	return 0;
}

int imu_get_acc_calib_bias(float acc_bias[3]){
	acc_bias[0] = bno.calib.accxoffset;
	acc_bias[1] = bno.calib.accyoffset;
	acc_bias[2] = bno.calib.acczoffset;
	return 0;
}

int imu_get_gyro_calib_bias(float gyro_bias[3]){
	gyro_bias[0] = bno.calib.gyrxoffset;
	gyro_bias[1] = bno.calib.gyryoffset;
	gyro_bias[2] = bno.calib.gyrzoffset;
	return 0;
}

int imu_get_mag_acc_radius(float mag_acc_radius[2]){
	mag_acc_radius[0] = bno.calib.magradius;
	mag_acc_radius[1] = bno.calib.accradius;
	return 0;
}

int imu_get_mag_calib_bias(float mag_bias[3]){
	mag_bias[0] = bno.calib.magxoffset;
	mag_bias[1] = bno.calib.magyoffset;
	mag_bias[2] = bno.calib.magzoffset;
	return 0;
}

int imu_get_sys_gyr_acc_mag_status(uint8_t status[4]){
	 status[0] = bno.calib.sts.sys;
	 status[1] = bno.calib.sts.gyr;
	 status[2] = bno.calib.sts.acc;
	 status[3] = bno.calib.sts.mag;
	 return 0;
 }

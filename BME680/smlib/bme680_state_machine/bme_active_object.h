/*
 * bme_active_object.h
 *
 *  Created on: Nov 18, 2024
 *      Author: nguye
 */

#ifndef BME680_BME680_STATE_MACHINE_BME_ACTIVE_OBJECT_H_
#define BME680_BME680_STATE_MACHINE_BME_ACTIVE_OBJECT_H_

#include "stdint.h"
#include "stdbool.h"
#include "bme68x.h"
#include "bme680_common.h"
#include "user_define.h"
#include "bme_active_object.h"
#include "math.h"
#include "bme68x_defs.h"

typedef struct{
	uint16_t par_temperature_1;
	int16_t par_temperature_2;
	int8_t par_temperature_3;
	uint16_t par_humidity_1_2[2];
	int8_t par_humidity_3_4_5_7[4];
	uint8_t par_humidity_6;
	int8_t par_gas_sensor[3];
	uint16_t par_pressure_1;
	int16_t par_pressure_2_4_5_8_9[5];
	int8_t par_pressure_3_6_7[3];
	uint8_t par_pressure_10;
}bme_calib_value_t;

typedef enum{
	BME_EVENT_HANDLED,
	BME_EVENT_IGNORED,
	BME_EVENT_TRANSITION,
}bme_event_status_t;

typedef enum{
	READ_BME_SENSOR_SIG = 1,
	BME_HEATING_SIG,
	BME_TICK_SIG,
	BME_ENTRY,
	BME_EXIT,
}bme_proobject_signal_t;

typedef enum{
	BME_HEATING_SENSORS_SM,
	BME_READING_SENSORS_SM,
}bme_proobject_state_t;

struct bme_proobject_tag;
struct bme_event_tag;

typedef struct bme_proobject_tag {
	bme_proobject_state_t active_state;
	/*BME STRUCTER*/

	uint16_t ticksigcnt;

	uint32_t del_period;

	uint8_t n_fields;

	bme680_mode_init_t bme_mode;

	bool IsGetCalibrationDataYet;

	struct bme68x_dev bme;
	struct bme68x_conf conf;
	struct bme68x_heatr_conf heatr_conf;
	struct bme68x_calib_data data_calib;
	struct bme68x_data data_parallel[3];
	struct bme68x_data data_sequential[3];
	struct bme68x_data data_forced;

}bme_proobject_t;

/*Generic(Super) event structure */
typedef struct bme_event_tag{
    uint8_t sig;
}bme_event_t;

/* For user generated events */
typedef struct{
	bme_event_t super;
    uint8_t ss;
}bme_proobject_user_event_t;

/* For tick event */
typedef struct{
	bme_event_t super;
    uint8_t ss;
}bme_proobject_tick_event_t;

void bme_proobject_init(bme_proobject_t *const mobj);
bme_event_status_t bme_proobject_state_machine(bme_proobject_t *const mobj, bme_event_t const * const e);


#endif /* BME680_BME680_STATE_MACHINE_BME_ACTIVE_OBJECT_H_ */

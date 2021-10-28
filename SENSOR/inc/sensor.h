#ifndef SENSOR_H__
#define SENSOR_H__
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "hmc.h"
#include "config_parameter.h"

void Init_sensor(void);
void Measure_sensor(sensor_data *sensordata);

#endif /* SENSOR_H__ */

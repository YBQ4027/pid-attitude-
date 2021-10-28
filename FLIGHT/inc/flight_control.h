#ifndef _FLIGHT_CONTROL_H_
#define _FLIGHT_CONTROL_H_
#include "config_parameter.h"

extern sensor_data Sensordata;
extern Control_signal control;
extern ParamDesired paramdisired;
extern float actualThrust;
void flight_control_task(void);


#endif	/* _FLIGHT_CONTROL_H_ */

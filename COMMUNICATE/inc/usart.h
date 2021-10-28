#ifndef _USART1_H_
#define _USART1_H_
#include "usart.h"
#include "config_parameter.h"

void usart1_send_char(u8 c);
void usart1_niming_report(u8 fun,u8*data,u8 len);
void usart1_report_Angle(u8 fun);
void usart1_report_senserdata(sensor_data 	*sensordata,u8 fun);
void usart1_report_Control(Control_signal *control,u8 fun);
void usart1_report_PID(PID_structure *pidAngleRoll,PID_structure *pidAnglePitch,PID_structure * pidAngleYaw,u8 fun);
void usart1_report_motorPWM(motorPWM *motor_PWM,u8 fun);
void usart1_report_loop_num(u8 fun);
#endif

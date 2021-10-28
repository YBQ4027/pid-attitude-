#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "config_parameter.h"
//#include "stabilizer_types.h"

void stateControlInit(void);
void StateControl(Control_signal *control,State *state,ParamDesired *paramdisired,float actualThrust);


#endif /*__STATE_CONTROL_H */


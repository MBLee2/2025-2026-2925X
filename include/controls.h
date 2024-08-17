
#ifndef _CONTROLS_H_
#define _CONTROLS_H_

#include "api.h"

// task fuctions for drivebase contol to be parrlelized
extern void taskFn_drivebase_control(void);
// task fuctions for flywheel contol to be parrlelized
extern void taskFn_basket_control(void);
// task fuctions for intake contol to be parrlelized
extern void taskFn_intake_control(void);
// task fuctions for intake contol to be parrlelized
extern void taskFn_mogo_control(void);
// Auto cata controller
extern void taskFn_auto_intake_push_control(void);
// task fuctions for intake contol to be parrlelized
extern void taskFn_hood_control(void);
//task functions for hood control to be parrlelized





#endif //_CONTROLS_H_
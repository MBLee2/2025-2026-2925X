
#ifndef _CONTROLS_H_
#define _CONTROLS_H_

#include "api.h"

// task fuctions for drivebase contol to be parrlelized
extern void taskFn_drivebase_control(void);
// task fuctions for flywheel contol to be parrlelized
extern void taskFn_flywheel_control(void);
// task fuctions for intake contol to be parrlelized
extern void taskFn_intake_control(void);
// task fuctions for intake contol to be parrlelized
extern void taskFn_wings_control(void);
// Auto cata controller
extern void taskFn_auto_intake_control(void);
// task fuctions for intake contol to be parrlelized
extern double psi;





#endif //_CONTROLS_H_
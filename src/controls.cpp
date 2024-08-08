#include "controls.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#define TURN_CONST 1.4
//Drivebase control
void taskFn_drivebase_control(void)
{
    
    printf("%s(): Entered \n", __func__);
    while (true) 
    {
        int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        int turnVelright = TURN_CONST*rightX;
        int turnVelleft = TURN_CONST*leftX;

        left_side_motors.move(leftY + turnVelleft);
		right_side_motors.move(leftY - turnVelleft);

        pros::delay(20);

    } // end of while loop

    printf("%s(): Exiting \n", __func__);
    
} // end of taskFn_drivebase_control

//Flwheel control
void taskFn_flywheel_control(void)
{
    printf("%s(): Entered \n", __func__);
    
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_flywheel_control

//Intake control
void taskFn_intake_control(void)
{
    printf("%s(): Entered \n", __func__);
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_intake_control

//Wings Control
void taskFn_wings_control(void)
{
    printf("%s(): Entered \n", __func__);
    printf("%s(): Exiting \n", __func__);
}

void taskFn_auto_intake_control(void)
{
    printf("%s(): Entered \n", __func__);
    printf("%s(): Exiting \n", __func__);
}
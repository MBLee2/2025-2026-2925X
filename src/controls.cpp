#include "controls.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"

#define TURN_CONST 1.4
//Drivebase control
void taskFn_drivebase_control(void){
    printf("%s(): Entered \n", __func__);
    bool drive_state = true; // true for normal, false for reversed
    while (true) 
    {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            drive_state = !drive_state;  // Toggle direction
            pros::delay(300);  // Add a small delay to avoid rapid toggling
        }

        int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        // If the drive direction is reversed, negate the joystick inputs
        if (!drive_state) {
            leftY = -leftY;
            rightY = -rightY;
        }

        int turnVelright = TURN_CONST*rightX;
        int turnVelleft = TURN_CONST*leftX;

        left_side_motors.move(leftY + turnVelleft);
		right_side_motors.move(leftY - turnVelleft);

        //chassis.arcade(leftY, leftX);

        pros::delay(20);
    } // end of while loop

    printf("%s(): Exiting \n", __func__);
    
} // end of taskFn_drivebase_control

//Flwheel control
void taskFn_lift_control(void){
    printf("%s(): Entered \n", __func__);
    bool basket_state = false;
    while (true) 
    {
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            lift.move(127);  
        }   
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            lift.move(-127);  
            while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            {
                lift.move(127);  
            }     
        } 
        lift.move(0);  
        pros::delay(20);
    }
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_lift_control

//Intake control
void taskFn_intake_control(void){
    printf("%s(): Entered \n", __func__);

    enum intake_state {
    INTAKE,
    OUTAKE,
    STOP
    };
    intake_state current_state = STOP;  // Initialize with a default state, STOP
    while (true) 
    {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) 
        {
            if (current_state == OUTAKE || current_state == STOP)
            {
                intake.move(127);
                current_state = INTAKE;

            }
            else if(current_state == INTAKE)
            {
                intake.move(0);
                current_state = STOP;
            }   
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) 
        {
            if (current_state == INTAKE || current_state == STOP)
            {
                intake.move(-127);
                current_state = OUTAKE;

            }
            else if(current_state == OUTAKE)
            {
                intake.move(0);
                current_state = STOP;
            }   
        }
        pros::delay(20);
    }
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_intake_control

//Wings Control
void taskFn_mogo_control(void){
    printf("%s(): Entered \n", __func__);
    bool mogo_state = false;
    while (true) 
    {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) 
        {
            if (mogo_state == false)
            {
                mogo_state = true;
                mogo_clamp.set_value(true);

            }
            else if(mogo_state == true)
            {
                mogo_state = false;  
                mogo_clamp.set_value(false);  
            }   
        }
       pros::delay(20); 
    }
    
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_mogo_control

void taskFn_auto_intake_push_control(void){
    printf("%s(): Entered \n", __func__);
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_auto_intake_push_control

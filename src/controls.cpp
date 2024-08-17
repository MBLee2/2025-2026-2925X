#include "controls.h"
#include "pros/device.hpp"
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
        
        
        // If the drive direction is reversed, negate the joystick inputs
        if (!drive_state) {
            leftY = -leftY;
        }

        int turnVelleft = TURN_CONST*leftX;

        left_side_motors.move(leftY + turnVelleft);
		right_side_motors.move(leftY - turnVelleft);

        //chassis.arcade(leftY, leftX);

        pros::delay(20);
    } // end of while loop

    printf("%s(): Exiting \n", __func__);
    
} // end of taskFn_drivebase_control

//Lift control
void taskFn_lift_control(void){
    printf("%s(): Entered \n", __func__);
    bool basket_state = false;
    while (true) 
    {
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            lift.move(127);  
        }   
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            lift.move(-127);  
            while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
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
    bool intake_lifted = false;
    bool hood_state = false;

    while (true) 
    {
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        rightX = rightX/127;
        if (rightX > 0.85){intake_lifted = true;}
        if (rightX < -0.85){intake_lifted = false;}

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

        if(intake_lifted == true)
        {
            intake_lift.set_value(true); //extended
        }

        if(intake_lifted == false)
        {
            intake_lift.set_value(false); //retracted
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) 
        {
            if (hood_state == false)
            {
                hood_state = true;
                hood1.set_value(true);
                hood2.set_value(true);

            }
            else if(hood_state == true)
            {
                hood_state = false;  
                hood1.set_value(false);
                hood2.set_value(false); 
            }   
        }
        pros::delay(20);
    }
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_intake_control

//Mogo Control
void taskFn_mogo_control(void){
    printf("%s(): Entered \n", __func__);
    bool mogo_state = false;
    bool sweeper_out = false;
    while (true) 
    {
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        rightY = rightY/127;
        if (rightY > 0.85){sweeper_out = true;}
        if (rightY < -0.85){sweeper_out = false;}

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

        if(sweeper_out == true)
        {
            mogo_rush.set_value(true); //extended
        }

        if(sweeper_out == false)
        {
            mogo_rush.set_value(false); //retracted
        }

       pros::delay(20); 
    }
    
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_mogo_control

void taskFn_auto_intake_push_control(void){
    /*sudo code   
    if color sensor sees wrong color 



    //*/
    printf("%s(): Entered \n", __func__);
    while (true) 
    {
        /*TEMPORARY WE NEED TO SWITCH TO AUTOMATED LATER*/
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            intake_puncher.set_value(true);
            pros::delay(100);
            intake_puncher.set_value(false);
        }
        pros::delay(20);
    }
    
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_auto_intake_push_control

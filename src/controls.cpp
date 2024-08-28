#include "controls.h"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
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

// Intake control
void taskFn_intake_control(void){
    printf("%s(): Entered \n", __func__);  // Log the function entry for debugging

    // Define an enum to represent the intake's possible states
    enum intake_state {
        INTAKE,   // Intake objects
        OUTAKE,   // Eject objects
        STOP      // Stop the intake
    };

    bool basket_state = false;  // Track the state of the basket (false = retracted, true = extended)
    bool intake_lifted = false;  // Track whether the intake is lifted (false = down, true = up)
    intake_state current_state = STOP;  // Initialize with a default state, STOP
    
    lift.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);  // Set the encoder units for the lift motor to degrees
    intake_color.set_led_pwm(100);  // Set the LED the intake color sensor to 100

    while (true)  // Infinite loop to keep checking controller input for intake control
    {
        double pos = lift.get_position();  // Get the current position of the lift
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // Get the horizontal joystick input for the right stick
        rightX = rightX / 127;  // Normalize the joystick input to a range of -1 to 1

        // Determine if the intake should be lifted based on joystick position
        if (rightX > 0.85) {
            intake_lifted = true;  // Lift the intake if joystick is pushed to the right
        }
        if (rightX < -0.85) {
            intake_lifted = false;  // Lower the intake if joystick is pushed to the left
        }
        
        // Toggle intake on or off with the A button
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) 
        {
            if (current_state == OUTAKE || current_state == STOP)  // If intake is stopped or ejecting, start intake
            {
                intake.move(127);  // Start the intake
                current_state = INTAKE;

            }
            else if (current_state == INTAKE)  // If intake is running, stop it
            {
                intake.move(0);  // Stop the intake
                current_state = STOP;
            }   
        }
        
        // Toggle the basket state with the Y button
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) 
        {
            if (basket_state == true)  // If the basket is extended, retract it
            {
                basket_state = false;
                hood1.set_value(false);
                hood2.set_value(false);
            }
            else if (basket_state == false)  // If the basket is retracted, extend it
            {
                basket_state = true;
                hood1.set_value(true);
                hood2.set_value(true);
            }   
        }

        // Retract the hood if the lift position is below -100 degrees
        if (pos < -100)
        {
            hood1.set_value(false);
            hood2.set_value(false);
        }
        else if (basket_state == true) {  // Otherwise, extend the hood if the basket is extended
            hood1.set_value(true);
            hood2.set_value(true);
        }

        master.print(0, 0, "hue: %f", intake_color.get_hue());  // Print the hue value from the intake color sensor to the controller
        int hue = intake_color.get_hue();  // Get the current hue from the intake color sensor

        // If the basket is extended and the detected hue matches specific values, control the intake accordingly
        if (basket_state == true)
        {
            if ((hue >= 7 && hue <= 17) || (hue >= 210 && hue <= 220))
            {
                pros::delay(170);  // Small delay before reversing the intake
                intake.move(-127);  // Reverse the intake for a short duration
                pros::delay(300);
                intake.move(127);  // Resume intake after the reversal
            }
        }

        // Toggle the intake to eject with the B button
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) 
        {
            if (current_state == INTAKE || current_state == STOP)  // If intake is running or stopped, start ejecting
            {
                intake.move(-127);  // Reverse the intake to eject
                current_state = OUTAKE;

            }
            else if (current_state == OUTAKE)  // If intake is ejecting, stop it
            {
                intake.move(0);  // Stop the intake
                current_state = STOP;
            }   
        }

        // Control the intake lift based on the intake_lifted flag
        if (intake_lifted == true)
        {
            intake_lift.set_value(true);  // Lift the intake
        }

        if (intake_lifted == false)
        {
            intake_lift.set_value(false);  // Lower the intake
        }
    }
    printf("%s(): Exiting \n", __func__);  // Log the function exit for debugging
} // end of taskFn_intake_control


//Intake control
void taskFn_hood_control(void){
    /*sudo code   
    if color sensor sees wrong color 
    //*/
    bool hood_state = false;
    printf("%s(): Entered \n", __func__);
    while (true) 
    {
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
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            intake.move(127);
            pros::delay(400);
        }
        
        pros::delay(20);
    }
    
    printf("%s(): Exiting \n", __func__);
} // end of taskFn_auto_intake_push_control

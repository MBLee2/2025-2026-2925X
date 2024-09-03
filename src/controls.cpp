#include "controls.h"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"

#define TURN_CONST 1.4 // Constant multipled by X input to allow for instant turns during driver control
//Drivebase control
void taskFn_drivebase_control(void){
    printf("%s(): Entered \n", __func__);  // Log the function entry for debugging
    bool drive_state = true; // true for normal, false for reversed drive direction
    while (true)  // Infinite loop to keep checking controller input and drive base state
    {
        // Get  horizontal and vertical joystick input for movement and turning
        int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);  
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); 
        
        // Multiply the turning input to prioritize turning over forward movement, enabling agile motion
        int turnVelleft = TURN_CONST * leftX;

        // Control the left and right motors based on the calculated values
        left_side_motors.move(leftY + turnVelleft);
        right_side_motors.move(leftY - turnVelleft);
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            drive_state = !drive_state;  // Toggle drive direction when the X button is pressed
            pros::delay(300);  // Add a small delay to avoid rapid toggling of direction
        }
        // If the drive direction is reversed, negate the joystick input for forward/backward movement
        if (!drive_state) {
            leftY = -leftY;
        }

        pros::delay(20); // loop runs at a steady pace, still avoids CPU overload
    } // end of while loop
    printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_drivebase_control

// Lift control
void taskFn_lift_control(void){
    printf("%s(): Entered \n", __func__);  // Log the function entry for debugging
    while (true)  // Infinite loop to keep checking controller input for lift control
    {
        // While the R2 button is pressed, move the lift up at full speed
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            lift.move(127);  // Move the lift up
        }   
        // While the R1 button is pressed, move the lift down at full speed
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            lift.move(-127);  // Move the lift down
            // If R2 is pressed while R1 is still held, move the lift up instead
            while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            {
                lift.move(127);  // Move the lift up
            }     
        } 
        lift.move(0);// If neither R1 nor R2 is pressed, stop the lift
        pros::delay(20); // loop runs at a steady pace, still avoids CPU overload
    }
    printf("%s(): Exiting \n", __func__);  // Log the function exit for debugging
} // end of taskFn_lift_control

// Mogo Control
void taskFn_mogo_control(void){
    printf("%s(): Entered \n", __func__);  // Log the function entry for debugging
    bool mogo_state = false;  // Track the state of the mogo clamp (false = open, true = closed)
    bool sweeper_out = false;  // Track the state of the sweeper (false = retracted, true = extended)
    while (true)  // Infinite loop to keep checking controller input for mogo control
    {
        // When the L1 button is pressed, toggle the mogo clamp state
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) 
        {
            if (mogo_state == false)  // If the clamp is open, close it
            {
                mogo_state = true;
                mogo_clamp.set_value(true);  // Close the mogo clamp

            }
            else if (mogo_state == true)  // If the clamp is closed, open it
            {
                mogo_state = false;  
                mogo_clamp.set_value(false);  // Open the mogo clamp
            }   
        }


        // Get the input from the right joystick and normalize the input to a range of -1 to 1
        int rightY = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y))/127;  
        // If the joystick is pushed upward past 85%, set sweeper_out to true (extended)
        if (rightY > 0.85){sweeper_out = true; }
        // If the joystick is pushed downward past 85%, set sweeper_out to false (retracted)
        if (rightY < -0.85){sweeper_out = false; }
        
        // If the sweeper is extended, extend the mogo rush
        if (sweeper_out == true)
        {
            mogo_rush.set_value(true);  // Extend the mogo rush
        }

        // If the sweeper is retracted, retract the mogo rush
        if (sweeper_out == false)
        {
            mogo_rush.set_value(false);  // Retract the mogo rush
        }

        pros::delay(20); // loop runs at a steady pace, still avoids CPU overload
    }
    printf("%s(): Exiting \n", __func__);  // Log the function exit for debugging
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
    intake_color.set_led_pwm(100);  // Set the LED PWM for the intake color sensor to 100

    while (true)  // Infinite loop to keep checking controller input for intake control
    {
        double pos = lift.get_position();  // Get the current position of the lift
        int rightX = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127;  // Normalize the right joystick input to -1 to 1
        int hue = intake_color.get_hue();  // Get the current hue value from the intake color sensor
        
        // Toggle intake on or off with the A button
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (current_state == OUTAKE || current_state == STOP)  // If the intake is stopped or ejecting, start intake
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
        // Eject objects with the B button
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
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
        
        // Control intake based on color sensor readings when basket is extended
        if (basket_state == true)
        {
            if (hue >= 7 && hue <= 17 || hue >= 210 && hue <= 220)  // If hue matches specific values
            {
                pros::delay(170);  // Small delay before reversing the intake
                intake.move(-127);  // Reverse the intake for a short duration
                pros::delay(300);
                intake.move(127);  // Resume intake after the reversal
            }
        }

        // Control the hood based on lift position
        if (pos < -100) {
            hood1.set_value(false);  // Retract the hood if lift position is below -100 degrees
            hood2.set_value(false);
        }
        else if (basket_state == true) {
            hood1.set_value(true);  // Extend the hood if the basket is extended
            hood2.set_value(true);
        }
        
        // Control the intake lift based on joystick position
        if (rightX > 0.85) {
            intake_lifted = true;  // Lift the intake if joystick is pushed to the right
        }
        if (rightX < -0.85) {
            intake_lifted = false;  // Lower the intake if joystick is pushed to the left
        }
        if (intake_lifted == true) {
            intake_lift.set_value(true);  // Extend the intake lift
        }
        if (intake_lifted == false) {
            intake_lift.set_value(false);  // Retract the intake lift
        }

        // Toggle the basket state with the Y button
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
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
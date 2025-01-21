#include "controls.h"
#include "auton_basics.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "hal.h"
#include "main.h"

bool intakeMode = true; //Mason Rudra place where should be


#define TURN_CONST                                                             \
  1.4 // Constant multipled by X input to allow for instant turns during driver
      // control

// Drivebase control
void taskFn_drivebase_control(void) {
  printf("%s(): Entered \n", __func__); // Log the function entry for debugging
  bool drive_state = true;    // true for normal, false for reversed drive direction
  while (true) // Infinite loop to keep checking controller input and drive base
               // state
  {
    // Get  horizontal and vertical joystick input for movement and turning
    int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    // Multiply the turning input to prioritize turning over forward movement,
    // enabling agile motion
    int turnVelleft = TURN_CONST * leftX;

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      drive_state = !drive_state; // Toggle drive direction when the X button is pressed
      pros::delay(300); // Add a small delay to avoid rapid toggling of direction
    }
    // If the drive direction is reversed, negate the joystick input for
    // forward/backward movement
    if (!drive_state) {
      leftY = -leftY;
    }

    if(leftY + turnVelleft == 0) {
      autoDrive = false;
    }

    // Control the left and right motors based on the calculated values
    drive(leftY + turnVelleft, leftY - turnVelleft);
    pros::delay(20); // loop runs at a steady pace, still avoids CPU overload
  }                  // end of while loop
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_drivebase_control


// Mogo Control
void taskFn_mogo_control(void) {
  printf("%s(): Entered \n", __func__); // Log the function entry for debugging
  bool mogo_state = false; // Track the state of the mogo clamp (false = open, true = closed)
  bool sweeper_out = false; // Track the state of the sweeper (false = retracted, true = extended)
  bool sixth_ring_state = false;

  while (true) // Infinite loop to keep checking controller input for mogo control
  {
    // When the L1 button is pressed, toggle the mogo clamp state
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      toggleClamp();
    }

    // Get the input from the right joystick and normalize the input to a range
    // of -1 to 1
    int rightY = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) / 127;
    // If the joystick is pushed upward past 85%, extend the mogo_rush arm
    // (extended)
    if (rightY > 0.85) {
      extendSweep();
    }
    // If the joystick is pushed downward past 85%, retract the mogo_rush arm
    // (retracted)
    if (rightY < -0.85) {
      retractSweep();
    }
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) 
    {
        if (sixth_ring_state == false)
        {
            sixth_ring_state = true;
            extendSixRing();
        }
        else if(sixth_ring_state == true)
        {
            sixth_ring_state = false;
            retractSixRing();
        }
    }

    pros::delay(20); // loop runs at a steady pace, still avoids CPU overload
  }
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_mogo_control


// Intake control
void taskFn_intake_control(void) {
  printf("%s(): Entered \n", __func__); // Log the function entry for debugging
  // Define an enum to represent the intake's possible states
  enum intake_state {
    INTAKE, // Intake objects
    OUTAKE, // Eject objects
    STOP  // Stop the intake
  };

  intake_color.set_led_pwm(100);  // Set the LED PWM for the intake color
  intake_state current_state = STOP; // Initialize with a default state, STOP
  int lift_counter = 0;
  bool temp_state = true;
  while (true) // Infinite loop to keep checking controller input for intake
  {
    double pos = getLiftPosition();; // Get the current position of the lift
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      if ((current_state == OUTAKE || current_state == STOP) && intakeMode) // If the intake is stopped or ejecting, start intake
      {
        //autoIntake = false;
        current_state = INTAKE;
        spinIntake(127);

      } 
      else if (current_state == INTAKE) // If intake is running, stop it
      {
        //autoIntake = false;
        current_state = STOP;
        stopIntake();
        clearRingQueue();
      }
    }
    // Eject objects with the B button
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      if ((current_state == INTAKE || current_state == STOP) && intakeMode) // If intake is running or stopped, start ejecting
      {
        //autoIntake = false;
        spinIntake(-127);
        clearRingQueue();
        current_state = OUTAKE;

      } 
      else if (current_state == OUTAKE) // If intake is ejecting, stop it
      {
        //autoIntake = false;
        current_state = STOP;
        clearRingQueue();
        stopIntake();
      }
    }
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      if (autoIntake == false)
      {
        startSorting();
      }
      else if (autoIntake == true) // If intake is ejecting, stop it
      {
        closeRedirect();
        stopSorting();
      }
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      if(getLiftPosition() > 86 || getLiftPosition() < 76)
      {
        if(lift_counter == 0){
          pros::Task lift_wall_stake([=] {moveLiftToPos(81);});
        } 
      }
      
      else if(lift_counter > 15) {
        temp_state = false;

        if(getLiftPosition() > 100){
          stopIntakeHold();
        } else {
          spinIntake(127);
        }

        if(getLiftPosition() < 24){
          liftPneumaticUp();
        }
        else if(getLiftPosition()){
          liftPneumaticDown();
        }
      }
      lift_counter++;
    } 
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      liftPneumaticDown();
      lift_counter = 0;
      temp_state = false;
      spinIntake(-127);
    } 
    else if(!temp_state){
      lift_counter = 0;
      if(getLiftPosition() > 24 && getLiftPosition() < 300){
        liftPneumaticDown();
        stopIntakeHold();
      }
      else{
        stopIntake();
        temp_state = true;
      }
    }
    pros::delay(20);
  }
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_intake_control

// Intake control
void taskFn_hood_control(void) {
  bool hood_state = false;
  printf("%s(): Entered \n", __func__);
  while (true) {
    int rightX = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127; // Normalize the right joystick input to -1 to 1
    // Toggle the basket state with the Y button
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      toggleHood();
      if(!getHood()){
        stopSorting();
        closeRedirect();
      } else {
        redirectRings();
      }
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
      stopSorting();
      toggleRedirect();
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
      extendRushClamp();
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
      retractRushClamp();
    }
    // Control the intake lift based on joystick position
    if (rightX > 0.85) {
      liftIntake();
    }
    if (rightX < -0.85) {
      dropIntake();
    }
    pros::delay(20);
  }


  printf("%s(): Exiting \n", __func__);
} // end of taskFn_auto_intake_push_control
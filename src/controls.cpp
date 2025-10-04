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
bool tankDrive = false;

#define TURN_CONST                                                             \
  1.4 // Constant multipled by X input to allow for instant turns during driver

// Drivebase control
void taskFn_drivebase_control(void) {
  printf("%s(): Entered \n", __func__); // Log the function entry for debugging
  bool drive_state = true;    // true for normal, false for reversed drive direction
  int leftX, leftY, turnVelleft, rightY;

  enum brake_mode{
    COAST,
    HOLD
  };

  brake_mode current_brake = COAST;

  while (true) // Infinite loop to keep checking controller input and drive base
               // state
  {
    // Get  horizontal and vertical joystick input for movement and turning
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);


    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      drive_state = !drive_state; // Toggle drive direction when the X button is pressed
      pros::delay(200); // Add a small delay to avoid rapid toggling of direction
    }

    // If the drive direction is reversed, negate the joystick input for
    // forward/backward movement
    if (!drive_state) {
      leftY = -leftY;
    }

    // Multiply the turning input to prioritize turning over forward movement,
    // enabling agile motion
    int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    turnVelleft = TURN_CONST * leftX;
    
    // Control the left and right motors based on the calculated values
    drive(leftY + turnVelleft, leftY - turnVelleft);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && leftY == 0 && leftX == 0){
      if(current_brake == COAST){
        current_brake = HOLD;
        setDriveBrake(pros::E_MOTOR_BRAKE_HOLD);
      }
    } else if(current_brake == HOLD){
      current_brake = COAST;
      setDriveBrake(pros::E_MOTOR_BRAKE_COAST);
    }

    if(leftY + turnVelleft == 0) {
      autoDrive = false;
    }

    pros::delay(20); // loop runs at a steady pace, still avoids CPU overload
  }                  // end of while loop
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_drivebase_control


//Intake control
void taskFn_intake_control(void){
    printf("%s(): Entered \n", __func__);

    bool basket_state = false;
    bool intake_lifted = false;
    while (true) 
    {
        
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) 
      {
        if (current_intake != INTAKE) // If intake is running, stop it
        {
          intakeAll(127);
        }
        else {
          stopAllIntake();
        }
      }
      // Eject objects with the B button
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        if (current_intake != OUTAKE) // If intake is running or stopped, start ejecting
        {
          outakeAll(127);
        } 
        else if (current_intake == OUTAKE) // If intake is ejecting, stop it
        {
          stopAllIntake();
        }
      }

      if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
      {
        toggleLoader();
      }

      if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
      {
        toggleStopper();
      }

      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) 
      {
        toggleGoal();
      }

      if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        if(antiJam){
          stopAntiJam();
        } else{
          startAntiJam();
        }
      }
      
      pros::delay(20);
    }
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_intake_control


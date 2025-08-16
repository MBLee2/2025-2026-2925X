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
enum intake_state {
  INTAKE, // Intake objects
  OUTAKE, // Eject objects
  STOP  // Stop the intake
};
intake_state current_state = STOP;
bool LBPickup1 = false;
#define TURN_CONST                                                             \
  1.4 // Constant multipled by X input to allow for instant turns during driver

// Drivebase control
void taskFn_drivebase_control(void) {
  printf("%s(): Entered \n", __func__); // Log the function entry for debugging
  bool drive_state = true;    // true for normal, false for reversed drive direction
  int leftX, leftY, turnVelleft, rightY;
  while (true) // Infinite loop to keep checking controller input and drive base
               // state
  {
    // Get  horizontal and vertical joystick input for movement and turning
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    // Multiply the turning input to prioritize turning over forward movement,
    // enabling agile motion

    // Control the left and right motors based on the calculated values
    if(!tankDrive){
      int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
      turnVelleft = TURN_CONST * leftX;
      drive(leftY + turnVelleft, leftY - turnVelleft);
    } else {
      int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
      drive(leftY, rightY);
    }

    /*if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      drive_state = !drive_state; // Toggle drive direction when the X button is pressed
      pros::delay(300); // Add a small delay to avoid rapid toggling of direction
    // If the drive direction is reversed, negate the joystick input for
    // forward/backward movement
    if (!drive_state) {
      leftY = -leftY;
    }
    }*/

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
    enum intake_state {
      INTAKE,
      MIDSCORE,
      OUTAKE,
      STOP
    };
    bool basket_state = false;
    intake_state current_state = STOP;  // Initialize with a default state, STOP
    bool intake_lifted = false;
    while (true) 
    {
        
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) 
      {
        if (current_state == OUTAKE || current_state == STOP)
        {
          spinIntake(127);
          spinScoring(127);
          spinStorage(127);
          spinReload(127);
          current_state = INTAKE;
        } 
        else if (current_state == INTAKE || current_state == MIDSCORE) // If intake is running, stop it
        {
          current_state = STOP;
          stopAllIntake();
        }
      }
      // Eject objects with the B button
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        if (current_state == INTAKE || current_state == MIDSCORE || current_state == STOP) // If intake is running or stopped, start ejecting
        {
          spinIntake(-127);
          stopScoring();
          stopStorage();
          spinReload(127);
          current_state = OUTAKE;
        } 
        else if (current_state == OUTAKE) // If intake is ejecting, stop it
        {
          current_state = STOP;
          stopAllIntake();
        }
      }

      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) 
      {
        if (current_state == MIDSCORE || current_state == OUTAKE || current_state == STOP) // If intake is running or stopped, start ejecting
        {
          spinIntake(127);
          spinScoring(127);
          spinStorage(127);
          spinReload(127);
          current_state = INTAKE;
        } else if(current_state == INTAKE){
          current_state = STOP;
          stopAllIntake();
        }
      }
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) 
      {
        if (current_state == INTAKE || current_state == OUTAKE || current_state == STOP) // If intake is running or stopped, start ejecting
        {
          spinIntake(127);
          spinScoring(-127);
          spinStorage(127);
          spinReload(127);
          current_state = MIDSCORE;
        } else if(current_state == MIDSCORE){
          current_state = STOP;
          stopAllIntake();
        }
      }
      
    }
    pros::delay(20);
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_intake_control


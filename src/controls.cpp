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

    bool basket_state = false;
    bool intake_lifted = false;
    while (true) 
    {
        
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) 
      {
        if (current_intake != INTAKE)
        {
          intakeAll(127);
        } 
        else if (current_intake == INTAKE) // If intake is running, stop it
        {
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


      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) 
      {
        if (current_intake != TOPSCORE) // If intake is running or stopped, start ejecting
        {
          scoreTop(127);
        } else if(current_intake == TOPSCORE){
          stopAllIntake();
        }
      }
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) 
      {
        if (current_intake != MIDSCORE) // If intake is running or stopped, start ejecting
        {
          scoreMiddle(127);
        } else if(current_intake == MIDSCORE){
          stopAllIntake();
        }
      }

      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
      {
        if(current_reload == FROM_INTAKE){
          current_reload = FROM_STORAGE;

          if(current_intake == TOPSCORE){
            topFromStorage(127);
          } else if (current_intake == MIDSCORE){
            middleFromStorage(127);
          }
        } else if (current_reload == FROM_STORAGE){
          stopAllIntake();
        }
      }

      if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
      {
        stopAllIntake();
      }
      
      pros::delay(20);
    }
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_intake_control


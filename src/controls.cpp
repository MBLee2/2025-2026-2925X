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
      // control

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
    if(!tankDrive){
      int rightY = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) / 127;
      int rightX = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127; // Normalize the right joystick input to -1 to 1
      // (retracted)
      if (rightY < -0.85) {
        retractRightSweeper();
        retractLeftSweeper();
      }
      if (rightX > 0.85) { 
        extendRightSweeper();
        retractLeftSweeper();
      }
      if (rightX < -0.85) {
        extendLeftSweeper();
        retractRightSweeper();
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
  int counter = 0;
  intake_color.set_led_pwm(25);  // Set the LED PWM for the intake color
  while (true) // Infinite loop to keep checking controller input for intake
  {
    double pos = getLiftPosition();; // Get the current position of the lift
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      if ((current_state == OUTAKE || current_state == STOP)) // If the intake is stopped or ejecting, start intake
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
        stopSorting();
      }
    }
    // Eject objects with the B button
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      if ((current_state == INTAKE || current_state == STOP)) // If intake is running or stopped, start ejecting
      {
        //autoIntake = false;
        spinIntake(-127);
        stopSorting();
        current_state = OUTAKE;

      } 
      else if (current_state == OUTAKE) // If intake is ejecting, stop it
      {
        //autoIntake = false;
        current_state = STOP;
        stopIntake();
      }
    }
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      if (autoIntake == false && current_state != OUTAKE)
      {
        startSorting();
      }
      else if (autoIntake == true) // If intake is ejecting, stop it
      {
        stopSorting();
      }
    }
    pros::delay(20);
  }
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_intake_control

void taskFn_lift_control(void)
{
  printf("%s(): Entered \n", __func__); // Log the function entry for debugging
  // Define an enum to represent the intake's possible states
  enum lift_state {
    WALLSTAKE, // Intake objects
    PICKUP, // Eject objects
    DOWN  // Stop the intake
  };
  float target = 0;
  int time = 0;
  bool dir = true;
  autoLift = false;
  int toggle_counter = 0;

  ladybrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  lift_state current_state1 = DOWN; // Initialize with a default state, STOP

  while (true) // Infinite loop to keep checking controller input for intake
  { 
    while(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      spinLift(80);
      autoLift = false;
    }
    while(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      spinLift(-80);
      autoLift = false;
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      stopIntake();
      current_state = STOP;
      pros::Task ws_task(liftUpWallStake);
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      pros::Task load_task(liftPickup);
      LBPickup1 = true;
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      retractLeftSweeper();
      tankDrive = true;
      moveLiftToPos(160, 127, 2000);
      closePTO();
      retractClimbBalance();
      pros::delay(1500);
      openPTO();

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && toggle_counter > 10) {
      retractLeftSweeper();
      tankDrive = true;
      togglePTO();
      toggle_counter = 0;
    }
    if(!autoLift){
      stopLift();
    }
    resetLiftPositionWithDistance();
    pros::delay(20);
    toggle_counter++;
  }
}

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

#define TURN_CONST                                                             \
  1.4 // Constant multipled by X input to allow for instant turns during driver
      // control

// Drivebase control
void taskFn_drivebase_control(void) {
  printf("%s(): Entered \n", __func__); // Log the function entry for debugging
  bool drive_state =
      true;    // true for normal, false for reversed drive direction
  while (true) // Infinite loop to keep checking controller input and drive base
               // state
  {
    // Get  horizontal and vertical joystick input for movement and turning
    int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    // Multiply the turning input to prioritize turning over forward movement,
    // enabling agile motion
    int turnVelleft = TURN_CONST * leftX;

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
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

// Lift control
void taskFn_lift_control(void) {
  printf("%s(): Entered \n", __func__); // Log the function entry for debugging
  setLiftBrake(pros::E_MOTOR_BRAKE_HOLD);
  while (true) // Infinite loop to keep checking controller input for lift control
  {
    // While the R1 button is pressed, move the lift up at full speed
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      autoLift = false;
      moveLift(127); // Move the lift up
    }
    // While the R1 button is pressed, move the lift down at full speed
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      autoLift = false;
      moveLift(-127); // Move the lift down
      // If R1 is pressed while R2 is still held, move the lift up instead
      while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        moveLift(127); // Move the lift up
      }
    }
    stopLift();    // If neither R1 nor R2 is pressed, stop the lift
    pros::delay(20); // loop runs at a steady pace, still avoids CPU overload
  }
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_lift_control

// Mogo Control
void taskFn_mogo_control(void) {
  printf("%s(): Entered \n", __func__); // Log the function entry for debugging
  bool mogo_state = false; // Track the state of the mogo clamp (false = open, true = closed)
  bool sweeper_out = false; // Track the state of the sweeper (false =
                            // retracted, true = extended)
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
    STOP    // Stop the intake
  };

  bool have_seen = false;

  int counter = 200;
  intake_state current_state = STOP; // Initialize with a default state, STOP

  setLiftEncoder(pros::E_MOTOR_ENCODER_DEGREES); // Set the encoder units for the lift
                                      // motor to degrees
  // intake_color.set_led_pwm(100);  // Set the LED PWM for the intake color
  // sensor to 100

  while (true) // Infinite loop to keep checking controller input for intake
               // control
  {
    double pos = getLiftPosition();; // Get the current position of the lift
    int rightX = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127; // Normalize the right joystick input to -1 to 1
    // int hue = intake_color.get_hue();  // Get the current hue value from the
    // intake color sensor master.print(1, 0, "C: %i", hue);

    // Toggle intake on or off with the A button
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      if (current_state == OUTAKE || current_state == STOP) // If the intake is stopped or ejecting, start intake
      {
        spinIntake(127);
        autoIntake = false;
        current_state = INTAKE;
      } else if (current_state == INTAKE) // If intake is running, stop it
      {
        stopIntake();
        autoIntake = false;
        current_state = STOP;
      }
    }

    // Eject objects with the B button
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      if (current_state == INTAKE || current_state == STOP) // If intake is running or stopped, start ejecting
      {
        spinIntake(-127); // Reverse the intake to eject
        autoIntake = false;
        current_state = OUTAKE;

      } else if (current_state == OUTAKE) // If intake is ejecting, stop it
      {
        stopIntake(); // Stop the intake
        autoIntake = false;
        current_state = STOP;
      }
    }

    // Previous color sensor logic: ((hue >= 7 && hue <= 17) || (hue >= 210 &&
    // hue <= 240))
    //  Control intake based on color sensor readings when basket is extended
    if (basket_state == false && current_state == INTAKE) {

      if (counter > 200 && (have_seen && getIntakeDist() > 20)) // If hue matches specific values
      {
        spinIntake(90);
        while (getIntakeDist() < 30) // If hue matches specific values
        {
          pros::delay(10);
        }
        intakeFor(90, 19.f);
        outakeFor(105, 370);
        spinIntake(127);
        have_seen = false;
        counter = 0;
      } else {
        if (!have_seen && intake_dist.get() < 30) {
          have_seen = true;
        }
        counter++;
      }

      if(have_seen){
        spinIntake(90);
      } else {
        spinIntake(127);
      }
    }

    // Control the hood based on lift position
    /*if (pos < -100) {
      hoodBwd();
    } else if (basket_state == true) {
      hoodFwd();
    }*/

    // Control the intake lift based on joystick position
    if (rightX > 0.85) {
      liftIntake();
    }
    if (rightX < -0.85) {
      dropIntake();
    }

    pros::delay(10);
  }
  printf("%s(): Exiting \n", __func__); // Log the function exit for debugging
} // end of taskFn_intake_control

/*void taskFn_auton_intake_control(void){
    printf("%s(): Entered \n", __func__);  // Log the function entry for
debugging

    bool basket_state = false;  // Track the state of the basket (false =
retracted, true = extended) bool intake_lifted = false;  // Track whether the
intake is lifted (false = down, true = up)

    lift.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);  // Set the encoder
units for the lift motor to degrees intake_color.set_led_pwm(100);  // Set the
LED PWM for the intake color sensor to 100

    while (true)  // Infinite loop to keep checking controller input for intake
control
    {
        double pos = lift.get_position();  // Get the current position of the
lift int hue = intake_color.get_hue();  // Get the current hue value from the
intake color sensor

        // Control intake based on color sensor readings when basket is extended
        if (basket_state == true)
        {
            if (hue >= 7 && hue <= 17 || hue >= 210 && hue <= 220)  // If hue
matches specific values
            {
                pros::delay(170);  // Small delay before reversing the intake
                intake.move(-127);  // Reverse the intake for a short duration
                pros::delay(300);
                intake.move(127);  // Resume intake after the reversal
            }
        }

        // Control the hood based on lift position
        if (pos < -100) {
            hood1.set_value(false);  // Retract the hood if lift position is
below -100 degrees hood2.set_value(false);
        }
        else if (basket_state == true) {
            hood1.set_value(true);  // Extend the hood if the basket is extended
            hood2.set_value(true);
        }
    }
    printf("%s(): Exiting \n", __func__);  // Log the function exit for
debugging } // end of taskFn_intake_control*/

// Intake control
void taskFn_hood_control(void) {
  /*sudo code
  if color sensor sees wrong color
  //*/
  bool hood_state = false;
  printf("%s(): Entered \n", __func__);
  while (true) {
    // Toggle the basket state with the Y button
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      toggleRedirect();
    }

    pros::delay(20);
  }

  printf("%s(): Exiting \n", __func__);
} // end of taskFn_auto_intake_push_control
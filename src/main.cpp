#include "main.h"
#include "auton_basics.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "dashboard.h"
#include "lemlib/api.hpp"
#include "controls.h"
#include "pros/device.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "hal.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
int counter = 0;


void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}

auton_routine selected_auton_routine = null_routine;
void initialize() {

    /**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
	{
    pros::lcd::initialize(); // ONLY FOR TUNING PID	
    chassis.calibrate(); // ONLY FOR TUNING PID
	
	// weird bug in system; without the following delay, was getting a white screen
	// on the brain rather than the display as expectedF
	pros::delay(10); 
	// Clear the Brain screen and show status
	resetLiftPosition();

	setDriveBrake(pros::E_MOTOR_BRAKE_COAST);
	

    setIntakeEncoder(pros::E_MOTOR_ENCODER_DEGREES);
	setIntakeBrake(pros::E_MOTOR_BRAKE_COAST);

	setIntakeColorLED(100);
    setIntakeColor2LED(100);
	sort_color_queue();
	autoIntake = false;
	
    pros::screen::set_eraser(pros::c::COLOR_BLACK);
	pros::screen::erase();
	pros::screen::set_pen(pros::c::COLOR_ANTIQUE_WHITE);
	pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Running initialize()");//*/
	
	/*// reset the rotation sensors
	rotL.reset_position();
	rotR.reset_position();//*/


	/** Reset (calibrate) IMU and ensure correct status 
	 *  Status values expected:
	 *		1			= Reset Succesfully
	 *		2147483647	= Calibration in progress or error!!
	 * 		ENXIO		= Error - The port value is not within the range of V5 ports (1-21).
	 * 		ENODEV		= Error - The port cannot be configured as an Inertial Sensor.
	 * 		EAGAIN		= Error - The sensor is already calibrating
	 * 		other value	= Error - unknown */
	/*int imu_reset_status;
    int imu_reset_start_time = pros::millis();
	while ((imu_reset_status = imu.reset()) != 1)
	{
		printf("%s(): imu.reset() Status = [%d]\n", __func__, imu_reset_status);	

		// Just wait if calibration is in progress
		if (imu_reset_status == 2147483647)
		{
			// IMU calibration should take little over 2 seconds
		    if ((pros::millis() - imu_reset_start_time) < 2100 )
			{
				pros::screen::set_pen(COLOR_YELLOW);
				pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Waiting 2 seconds for IMU calibration...");
				pros::delay(100);
				continue;
			}
		}

		// Else ask user to fix IMU issues; may need to restart the program or plugin the IMU in 
		// the correct  port as configured in robot_config.h
		printf("%s(): Possible imu.reset() FAILURE - Fix IMU issues to continue!\n", __func__);
		// Flash an error message in alternating colors on the Brain screen to get attention!
		if (pros::screen::get_pen() != COLOR_RED)
			pros::screen::set_pen(COLOR_RED);
		else if (pros::screen::get_pen() == COLOR_RED)
			pros::screen::set_pen(COLOR_ORANGE);
		pros::screen::print(pros::E_TEXT_MEDIUM, 3, "IMU calibration Error!! [%d]   ", imu_reset_status);
		pros::screen::print(pros::E_TEXT_MEDIUM, 4, "FIX IMU issues to continue!!");
		pros::delay(300);
	
	} // end while (1)

	printf("%s(): Exiting\n", __func__);*/

	// Clear the Brain screen and show status
    pros::screen::set_eraser(pros::c::COLOR_BLACK);
	pros::screen::erase();

    } // end initialize()
    pros::Task screenTask(screen); // create a task to print the position to the screen HERE
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	chassis.setPose(0,0,0);
    /**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
{
	//HERE
	
	printf("%s(): Entered\n", __func__);

	// weird bug in system; without the following delay, was getting a white screen
	// on the brain rather than the display as expected
	pros::delay(10); 


	// select the auton from the menu
	selected_auton_routine = select_auton_routine();

	// Clear the Brain screen and show status
    pros::screen::set_eraser(pros::c::COLOR_BLACK);
	pros::screen::erase();
	pros::screen::set_pen(pros::c::COLOR_ANTIQUE_WHITE);
	pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Running competition_initialize()");//*/


	}

	printf("%s(): Exiting\n", __func__);
	
	// Clear the Brain screen and show status
    pros::screen::set_eraser(pros::c::COLOR_BLACK);
	pros::screen::erase();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

ASSET(path1_txt);
ASSET(rush6ball_txt);
ASSET(test_txt);

void autonomous() {
	// Clear the Brain screen
	//chassis.moveToPoint(0, 30, 10000);
	//HERE
	
	auton_routine default_routine = safe_positive; //DEFAULT ROUTINE

	auton = true;
   	printf("%s(): Entered\n", __func__);
	pros::screen::set_eraser(pros::c::COLOR_BLACK);
	pros::screen::erase();
	pros::screen::set_pen(pros::c::COLOR_ANTIQUE_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Running autonomous()");
	
	// ensure that an auton routine has been slected
	if (selected_auton_routine.routine_func == nullptr)
	{
		selected_auton_routine = default_routine;
		pros::screen::set_pen(pros::c::COLOR_RED);
		pros::screen::print(pros::E_TEXT_LARGE, 3, "No Auton routine selected");
		pros::screen::print(pros::E_TEXT_LARGE, 4, "Default Auton: safe_positive");
	}

	// Call the function associated with the selected auton routine		
	selected_auton_routine.routine_func();//*/

	// Start the independent parallel tasks needed to support autonomous mode
	//pros::Task dashboard_task(taskFn_dashboard_display, "dashboard-task");
	//pros::Task drivebase_task(taskFn_display_gps_coordinates, "gps-display-task");
	
	printf("%s(): Exiting\n", __func__);

}

/**xx
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	master.clear();
	pros::Task dashboard_task(taskFn_dashboard_display, "dashboard-task");
    pros::Task drivebase_task(taskFn_drivebase_control,"drivebase-task");	
    pros::Task mogo_task(taskFn_mogo_control,"mogo-task");
	pros::Task intake_task(taskFn_intake_control,"intake-task");
	pros::Task hood(taskFn_hood_control,"intake_push_task");
    
}
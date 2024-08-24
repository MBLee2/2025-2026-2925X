#include "auton_basics.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "lemlib/pose.hpp"
#include "robot_config.h"
#include "controls.h"
#include "lemlib/api.hpp"

double radToDeg(double rad){
    return rad * (180 / PI);
}

//distance sensor constants
const double LEFT_SPACING = 260.35;
const double RIGHT_SPACING = 139.7;
const double BACK_SPACING = 330.2;
const double LEFT_DIFFERENCE = 11.1125;
const double RIGHT_DIFFERENCE = 11.1125;

void readjustHeading(int side, double roundedHeading)
/**
 * @brief Calculate and adjust the robot's heading using trigonometry and distance sensors
 *  mounted on the sides of the robot measuring against the wall
 * @param side which side of the robot is being used; 0 - Right, 1 - Left, 2 - Back
 * @param roundedHeading robot's approxiamated direction in multiples of 90, which wall the front is facing
 */
{
    double newHeading;
    
    if(side == 0){
        newHeading = radToDeg(atan((distance_rb.get() - distance_rf.get() - RIGHT_DIFFERENCE) / RIGHT_SPACING));
    } else if (side == 1) {
        newHeading = radToDeg(atan((distance_lf.get() - distance_lb.get() - LEFT_DIFFERENCE) / LEFT_SPACING));
    } else {
        newHeading = radToDeg(atan((distance_bl.get() - distance_br.get()) / BACK_SPACING));
    }

    newHeading += roundedHeading;

    chassis.setPose(chassis.getPose().x, chassis.getPose().y, newHeading);
}

// /*
//     Diffrent auton functions that Rudra made for start of OU season and are not the best but work for very short movments 
//     There are also a lot of conversion functions so I am not deleteing it 
// */


// // Conversion fuctions
// double cartesian2compass(double cartesian_angle) 
// /**
//  * @brief Convert a heading calculated using trignometry function to the heading compatible 
//  *  with the field GPS navigation system (compass angle).
//  * @param cartesian_angle Cartesian angle, measured in degrees
//  * @return Compass angle, measured in degrees
//  *      Returns -999.999 in case of failure
// */
// {
//     //printf("%s(%3.2f) called \n", __func__, cartesian_angle);

//     double compass_angle = -999.999;

//     // sanity check the specified heading values to be within range
//     if(cartesian_angle < 0 || cartesian_angle > 359.9999)
//     {
//         printf("%s(): FAILURE - Invalid cartesian_angle (%3.2f)", __func__, cartesian_angle);
//         return compass_angle;
//     }

//     if ((cartesian_angle >= 0.0) && (cartesian_angle <= 90.0)) {
//         compass_angle = 90.0 - cartesian_angle;
//     } else {
//         compass_angle = 450.0 - cartesian_angle;
//     }

//     //printf("%s(): SUCCESS: Returned [%3.2f] degrees compass (GPS)\n", __func__, compass_angle);
//     return compass_angle;

// } // end cartesian2compass

// double compass2cartesian(double compass_angle) 
// /**
//  * @brief Convert a heading seen by the field GPS navigation system (compass angle) 
//  *  to cartesian angles as needed by trignometric functions
//  * @param compass_angle Compass angle, measured in degrees 
//  * @return Cartesian angle, measured in degrees. 
//  *      Returns -999.999 in case of failure
// */
// {
//     //printf("%s(%3.2f) called \n", __func__, compass_angle);

//     double cartesian_angle = -999.999;
    
//     // sanity check the specified heading values to be within range
//     if(compass_angle < 0 || compass_angle > 359.9999)
//     {
//         printf("%s(): FAILURE - Invalid compass_angle (%3.2f)", __func__, compass_angle);
//         return cartesian_angle;
//     }

//     if ((compass_angle >= 0.0) && (compass_angle <= 90.0)) {
//         cartesian_angle = 90.0 - compass_angle;
//     } else {
//         cartesian_angle = 450.0 - compass_angle;
//     }

//     //printf("%s(): SUCCESS: Returned [%3.2f] degrees cartesian\n", __func__, cartesian_angle);
//     return cartesian_angle;

// } // end compass2cartesian

// double get_heading_diff(double initial_heading, double target_heading, rotation_direction rot_dir)
// /**
//  * @brief Find the difference in heading between the intial and target headings, both measured in degrees.
//  * Both the headings are expected to be compatible with the field GPS navigation system (compass angle).
//  * Takes into account the specified direction of rotation. If either (CLOCKWISE or ANTICLOCKWISE) is acceptable
//  * the more efficient direction - i.e. the shoter angular distance - is calculated. Note that the output of this 
//  * function is most likely going to be used to call the yaw_by() function or to calculate the needed rotation in yaw_to_heading() 
//  * in order to reach the target heading.
//  * 
//  * @param initial_heading heading at the starting, in degrees. Valid Range: 0 to 359.9999
//  * @param target_heading heading to move to, in degrees. Valid Range: 0 to 359.9999
//  * @param rot_dir rotation direction; See definition of ENUM e_rotation_direction. Default: E_ROTATE_EITHER
//  *  
//  * @return Difference in the two headings values, measured in degrees. The sign indicates direction of 
//  *  rotation recommended to achieve target heading with +ve for Clockwise and -ve for Aniti-clockwise
//  *  Valid Range: -359.99 to 359.9999. Returns -999.999 for failure cases
// */
// {
//     /*printf("%s(%+03.2f, %+03.2f, %d) called \n", __func__, 
//         initial_heading, target_heading, rot_dir); //*/

//     double return_value = -999.999;

//     // sanity check the specified heading values to be within range
//     if(initial_heading < 0 || initial_heading > 359.9999)
//     {
//         printf("%s(): FAILURE - Invalid initial_heading (%f)\n", __func__, initial_heading);
//         return return_value;
//     }
//     if(target_heading < 0 || target_heading > 359.9999)
//     {
//         printf("%s(): FAILURE - Invalid target_heading (%f)\n", __func__, target_heading);
//         return return_value;
//     }

//     // calculate the actual difference between the two headings
//     double heading_difference = target_heading - initial_heading;
//     // This value will always be between -359.999 and 359.999
    
//     if (rot_dir == E_ROTATE_EITHER) 
//     // Choose Clockwise or Anti-clockeise, based on the least angular distance
//     // The returned value will always be between -180 and +180
//     {
//         if (std::abs(heading_difference) <= 180 )
//         {
//             return_value = heading_difference; // this will be either
//             // A Clockwise (i.e. +ve) value between 0 and 180 or 
//             // An Anit-clockwise (i.e. -ve) value between -180 and 0
//             // or 0
//         }
//         else if (heading_difference > 180)
//         {
//             return_value = heading_difference - 359.9999; // this will be the shortest angular distance
//             // which is always an Anti-clockwise (i.e. -ve) value, between -180 and 0
//         }
//         else // (heading_difference < -180)
//         {
//             return_value = 359.9999 + heading_difference; // this will be the shortest angular distance
//             // which is always a Clockwise (i.e. +ve) value, between 0 and 180
//         }
//     }
//     else if (rot_dir == E_ROTATE_CLOCKWISE) 
//     // Choose the Clockwise (i.e. +ve value) only, irrespective of the shorter angular distance
//     // The returned value will always be between 0 and +359.999
//     {
//         if (heading_difference > 0)
//         {
//             return_value = heading_difference; 
//         }
//         else
//         {
//             return_value = 359.9999 + heading_difference;
//         }
//     }
//     else if (rot_dir == E_ROTATE_ANTICLOCKWISE) 
//     // Choose the Anti-clockwise (i.e. -ve value) only, irrespective of the shorter angular distance
//     // The returned value will always be between -359.999 and 0
//     {
//         if (heading_difference < 0)
//         {
//             return_value = heading_difference; 
//         }
//         else
//         {
//             return_value = heading_difference - 359.9999;
//         }
//     }
//     else
//     {
//         printf("%s(): FAILURE - Invalid rot_dir (%d)\n", __func__, rot_dir);
//         return return_value;
//     }

//     /*printf("%s(): Heading Difference is [%f]\n", __func__, return_value);//*/
//     return return_value;
    
// } // end get_heading_diff()


// double degrees2inches (double degrees)
// {
//     double inches = ((degrees / 360) * (DRIVEBASE_GEAR_SIZE_ON_MOTOR / DRIVEBASE_GEAR_SIZE_ON_WHEEL) * PI * DRIVEBASE_WHEEL_DIAMETER);
    
//     return inches;
// }

// // Move fuctions with Odometry

// bool move_drivetrains(float left_target_dist, float right_target_dist, int time_out, int max_voltage, float dist_margin)
// /**
//  * @brief Move the robot's left and right drivetrains independently to the specified distances. 
//  * @note This independent control of the two drivetrains allows this function to be reused / wrapped inside  
//  * other functions - e.g. It can be used to implement a lnear (straight line) movement path by using the
//  * same distance for left and right distances. It can also be used to implement a curvilinear (arc) movement 
//  * path by calculating the different distances the left and right drivetrains need to cover, and passing
//  * the same on to this function. 
//  * @note It relies on the V5 rotation sensor (left and right sensors) based odometry. 
//  * 
//  * Sensors used:
//  *      - V5 Rotational Sensor: https://kb.vex.com/hc/en-us/articles/360051368331-Using-the-V5-Rotation-Sensor
//  * 
//  * Features:
//  *  -   [Optional] The max time that this function needs to be completed within can be specified
//  *  -   [Optional] The max speeed reached during the movement can be specified.
//  *  -   [Optional] The acceptable error margin can be specified to accommodate real life scenarios
//  * 
//  * @param left_target_dist Distance in inches that the LEFT drivetrain of the robot needs to move 
//  *      Direction specified with sign: +ve (forward), -ve (backward).
//  * @param right_target_dist Distance in inches that the RIGHT drivetrain of the robot needs to move 
//  *      Direction specified with sign: +ve (forward), -ve (backward).
//  * @param time_out The maximum time in milli seconds the function can take to complete.
//  *      It ruturns a failure if this time is exceeded. Default: 60 seconds (max auton time)
//  * @param max_voltage Maximum volage that can be applied to the drivetrains
//  *      Valid Range: DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL to +127. Default: +127
//  * @param dist_margin The error at which the target will deemed to have been reached. 
//  * 			Default: DRIVEBASE_ACCEPTABLE_MOVE_ERROR_IN
//  *  
//  * @return bool - indicating if it reached the target distance or not
// */
// {   
//     // start the time-out timer
//     int func_start_time = pros::millis();
    
//     printf("\n%s(%f, %f, %d, %d, %f) called \n", __func__, 
//         left_target_dist, right_target_dist, time_out, max_voltage, dist_margin);

//     // check if the distances are trivial and to be ignored or actually needs working on
//     bool left_side_motors_done = false;
//     if (std::abs(left_target_dist) < dist_margin) 
//     {
//         printf("%s(): Specified left_target_dist (%.3f) is lower than configured " \
//             "dist_margin (%.3f) - No Action taken on left drivetrain \n", __func__, left_target_dist, dist_margin);
//         left_side_motors_done  = true;
//     }
//     bool right_side_motors_done = false;
//     if (std::abs(right_target_dist) < dist_margin) 
//     {
//         printf("%s(): Specified right_target_dist (%.3f) is lower than configured " \
//             "dist_margin (%.3f) - No Action taken on right drivetrain \n", __func__, right_target_dist, dist_margin);
//         right_side_motors_done  = true;
//     }
//     if (left_side_motors_done && right_side_motors_done)
//     {
//         printf("%s(): No Action needed! \n", __func__);
//         return true;
//     }

//     // ensure the drivetrains get the minimum viable move voltage
//     if (max_voltage < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) 
//     {
//         printf("%s(): Specified max_voltage (%d) is lower than configured " \
//             "DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL (%d) - Using DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL instead \n", 
//             __func__, max_voltage, DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL);
//         max_voltage = DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL;
//     }

//     // -- Left Drivetrain setup --
//     // Set brake mode to HOLD
//     left_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
//     // Tare the possition of the left drivetrain motors
//     left_back_motor.tare_position();
//     left_mid_motor.tare_position();
//     left_front_motor.tare_position();
//     /*// tare the rotation sensor position before starting
//     rotL.set_position(0);//*/
    
//     // -- Right Drivetrain setup --
//     // Set brake mode to HOLD
//     right_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
//     // Tare the possition of the left drivetrain motors
//     right_front_motor.tare_position();
//     right_mid_motor.tare_position();
//     right_back_motor.tare_position();
//     /*// tare the rotation sensor position before starting
//     rotR.set_position(0);//*/
   
//     int loop_duration = 20;     // loop interval in milliseconds
//     int iter = 0;           // iteration counter
//     int execution_time = 0;
//     bool success = false;
    
//     while(true)
//     {      
//         // -- Left Drivetrain processing --
//         // Read current positions
//         double left_pos = ((left_front_motor.get_position() + left_mid_motor.get_position() + left_back_motor.get_position()) / 3);
//         double left_dist_traveled = degrees2inches(left_pos);
//         /*double left_dist_traveled = rotL.get_position() * inches_per_tick;//*/
//         double left_remaining_dist = left_target_dist - left_dist_traveled;
//         printf("%s(): left_side_motors  Covered: %+3.3f, Remaining: %+3.3f \n", 
//             __func__, left_dist_traveled, left_remaining_dist);
//         // move if target is not yet reached
//         if (std::abs(left_remaining_dist) > dist_margin)
//         {
//             left_side_motors_done = false; 
//             // use a voltage that is proportional (P of PID) to distance remaining
//             // Note that the sign will reverse if the drivetrain overshoots its target distance
//             int left_voltage = round(max_voltage * (1 - left_dist_traveled/left_target_dist));
//             // note that the above calculation also takes care of overshoot. If overshot,
//             // the left_dist_traveled > left_target_dist, resulting in a -ve voltage calculation, which will 
//             // reverse the direction of rotation of the left drivetrain

//             // ensure the drivetrains get the minimum viable move voltage
//             if (std::abs(left_voltage) < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) 
//             {
//                 left_voltage = (std::signbit(left_voltage)) ? -DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL : DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL;
//             }
//             // apply the voltage to the drivetrain, based on direction of motion expected
//             left_voltage = (left_target_dist > 0) ? left_voltage : -left_voltage;
//             left_side_motors.move(left_voltage);
//             printf("%s(): left_side_motors  voltage: %d \n", __func__, left_voltage);
//         } 
//         else 
//         {
//             left_side_motors_done = true;
//             left_side_motors.brake();
//             printf("%s(): left_side_motors  Brake! - Error: %+3.3f \n", __func__, left_remaining_dist);
//         }


//         // -- Right Drivetrain processing --
//         // Read current positions
//         double right_pos = ((right_front_motor.get_position() + right_mid_motor.get_position() + right_back_motor.get_position()) / 3);
//         double right_dist_traveled = degrees2inches(left_pos);
//         /*double right_dist_traveled = rotR.get_position() * inches_per_tick;//*/
//         double right_remaining_dist = right_target_dist - right_dist_traveled;
//         printf("%s(): right_side_motors Covered: %+3.3f, Remaining: %+3.3f \n", 
//             __func__, right_dist_traveled, right_remaining_dist);
//         // move if target is not yet reached
//         if (std::abs(right_remaining_dist) > dist_margin)
//         {
//             right_side_motors_done = false;            
//             // use a voltage that is  proportional (P of PID) to distance remaining
//             // Note that the sign will reverse if the drivetrain overshoots its target distance
//             int right_voltage = round(max_voltage * (1 - right_dist_traveled/right_target_dist));
//             // note that the above calculation also takes care of overshoot. If overshot,
//             // the right_dist_traveled > right_target_dist, resulting in a -ve voltage calculation, which will 
//             // reverse the direction of rotation of the right drivetrain

//             // ensure the drivetrains get the minimum viable move voltage
//             if (std::abs(right_voltage) < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) 
//             {
//                 right_voltage = (std::signbit(right_voltage)) ? -DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL : DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL ;
//             }
//             // apply the voltage to the drivetrain, based on direction of motion expected
//             right_voltage = (right_target_dist > 0) ? right_voltage : -right_voltage;
//             right_side_motors.move(right_voltage);
//             printf("%s(): right_side_motors voltage: %d \n", __func__, right_voltage);
//         }
//         else 
//         {
//             right_side_motors_done = true;
//             right_side_motors.brake();
//             printf("%s(): right_side_motors: Brake! - Error: %+3.3f \n", __func__, right_remaining_dist);
//         }

//         // when both left and righr drivetrains have reacheded their respective targets, break
//         if (left_side_motors_done && right_side_motors_done)
//         {
//             printf("%s(): SUCCESS: Moved Left %.3f and Right %.3f inches in %d milliseconds and %d iterations\n", 
//                 __func__, left_target_dist, right_target_dist, execution_time, iter);
//             success = true;
//             break;
//         }
        
//         // check for a time out and abandon if exceeded
//         if ( (execution_time = pros::millis() - func_start_time) > time_out )
//         {
//             printf("%s(): FAILURE: - TIMEOUT of %d milliseconds exceeded! \n", __func__, time_out);
//             success = false;
//             break;
//         }

//         printf("%s(): waiting %d milliseconds\n", __func__, loop_duration);
//         pros::delay(loop_duration);

//         printf("%s(): iteration %d completed\n\n", __func__, iter);
//         iter++; // iteration counter

//     } // end while(1)
    
//     // stop the drivetrains
//     printf("%s(): Braked both drivetrains!\n", __func__);
//     left_side_motors.brake();
//     right_side_motors.brake();

//     return success;
    
// } // end of move_drivetrains()

// bool move_linear(float target_dist, int time_out, int max_voltage, float dist_margin)
// /**
//  * @brief Move the whole robot (i.e. both the left and right drivetrains together) in a straight line.
//  * This function is a wrapper around the move_drivetrains() and uses a single distance parameter 
//  * instead of two distinct ones for left and right drivetrains.
// */
// {    
//     return move_drivetrains(target_dist, target_dist, time_out, max_voltage, dist_margin);
// } // end of move_linear()


// /**Autonomous motion functions using the IMU sensor
//  * ------------------------------------------------*/

// /*---- Autonomous motion functions using the IMU sensor -----*/
// bool yaw_by(float target_yaw, int time_out, int max_voltage, float yaw_margin) 
// /**
//  * @brief Rotate the robot by a specified degrees around the z-axis. Uses the IMU (Inertial Measurement 
//  * Unit or "gyro") to measure rotation and uses proportional power to make the robot rotate to reach the 
//  * target yaw. Also, to prevent wheel slippage at start, it uses a ramp-up logic, instead of applying
//  * max_voltage right at the beginning.
//  *
//  * Sensors used:
//  *      - V5 IMU: https://kb.vex.com/hc/en-us/articles/360037382272-Using-the-V5-Inertial-Sensor
//  *
//  * Features:
//  *  -   [Optional] The max time that this function needs to be completed within can be specified
//  *  -   [Optional] The max speeed reached during the rotation can be specified
//  *  -   [Optional] The acceptable error margin can be specified to accommodate real life scenarios
//  *
//  * @param target_yaw Intended yaw of the robot, +ve for Clockwise, -ve for Aniti-clockwise 
//  *      Valid range: -359.99 to 359.99 degrees
//  * @param time_out The maximum time in milli seconds the function can take to complete.
//  *      It ruturns a failure if this time is exceeded. Default: 60 seconds (max auton time)
//  * @param max_voltage Maximum volage that can be applied to the drivetrains
//  *      Valid Range: DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL to 127.  Default: 127
//  * @param yaw_margin The error at which the target will deemed to have been reached. 
//  * 		Default: DRIVEBASE_ACCEPTABLE_HEADING_ERROR
//  * 
//  * @return bool for success reaching the target yaw (heading), or not
//  */
// {
// 	// start the time-out timer
//     int func_start_time = pros::millis();

//     printf("%s(%+3.2f, %d, %d, %3.2f) called\n", __func__, target_yaw,
//         time_out, max_voltage, yaw_margin);

//     // do a sanity check on the input arguments
//     if ((target_yaw < -359.99) || (target_yaw > 359.99) )
//     {
//         printf("%s(): FAILURE: Specified target_yaw (%+3.2f) is out of range. Valid range: "\
//             "-359.99 to +359.99\n", __func__, target_yaw);
//         return false;
//     }
//     if ((max_voltage < 0) || (max_voltage > 127))
//     {
//         printf("%s(): FAILURE: Specified max_voltage (%d) is out of range. Valid range: "\
//             "0 to 127\n", __func__, max_voltage);
//         return false;
//     }

//     // check if any work is really needed
//     if (std::abs(target_yaw) < yaw_margin) 
//     {
//         printf("%s(): SUCCESS: Specified target_yaw (%+3.2f) is lower than configured " \
//             "yaw_margin (%3.2f) - No Action taken \n", __func__, target_yaw, yaw_margin);
//         return true;
//     }

//     // Set brake mode to HOLD for both left and right drivetrains
//     left_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD); 
//     right_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

//     // tare the rotation of the IMU sensor (i.e. reset to 0), before starting
//     imu.tare_rotation();
//     // set the data rate of the IMU sensor to the fastest allowed 
//     imu.set_data_rate(5); // set to 5ms (lowest possible)

//     float ramp_time = 150;   // ramp up time in milliseconds before applying max_voltage
//     int loop_duration = 50;     // loop interval in milliseconds
//     int execution_time = 0; // total execution time spent in this function
//     int iter = 0;           // iteration counter
//     bool success = false;
//     while(true)
//     {      
//         // read in the current yaw value
//         double current_yaw = current_yaw = imu.get_rotation();  
//         /** @note do not use yaw values (.get_yaw()) as it has a range -180 to 180
//          *  whereas rotation (.get_rotation()) has a range -inf to +inf which is 
//          * appropriate when turning >= 180 degrees */
        
//         // Calculate how much more to rotate to achieve target yaw
//         double remaining_yaw = target_yaw - current_yaw;
//         printf("%s(): Target Yaw: %+03.2f | Current Yaw: [%+03.2f] -> "\
//             "Remaining Yaw: [%+03.2f] degrees\n", 
//             __func__, target_yaw, current_yaw, remaining_yaw);

//         // check if any movement is actually needed, or we are already within the acceptable margin
//         if (std::abs(remaining_yaw) <= yaw_margin)
//         {
//             success = true;
//             printf("%s(): SUCCESS: Rotated [%+3.2f] degrees (with margin: %+3.2f degrees) in %d "\
//                 "milliseconds within %d iterations \n",
//                 __func__, target_yaw, remaining_yaw, execution_time, iter);
//             break;
//         }

//         // use a voltage that is proportional (P of PID) to degrees of yaw remaining
//         int voltage = round(max_voltage * (1 - current_yaw/target_yaw));
//         // note that the above calculation also takes care of overshoot. If overshot,
//         // the current_yaw > tareget_yaw, resulting in a -ve voltage calculation, which will 
//         // reverse the direction of rotation of the robot
//         /*printf("%s(): remaining_yaw = %+3.2f, target_yaw = %+3.2f -> proportion = %+0.3f\n", 
//             __func__, remaining_yaw, target_yaw, (1 - current_yaw/target_yaw));
//         printf("%s(): voltage (prorportional): %d\n", __func__, voltage); //*/

//         /** ramp up to full voltage in the first few iterations:
//          * t0: full voltage, for the first iteration - to overcome inertia
//          * t1: 1 * (loop_duration / ramp_time) * voltage
//          * t2: 2 * (loop_duration / ramp_time) * voltage
//          * t3: 3 * (loop_duration / ramp_time) * voltage ...
//          * tn: full voltage, for n >= (loop_duration / ramp_time) */   
//         if ((iter > 0)  &&  (iter <= (ramp_time / loop_duration)))
//         {
//             voltage = round(iter * loop_duration / ramp_time * voltage);
//             /*printf("%s(): voltage ramp up: i[%d]: %d\n", __func__, iter, voltage); //*/
//         }

//         // ensure the drivetrains get the minimum viable move voltage
//         if (std::abs(voltage) < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) 
//         {
//             voltage = (std::signbit(voltage)) ? \
//                 -DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL : DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL;
//         }
//         /*printf("%s(): voltage (min adjusted):      %d\n", __func__, voltage); //*/
        
//         // adjust the direction of motion (+ve for clockwise, -ve for anticlockwise) based on 
//         // based on direction of rotation specified by the sign of the target_yaw specified
//         voltage = (target_yaw > 0) ? voltage : -voltage;
//         /*printf("%s(): voltage (rotation dir adjusted):  %d\n", __func__, voltage); //*/

//         // apply the voltage to the drivetrain
//         left_side_motors.move(voltage);
//         right_side_motors.move(-voltage);
//         printf("%s(): voltage applied: %d\n", __func__, voltage);

//         // calculate how long execution of this function has been ongoing
//         execution_time = pros::millis() - func_start_time;

//         // log data for a voltage plot - used typically in finetuning the function
//         printf("%s(): Plot Data - %d, %d, %f, %f, %d\n", __func__, 
//             iter, execution_time, current_yaw, remaining_yaw, voltage); //*/

//         // check for a time out. Abandon if time out is exceeded
//         if (execution_time > time_out )
//         {
//             printf("%s(): FAILURE: - TIMEOUT of %d milliseconds exceeded! \n", __func__, time_out);
//             success = false;
//             break;
//         }

//         printf("%s(): waiting %d milliseconds\n", __func__, loop_duration);
//         pros::delay(loop_duration);

//         printf("%s(): iteration %d completed\n\n", __func__, iter);
//         iter++; // iteration counter

//     } // end while(1)
    
//     // stop the drivetrains
//     left_side_motors.brake();
//     right_side_motors.brake();
//     printf("%s(): Braked both drivetrains!\n", __func__);

//     return success;

// } // end yaw_by()


// /*---- Autonomous motion functions using the GPS sensor -----*/
// bool yaw_to_heading(float target_heading, rotation_direction rot_dir, int time_out, int max_voltage, float yaw_margin) 
// /**
//  * @brief Rotate the robot to a specified heading. Uses the GPS (Game Positioning System) to 
//  * measure yaw (heading) and uses proportional power to make the robot rotate to reach the 
//  * target heading. While by default the shorter angular distance is chosen as the direction of
//  * rotation, it can be specified and overridden. Also, to prevent wheel slippage at start, it 
//  * uses a ramp-up logic, instead of applying max_voltage right at the beginning.
//  * @note It does not use the (x,y) coordinate system of the new fields (with QR like GPS codes). 
//  * Instead, it just uses the IMU function of the GPS sensor.
//  *
//  * Sensors used:
//  *      - V5 GPS: https://kb.vex.com/hc/en-us/articles/360061932711-Using-the-V5-GPS-Sensor 
//  *
//  * Features:
//  *  -   [Optional] The max time that this function needs to be completed within can be specified
//  *  -   [Optional] The max speeed reached during the rotation can be specified
//  *  -   [Optional] The acceptable error margin can be specified to accommodate real life scenarios
//  *
//  * @param target_heading Intended yaw of the robot, +ve for Clockwise, -ve for Aniti-clockwise 
//  *      Valid range: -180 to 180 degrees
//  * @param rot_dir rotation direction; See definition of ENUM e_rotation_direction. 
//  *      Default: E_ROTATE_EITHER - the shorter angular distance will be chosen
//  * @param time_out The maximum time in milli seconds the function can take to complete.
//  *      It ruturns a failure if this time is exceeded. Default: 60 seconds (max auton time)
//  * @param max_voltage Maximum volage that can be applied to the drivetrains
//  *      Valid Range: DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL to +127.  Default: +127
//  * @param yaw_margin The error at which the target will deemed to have been reached. 
//  * 		Default: DRIVEBASE_ACCEPTABLE_HEADING_ERROR
//  * 
//  * @return bool for success reaching the target yaw (heading), or not
//  */
// {
// 	// start the time-out timer
//     int func_start_time = pros::millis();

//     printf("%s(%+3.2f, %d, %d, %d, %3.2f) called\n", __func__, target_heading, rot_dir,
//         time_out, max_voltage, yaw_margin);

//     // do a sanity check on the input arguments
//     if ((target_heading < 0) || (target_heading > 359.9999) )
//     {
//         printf("%s(): FAILURE: Specified target_heading (%+3.2f) is out of range. Valid range: "\
//             "0 to 359.99\n", __func__, target_heading);
//         return false;
//     }

//     if ((max_voltage < 0) || (max_voltage > 127))
//     {
//         printf("%s(): FAILURE: Specified max_voltage (%d) is out of range. Valid range: "\
//             "0 to 127]\n", __func__, max_voltage);
//         return false;
//     }

//     // Set brake mode to HOLD for both left and right drivetrains
//     left_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD); 
//     right_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

//     // set the data rate to the fastest allowed 
//     gps.set_data_rate(5); // set to 5ms (lowest possible)    

//     float ramp_time = 150;   // ramp up time in milliseconds before applying max_voltage
//     int loop_duration = 50;     // loop interval in milliseconds
//     int execution_time = 0; // total execution time spent in this function
//     int iter = 0;           // iteration counter
//     bool success = false;
//     while(true)
//     {
//         // update the current heading value
//         double current_heading = gps.get_heading();    
//         // caculate the yaw needed to point the robot towards the target_heading
//         // NOTE: This respects the direction of rotation specified in function call.
//         double yaw_remaining = get_heading_diff(current_heading, target_heading, rot_dir);
//         if (yaw_remaining  == -999.999)
//         {
//             printf("%s(): FAILURE: Could not calculate heading difference\n", __func__);
//             success = false;
//             break;
//         }    
//         printf("%s(): Target Heading: %+03.2f | Current Heading: [%+03.2f] -> "\
//             "Remaining Yaw: [%+03.2f] degrees\n", 
//             __func__, target_heading, current_heading, yaw_remaining);

//         // on the first iteration, note down the target_yaw
//         double target_yaw = (iter == 0) ? yaw_remaining : target_yaw;

//         // check if any movement is actually needed, or we are already within the acceptable margin
//         if (std::abs(yaw_remaining) <= yaw_margin)
//         {
//             success = true;
//             printf("%s(): SUCCESS: Rotated to [%+3.2f] degree heading (with margin: %+3.2f "\
//                 "degrees) in %d milliseconds within %d iterations \n", __func__, 
//                 target_heading, yaw_remaining, execution_time, iter);
//             break;
//         }

//         // use a voltage that is proportional (P of PID) to degrees of yaw remaining
//         int voltage = round(max_voltage * (yaw_remaining / target_yaw));
//         // note that the above calculation also takes care of overshoot. If overshot,
//         // the yaw_remaining > tareget_yaw, resulting in a -ve voltage calculation, which will 
//         // reverse the direction of rotation of the robot
//         /*printf("%s(): yaw_remaining = %+3.2f, target_yaw = %+3.2f -> proportion = %+0.3f\n", 
//             __func__, yaw_remaining, target_yaw, (yaw_remaining / target_yaw));
//         printf("%s(): voltage (prorportional):     %d\n", __func__, voltage); //*/

//         /** ramp up to full voltage as follows:
//          * t0: full voltage, for the first iteration - to overcome inertia
//          * t1: 1 * (loop_duration / ramp_time) * voltage
//          * t2: 2 * (loop_duration / ramp_time) * voltage
//          * t3: 3 * (loop_duration / ramp_time) * voltage ...
//          * tn: full voltage, for n >= (loop_duration / ramp_time) */   
//         if ((iter > 0)  &&  (iter <= (ramp_time / loop_duration)))
//         {
//             voltage = round(iter * loop_duration / ramp_time * voltage);
//             /*printf("%s(): voltage ramp up: i[%d]: %d\n", __func__, iter, voltage); //*/
//         }

//         // ensure the drivetrains get the minimum viable move voltage
//         if (std::abs(voltage) < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) 
//         {
//             voltage = (std::signbit(voltage)) ? -DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL : DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL;
//         }
//         /*printf("%s(): voltage (min adjusted):      %d\n", __func__, voltage); //*/

//         // adjust the direction of motion (+ve for clockwise, -ve for anticlockwise) based on 
//         // based on direction of rotation specified by the sign of the target_yaw
//         voltage = (std::signbit(target_yaw)) ? -voltage : voltage;
//         /*printf("%s(): voltage (rot_dir adjusted):  %d\n", __func__, voltage); //*/

//         // apply the voltage to the drivetrain, to make it spin around the Z-axis
//         left_side_motors.move(voltage);
//         right_side_motors.move(-voltage);
//         printf("%s(): voltage applied: %d\n", __func__, voltage);

//         // calculate how long execution of this function has been ongoing
//         execution_time = pros::millis() - func_start_time;

//         // log data for a voltage plot - used typically in finetuning the function
//         printf("%s(): Plot Data - %d, %d, %f, %f, %d\n", __func__, 
//             iter, execution_time, current_heading, yaw_remaining, voltage); //*/
 
//         // check for a time out. Abandon if time out is exceeded
//         if (execution_time > time_out )
//         {
//             printf("%s(): FAILURE: - TIMEOUT of %d milliseconds exceeded! \n", __func__, time_out);
//             success = false;
//             break;
//         }

//         printf("%s(): waiting %d milliseconds\n", __func__, loop_duration);
//         pros::delay(loop_duration);

//         printf("%s(): iteration %d completed\n\n", __func__, iter);
//         iter++; // iteration counter

//     } // end while(1)

//     // stop the drivetrains
//     left_side_motors.brake();
//     right_side_motors.brake();
//     printf("%s(): Braked both drivetrains!\n", __func__);

//     return success;

// } // end yaw_to_heading()

// bool move_to_xy(float target_x, float target_y, bool move_backwards, int time_out, int max_voltage, float dist_margin)
// /**
//  * @brief Move the robot to the (x,y) GPS coordinates specified. Move backwards, if specified.
//  *  It is done in 2 parts. First the robot yaws to the direction of the target position. For this
//  *  it uses the standard yaw_by() function. Secondly, it takes a linear part to the destination.
//  * 
//  * Sensors used:
//  *      - V5 GPS Sensor: https://kb.vex.com/hc/en-us/articles/360061932711-Using-the-V5-GPS-Sensor
//  *      - Also see: https://kb.vex.com/hc/en-us/articles/360061375711-Identifying-Location-Details-Using-the-GPS-Sensor-in-the-VRC-Spin-Up-Playground-for-VEXcode-VR
//  * 
//  * Features:
//  *  -   [Optional] The max time that this function needs to be completed within can be specified
//  *  -   [Optional] The max speeed reached during the movement can be specified.
//  *  -   [Optional] The acceptable error margin can be specified to accommodate real life scenarios
//  * 
//  * @param target_x x-coordinate for the robot to move to, on the GPS coordinate system. 
//  *      Valid Range: -1800 to 1800 i.e. FIELD_X_AXIS_MIN to FIELD_X_AXIS_MAX
//  * @param target_y y-coordinate for the robot to move to, on the GPS coordinate system. 
//  *      Valid Range: -1800 to 1800 i.e. FIELD_Y_AXIS_MIN to FIELD_Y_AXIS_MAX
//  * @param move_backwards determines the forwards / backwards direction of motion of the robot
//  *      Default: false - i.e. move forwards by default
//  * @param time_out The maximum time in milli seconds the function can take to complete.
//  *      It ruturns a failure if this time is exceeded. Default: 60 seconds (max auton time)
//  * @param max_voltage Maximum volage that can be applied to the drivetrains
//  *      Valid Range: DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL to +127. Default: +127
//  * @param dist_margin The error at which the target will deemed to have been reached. 
//  * 			Default: DRIVEBASE_ACCEPTABLE_MOVE_ERROR_M
//  * @return bool - indicating if it reached the target distance or not
// */
// {   
//     // start the time-out timer
//     int func_start_time = pros::millis();
    
//     printf("%s(%f, %f, %d, %d, %f) called \n", 
//         __func__, target_x, target_y, time_out, max_voltage, dist_margin);

//     if (target_x > FIELD_X_AXIS_MAX || target_x < FIELD_X_AXIS_MIN ||
//         target_y > FIELD_Y_AXIS_MAX || target_y < FIELD_Y_AXIS_MIN)
//     {
//         printf("%s(): FAILURE: - Invalid coordinates (%.3f, %.3f) - Cannot proceed!\n", 
//             __func__, target_x, target_y);
//         return false;       
//     }

//     // get current coordinates of the center (i.e. center of rotation) of the robot
//     // The center of rotation is where the GPS unit's measurements are configured for
//     double current_center_x = gps.get_status().x;
//     double current_center_y = gps.get_status().y;
//     printf("%s(): Initial coordinates (%.3f, %.3f)\n", __func__, current_center_x, current_center_y);
//     // Get the current heading
//     double current_heading = gps.get_heading();
//     printf("%s(): Initial Heading is [%3.2f] degrees\n", __func__, current_heading);
    
//     // calculate the heading necessary for the robot to point directly at the target
//     double heading_needed = rad2deg(atan2((target_y - current_center_y), (target_x - current_center_x)));
//     // atan2 will return values between -180 and 180 degrees. Convert it to the range 0 to 360 degrees
//     heading_needed = (heading_needed < 0) ? heading_needed + 360 : heading_needed;
//     /*printf("%s(): Needed  Heading is [%3.2f] degrees (Cartesian, calculated)\n", __func__, heading_needed);//*/
//     // convert from the cartesian coordinates to GPS (Compass) coordinates
//     heading_needed = cartesian2compass(heading_needed);
//     printf("%s(): Needed  Heading is [%3.2f] degrees (GPS, calculated)\n", __func__, heading_needed);

//     // check if the robot is expected to move backwards, rather than the more common forwards direction.
//     // If move_backwards is true, then the heading needed must be updated to change by +/-180 degrees
//     if (move_backwards) 
//     {
//         heading_needed += ((heading_needed + 180) > 359.9999) ? -180 : 180;
//         printf("%s(): Needed  Heading updated to [%3.2f] degrees (GPS, calculated), since move_backwards is [%d]\n", 
//             __func__, heading_needed, move_backwards);
//     }

// // TODO: remove after testing success
// while(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == false) pros::delay(250);    
//     // yaw to point at the needed heading
//     yaw_to_heading(heading_needed);

//     // with the robot now pointing towards the heading_needed, plan to move the robot to the target coordinates

//     // calculate the TARGET coordinates of the (center of the) Left drivetrain
//     double target_left_x = target_x - (DRIVEBASE_LEFT_OFFSET_M * sin(heading_needed));
//     double target_left_y = target_y + (DRIVEBASE_LEFT_OFFSET_M * cos(heading_needed));
//     printf("%s(): Left  DT  TARGET coordinates (%.3f, %.3f)\n", __func__, target_left_x, target_left_y);
//     // calculate the TARGET coordinates of the (center of the) Right drivetrain
//     double target_right_x = target_x + (DRIVEBASE_RIGHT_OFFSET_M * sin(heading_needed));
//     double target_right_y = target_y - (DRIVEBASE_RIGHT_OFFSET_M * cos(heading_needed));
//     printf("%s(): Right DT  TARGET coordinates (%.3f, %.3f)\n", __func__, target_right_x, target_right_y);

//     // ensure the drivetrains get the minimum viable move voltage
//     if (max_voltage < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) 
//     {
//         max_voltage = DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL;
//         printf("%s(): Using DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL (%d) since specified max_voltage (%d) is lower\n", 
//             __func__, DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL, max_voltage);
//     }

//     // Set brake mode to HOLD for both Left and Right drivetrains
//     left_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD); 
//     right_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
   
    
//     double left_target_dist = 0, right_target_dist = 0;
//     bool left_side_motors_done = false, right_side_motors_done = false;

//     int loop_duration = 50;     // loop interval
//     int iter = 0;           // iteration counter
//     int execution_time = 0;
//     bool success = false;
//     while (true)
//     {

// // TODO: remove after testing success
// if (iter % 20 == 0) {
// left_side_motors.brake();  right_side_motors.brake();
// printf("Waiting for tester input\n");
// while(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == false) pros::delay(250);    
// }
//         // update current coordinates of (the center of rotation of) the robot
//         current_center_x = gps.get_status().x;
//         current_center_y = gps.get_status().y;
//         printf("%s(): Current coordinates (%.3f, %.3f)\n", __func__, current_center_x, current_center_y);
//         // update the current heading of the robot
//         current_heading = gps.get_heading();
//         printf("%s(): Current Heading is [%3.2f] degrees\n", __func__, current_heading);

//         // recalculate to update the heading_needed - This is used primarily to figure out if the robot 
//         // has overshot its target and will thus impact the direction of motion of the robot
//         heading_needed = rad2deg(atan2((target_y - current_center_y), (target_x - current_center_x)));
//         // atan2 will return values between -180 and 180 degrees. Convert it to the range 0 to 360 degrees
//         heading_needed = (heading_needed < 0) ? (heading_needed + 360) : heading_needed;
//         /*printf("%s(): Needed  Heading is [%3.2f] degrees (Cartesian, calculated)\n", __func__, heading_needed);//*/
//         // convert from the cartesian coordinates to GPS (Compass) coordinates
//         heading_needed = cartesian2compass(heading_needed);
//         printf("%s(): Needed  Heading is [%3.2f] degrees (GPS, calculated)\n", __func__, heading_needed);

//         // If the needed heading and current heading differ by a large margin (theoretically 180, but using
//         // a lower practical value - 160 - to account for physical errors), then chances are that the robot has
//         // overshot the target and need to reverse direction
//         bool overshot = (std::abs(heading_needed - current_heading) > 160) ? overshot = true : false;


//         // Left Drivetrain processing 
//         // --------------------------
//         // update the current coordinates of the (center of the) Left drivetrain
//         double current_left_x = current_center_x - (DRIVEBASE_LEFT_OFFSET_M * sin(current_heading));
//         double current_left_y = current_center_y + (DRIVEBASE_LEFT_OFFSET_M * cos(current_heading));;
//         // Calculate the remaining distance
//         double left_remaining_dist 
//             = std::sqrt(std::pow((target_left_x - current_left_x), 2) + std::pow((target_left_y - current_left_y), 2));
//         if (iter == 0) 
//         {
//             // on the first iteration, note down the target distance
//             left_target_dist = left_remaining_dist;
//         }        
//         printf("%s(): Left  DT  Covered: %+.3f, Remaining: %+.3f \n", 
//             __func__, (left_target_dist - left_remaining_dist), left_remaining_dist);
//         // move if target is not yet reached
//         if (left_remaining_dist > dist_margin)
//         {
//             left_side_motors_done = false; 
//             // use a voltage that is proportional (P of PID) to distance remaining
//             int left_voltage = round(max_voltage * (left_remaining_dist/left_target_dist));
//             // but ensure the drivetrain gets the minimum viable move voltage
//             left_voltage = (left_voltage < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) ? DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL : left_voltage;
            
//             // adjust the direction of motion (+ve for forward, -ve for backwards) based on two criteria
//             // #1 The direction of motion specified when calling this fucntion
//             left_voltage = (move_backwards) ? -left_voltage : left_voltage;
//             // #2 If robot has overshot its target and hence needs to reverse direction
//             left_voltage = (overshot) ? -left_voltage : left_voltage;

//             // apply the voltage to the drivetrain
//             left_side_motors.move(left_voltage);
//             printf("%s(): left_side_motors  voltage: %d \n", __func__, left_voltage);
//         } 
//         else 
//         {
//             left_side_motors_done = true;
//             left_side_motors.brake();
//             printf("%s(): Left  DT  Brake! - Error: %+.3f meters\n", __func__, left_remaining_dist);
//         }

//         // Right Drivetrain processing 
//         // --------------------------
//         // update the current coordinates of the (center of the) Right drivetrain
//         double current_right_x = current_center_x + (DRIVEBASE_RIGHT_OFFSET_M * sin(current_heading));
//         double current_right_y = current_center_y - (DRIVEBASE_RIGHT_OFFSET_M * cos(current_heading));;
//         // Calculate the remaining distance
//         double right_remaining_dist 
//             = std::sqrt(std::pow((target_right_x - current_right_x), 2) + std::pow((target_right_y - current_right_y), 2));
//         if (iter == 0) 
//         {
//             // on the first iteration, note down the target distance
//             right_target_dist = right_remaining_dist;
//         }
//         printf("%s(): Right DT Covered: %+.3f, Remaining: %+.3f \n", 
//             __func__, (right_target_dist - right_remaining_dist), right_remaining_dist);
//         // move if target is not yet reached
//         if (right_remaining_dist > dist_margin)
//         {
//             right_side_motors_done = false;            
//             // use a voltage that is  proportional (P of PID) to distance remaining
//             int right_voltage = round(max_voltage * (right_remaining_dist/right_target_dist));
//             // but ensure the drivetrain gets the minimum viable move voltage
//             right_voltage = (right_voltage < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) ? DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL : right_voltage;
            
//             // adjust the direction of motion (+ve for forward, -ve for backwards) based on two criteria
//             // #1 The direction of motion specified when calling this fucntion
//             right_voltage = (move_backwards) ? -right_voltage : right_voltage;
//             // #2 If robot has overshot its target and hence needs to reverse direction
//             right_voltage = (overshot) ? -right_voltage : right_voltage;
            
//             // apply the voltage to the drivetrain
//             right_side_motors.move(right_voltage);
//             printf("%s(): right_side_motors voltage: %d \n", __func__, right_voltage);
//         }
//         else 
//         {
//             right_side_motors_done = true;
//             right_side_motors.brake();
//             printf("%s(): right_side_motors: Brake! - Error: %+.3f meters\n", __func__, right_remaining_dist);
//         }

//         // when both left and right drivetrains have reacheded their respective targets, its done!
//         if (left_side_motors_done && right_side_motors_done)
//         {
//             printf("%s(): SUCCESS: Moved %s to (%.3f, %.3f) in %d milliseconds with %d iterations \n", 
//                 __func__, target_x, target_y, (move_backwards ? "backwards" : "forwards"), execution_time, iter);
//             success = true;
//             break;
//         }

//        // check for a time out and abandon function if exceeded
//         if ( (execution_time = pros::millis() - func_start_time) > time_out )
//         {
//             printf("%s(): FAILURE: - TIMEOUT of %d milliseconds exceeded! \n", __func__, time_out);
//             success = false;
//             break;
//         }

//         printf("%s(): waiting %d \n", __func__, loop_duration); 
//         pros::delay(loop_duration);

//         printf("%s(): iteration %d completed\n\n", __func__, iter);
//         iter++; // iteration counter

//     } //end of while loop
    
//     // stop the drivetrains
//     printf("%s():  Braked both drivetrains!\n", __func__);
//     left_side_motors.brake();
//     right_side_motors.brake();

//     return success;

// } //end move_to_xy()


// /**Autonomous motion functions using motor encoders
//  * ------------------------------------------------*/

// bool move_with_motor_encoder (float target_dist, int time_out, int max_voltage, float dist_margin)
// {
//     // start the time-out timer
//     int func_start_time = pros::millis();
    
//     printf("%s(%f, %d, %d, %f) called \n", 
//         __func__, target_dist, time_out, max_voltage, dist_margin);

//     // Configure the motors
//     // Set motor encoder units
//     left_side_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
//     right_side_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
//     // Set brake modes of motors
//     left_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
//     right_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
//     // Tare the position of the motor encoders
//     left_side_motors.tare_position();
//     right_side_motors.tare_position();
//     /*// Tare the position of the motors
//     left_mtrB.tare_position();

//     // Tare the possition of the motors
//     //left_side_motors.tare_position();
//     //right_side_motors.tare_position();
//     left_mtrB.tare_position();
//     right_mtrB.tare_position();
//     left_mtrF.tare_position();
//     right_mtrF.tare_position();//*/

//     int rampTime = 150;     // in milliseconds
//     int delayTime = 10;     // loop interval
//     int iter = 0;           // iteration counter
//     int execution_time = 0;
//     bool success = false;
//     while (true)
//     {   
//         // Get motor encoder location
//         double left_pos = (left_back_motor.get_position() + left_front_motor.get_position() + left_mid_motor.get_position())/3;
//         printf("%s(): left pos = (%.3f) \n", __func__, left_pos);

//         double right_pos = (right_back_motor.get_position() + right_front_motor.get_position() + right_mid_motor.get_position())/3;
//         printf("%s(): right pos = (%.3f) \n", __func__, right_pos);
        
//         double avrage_degree_traveled = ((left_pos + right_pos)/2);
//         printf("avrage_degree_traveled(%.3f) \n", __func__, avrage_degree_traveled);
//         double avrage_inches_traveled = degrees2inches(avrage_degree_traveled);

//         printf("%s(): Avrage_inches_traveled(%.3f) \n", __func__, avrage_inches_traveled);

//         double dist_remaning = target_dist - avrage_inches_traveled;

//         // check if any movement is actually needed, or we are already within the acceptable margin
//         if (std::abs(dist_remaning) <= dist_margin)
//         {
//             success = true;
//             printf("%s(): SUCCESS: Moved [%+3.2f] inches (with margin: %+3.2f "\
//                 "inches) in %d milliseconds within %d iterations \n", __func__, 
//                 target_dist, dist_remaning, execution_time, iter); //*/
//             break;
//         }

//         // use a voltage that is proportional (P of PID) to distance remaining
//         // Note that the sign will reverse if the drivetrain overshoots its target distance
//         int voltage = round(max_voltage * (1 - avrage_inches_traveled/target_dist));
//         // note that the above calculation also takes care of overshoot. If overshot,
//         // the avrage_inches_traveled > target_dist, resulting in a -ve voltage calculation, which will 
//         // reverse the direction of rotation of the left drivetrain

//         /** ramp up to full voltage as follows:
//          * t0: full voltage, for the first iteration - to overcome inertia
//          * t1: 1 * (delayTime / rampTime) * voltage
//          * t2: 2 * (delayTime / rampTime) * voltage
//          * t3: 3 * (delayTime / rampTime) * voltage ...
//          * tn: full voltage, for n >= (delayTime / rampTime) */   
//         /*if ((iter > 0)  &&  (iter <= (rampTime / delayTime)))
//         {
//             voltage = round(iter * (delayTime / rampTime));
//             printf("%s(): voltage ramp up: i[%d]: %d\n", __func__, iter, voltage); //
//         }//*/
        
//         // ensure the drivetrains get the minimum viable move voltage
//         if (std::abs(voltage) < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) 
//         {
//             voltage = (std::signbit(voltage)) ? -DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL : DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL;
//         }
//         // apply the voltage to the drivetrain, based on direction of motion expected
//         voltage = (target_dist > 0) ? voltage : -voltage;

//         // apply the voltage
//         left_side_motors.move_voltage(voltage);
//         right_side_motors.move_voltage(voltage);
//         printf("%s(): Voltage applied: %d \n", __func__, voltage);

//         // check for a time out. Abandon if time out is exceeded
//         if ((execution_time = pros::millis() - func_start_time) > time_out )
//         {
//             printf("%s(): FAILURE: - TIMEOUT of %d milliseconds exceeded! \n", __func__, time_out);
//             success = false;
//             break;
//         }

//         printf("%s(): waiting %d milliseconds\n", __func__, delayTime);
//         pros::delay(delayTime);

//         printf("%s(): iteration %d completed\n\n", __func__, iter);
//         iter++; // iteration counter

//     } // end while(1)

//     // stop the drivetrains
//     printf("%s(): Braked both drivetrains!\n", __func__);
//     left_side_motors.brake();
//     right_side_motors.brake();

//     return success;

// } // end move_motor_encoder

// bool move_with_motor_encoder_fast (float target_dist, int time_out, int max_voltage, float dist_margin)
// {
//     // start the time-out timer
//     int func_start_time = pros::millis();
    
//     printf("%s(%f, %d, %d, %f) called \n", 
//         __func__, target_dist, time_out, max_voltage, dist_margin);

//     // Configure the motors
//     // Set motor encoder units
//     left_side_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
//     right_side_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
//     // Set brake modes of motors
//     right_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
//     left_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

//     // Tare the possition of the motors
//     left_back_motor.tare_position();
//     right_back_motor.tare_position();
//     left_mid_motor.tare_position();
//     right_mid_motor.tare_position();
//     left_front_motor.tare_position();
//     right_front_motor.tare_position();//*/

//     int rampTime = 150;     // in milliseconds
//     int delayTime = 10;     // loop interval
//     int iter = 0;           // iteration counter
//     int execution_time = 0;
//     bool success = false;
//     while (true)
//     {   
//         /*if ((iter > 0)  &&  (iter <= (rampTime / delayTime)))
//         {
//             max_voltage = round(iter * (delayTime/rampTime));
//             printf("%s(): max_voltage ramp up: i[%d]: %d\n", __func__, iter, max_voltage); //
//         }
        
//         // ensure the drivetrains get the minimum viable move voltage
//         if (std::abs(max_voltage) < DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL) 
//         {
//             max_voltage = (std::signbit(max_voltage)) ? -DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL : DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL;
//         }//*/
        
//         // Get motor encoder location
//          double left_pos = (left_back_motor.get_position() + left_front_motor.get_position() + left_mid_motor.get_position())/3;
//         //printf("%s(): left pos = (%.3f) \n", __func__, left_pos);

//         double right_pos = (right_back_motor.get_position() + right_front_motor.get_position() + right_mid_motor.get_position())/3;
//         //printf("%s(): right pos = (%.3f) \n", __func__, right_pos);
        
//         double avrage_degree_traveled = ((left_pos + right_pos)/2);
//         double avrage_inches_traveled = degrees2inches(avrage_degree_traveled);
//         printf("Avrage_inches_traveled(%.3f) \n", __func__, avrage_inches_traveled);

//         double dist_remaning = target_dist - avrage_inches_traveled;

//         left_side_motors.move_voltage(max_voltage);
//         right_side_motors.move_voltage(max_voltage);
        
//         if (avrage_inches_traveled < target_dist + dist_margin && avrage_inches_traveled > target_dist - dist_margin)
//         {
//             left_side_motors = 0;
//             right_side_motors = 0;
//             break;
//         }
//         pros::delay (delayTime);

//     } // end while loop

//     return true;

// } // end move_motor_encoder_fast

// float get_x()
// {   double Y_OFFSET = -6.6;
//     return ((72-(distance_y.get()/25.4)) + Y_OFFSET);

// }
// float get_y()
// {   
//     double X_OFFSET = -7;
//     return ((72-(distance_x.get()/25.4)) + X_OFFSET);
// }

// double normalize_angle(double angle) {
//     double normalized_angle = atan2(sin(angle * M_PI / 180.0), cos(angle * M_PI / 180.0));
//     normalized_angle = normalized_angle * 180.0 / M_PI;
//     if (normalized_angle < 0) {
//         normalized_angle += 360.0;
//     }
//     master.print(0, 0, "", normalized_angle);
//     return normalized_angle;
// }

// void reset ()
// {
//     if(normalize_angle(chassis.getPose().theta) > -5 && normalize_angle(chassis.getPose().theta) < 5)
//     {
//         chassis.setPose(-1*(get_y()),-1*(get_x()),chassis.getPose().theta);
//     }
//     else if(normalize_angle(chassis.getPose().theta) > 85 && normalize_angle(chassis.getPose().theta) < 95)
//     {
//         chassis.setPose(-1*(get_x()),get_y(),chassis.getPose().theta);
//     }

//     else if(normalize_angle(chassis.getPose().theta) > 175 && normalize_angle(chassis.getPose().theta) < 185)
//     {
//         chassis.setPose(get_y(),get_x(),chassis.getPose().theta);   
//     }
    
//     else if(normalize_angle(chassis.getPose().theta) > 265 && normalize_angle(chassis.getPose().theta) < 275)
//     {
//         chassis.setPose(get_x(),-1*(get_y()),chassis.getPose().theta);
//     }
// }
// void reset1 ()
// {
//     reset();
// }
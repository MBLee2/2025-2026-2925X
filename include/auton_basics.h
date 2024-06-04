#ifndef _AUTON_BASICS_H_
#define _AUTON_BASICS_H_

#include "api.h"

// Constants for auton code, based on observations and 
// refinements based on testing results
#define DRIVEBASE_MIN_MOVE_VOLTAGE         3000    // range: 0 to 12000 (milli volts)
#define DRIVEBASE_MIN_MOVE_VOLTAGE_DIGITAL 25      // range: 0 to 127
#define DRIVEBASE_ACCEPTABLE_MOVE_ERROR_IN 0.25    // inches
#define DRIVEBASE_ACCEPTABLE_MOVE_ERROR_M  0.00635 // meters
#define DRIVEBASE_ACCEPTABLE_HEADING_ERROR 0.5     // degrees


// VRC competition fields measurement limits, needed by the GPS sensor for auton 
#define FIELD_X_AXIS_MAX  1.800    // +1.8 meters
#define FIELD_X_AXIS_MIN -1.800    // -1.8 meters
#define FIELD_Y_AXIS_MAX  1.800    // +1.8 meters
#define FIELD_Y_AXIS_MIN -1.800    // -1.8 meters




// ENUM to capture which sensors is in use
// Typically used in autonomous functions needing position and heading data
enum e_sensors_used {
    E_IMU_ONLY,
    E_GPS_ONLY,
    E_IMU_AND_GPS
};

// ENUM to direction of rotation
typedef enum e_rotation_direction {
    E_ROTATE_CLOCKWISE,
    E_ROTATE_ANTICLOCKWISE,
    E_ROTATE_EITHER
} rotation_direction;

// functions to move between the Cartesian Coordinate System (used in trignometry) 
// and the Compass Coordinates used in navigation by the GPS sensor system
double cartesian2compass (double cartesian_angle); // cartesian to compass coordinates, both in dergees
double compass2cartesian (double compass_angle);   // compass to cartesian coordinates, both in degrees
double degrees2inches (double degrees);   // compass to cartesian coordinates, both in degrees

double get_heading_diff (double initial_heading, double target_heading, 
    rotation_direction rot_dir = E_ROTATE_EITHER);

// drivebase motion functions needed for auton

// Autonomous motion functions using the IMU sensor
bool move_drivetrains(float left_target_dist, float right_target_dist, 
    int time_out = 60000, int max_voltage = 127, 
    float dist_margin = DRIVEBASE_ACCEPTABLE_MOVE_ERROR_IN);

bool move_linear(float target_dist,
    int time_out = 60000, int max_voltage = 127, 
    float dist_margin = DRIVEBASE_ACCEPTABLE_MOVE_ERROR_IN);

bool yaw_by(float target_yaw,
    int time_out = 60000, int max_voltage = 127, 
    float yaw_margin = DRIVEBASE_ACCEPTABLE_HEADING_ERROR);


// Autonomous motion functions using GPS sensor
bool yaw_to_heading(float target_heading, rotation_direction rot_dir = E_ROTATE_EITHER,
    int time_out = 60000, int max_voltage = 127, 
    float yaw_margin = DRIVEBASE_ACCEPTABLE_HEADING_ERROR);

bool move_to_xy (float target_x, float target_y, bool move_backwards = false, 
    int time_out = 60000, int max_voltage = 127, 
    float dist_margin = DRIVEBASE_ACCEPTABLE_MOVE_ERROR_M); 

bool move_with_motor_encoder (float target_dist,
    int time_out = 60000, int max_voltage = 12000, 
    float dist_margin = DRIVEBASE_ACCEPTABLE_MOVE_ERROR_M); 
float get_y();
float get_x();
void reset();
void reset1();
double normalize_angle(double angle);
/* bool move_with_motor_encoder (float target_dist,
    int time_out = 60000, int max_voltage = 12000, 
    float dist_margin = DRIVEBASE_ACCEPTABLE_MOVE_ERROR_IN);*/

#endif //_AUTON_BASICS_H_
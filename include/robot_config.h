#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

#include "api.h"
#include "lemlib/api.hpp"

#define PI 3.14159265
// useful conversions
#define in2m(inches)      (inches * 0.0254)        // inches to metres
#define m2in(metres)      (metres * 39.3700787402) // meters to inches
#define m2mm(metres)      (metres / 1000)          // meters to millimetres
#define mm2m(millimetres) (millimetres * 1000)     // millimetres to metres
#define deg2rad(degrees)  (degrees * PI/180)   // degress to radians
#define rad2deg(radians)  (radians * 180/PI)   // radians to degress

// drivebase attributes
#define DRIVEBASE_WHEEL_DIAMETER     3.25 // inches
#define DRIVEBASE_GEAR_SIZE_ON_MOTOR 36.0   // 36 tooth gear
#define DRIVEBASE_GEAR_SIZE_ON_WHEEL 60.0   // 60 tooth gear
#define DRIVEBASE_X_AXIS_ACCEPTED_ERROR 20 //deadzone for x axis
// drivebase dimensions
#define DRIVEBASE_LEFT_OFFSET_IN   6.25    // inches (distance between left   drivetrain and center of rotation)
#define DRIVEBASE_RIGHT_OFFSET_IN  6.25    // inches (distance between right  drivetrain and center of rotation)
#define DRIVEBASE_LEFT_OFFSET_M    0.15875 // meters (distance between left   drivetrain and center of rotation)
#define DRIVEBASE_RIGHT_OFFSET_M   0.15875 // meters (distance between right  drivetrain and center of rotation)



//controller
extern pros::Controller master;
//chassis
extern lemlib::Chassis chassis;
extern pros::Motor left_front_motor; 
extern pros::Motor left_mid_motor;
extern pros::Motor left_back_motor;
extern pros::Motor right_front_motor;
extern pros::Motor right_mid_motor;
extern pros::Motor right_back_motor;

//motors
extern pros::Motor intake_mtr; 
extern pros::Motor flywheel_mtr; 
extern pros::Motor_Group left_side_motors;              // Left drivetrain of drivebase
extern pros::Motor_Group right_side_motors;             // Right drivetrain of drivebase
//pistons
extern pros::ADIDigitalOut lift_pistons;
extern pros::ADIDigitalOut PTO_piston;
extern pros::ADIDigitalOut back_wing_piston;
extern pros::ADIDigitalOut right_piston;
extern pros::ADIDigitalOut left_piston;
//sensors
extern pros::Gps gps;
extern pros::IMU imu;
 



#endif //_ROBOT_CONFIG_H_   
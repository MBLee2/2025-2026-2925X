#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

#include "api.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

#define PI 3.14159265
// useful conversions
#define in2m(inches)      (inches * 0.0254)        // inches to metres
#define m2in(metres)      (metres * 39.3700787402) // meters to inches
#define m2mm(metres)      (metres / 1000)          // meters to millimetres
#define mm2m(millimetres) (millimetres * 1000)     // millimetres to metres
#define deg2rad(degrees)  (degrees * PI/180)   // degress to radians
#define rad2deg(radians)  (radians * 180/PI)   // radians to degress

// drivebase attributes
#define DRIVEBASE_WHEEL_DIAMETER     lemlib::Omniwheel::NEW_275 // inches
#define DRIVEBASE_GEAR_SIZE_ON_MOTOR 24.0   // 36 tooth gear
#define DRIVEBASE_GEAR_SIZE_ON_WHEEL 24.0   // 60 tooth gear
#define DRIVEBASE_GEAR_RATIO         1
#define LIFT_GEAR_RATIO              1/6.
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
extern lemlib::Chassis chassisWithGoal;
extern pros::Motor lf; 
extern pros::Motor lb;
extern pros::Motor rf; 
extern pros::Motor rb;
extern pros::MotorGroup left_side_motors;              // Left drivetrain of drivebase
extern pros::MotorGroup right_side_motors;             // Right drivetrain of drivebase

//Motors
extern pros::Motor intake;
extern pros::Motor scoring;
extern pros::Motor storage;
extern pros::Motor reload;


//extern pros::Motor lift;

//pistons


//Sensors
//extern pros::Optical intake_color;==
extern pros::GPS gps;
extern pros::IMU imu;
// extern pros::Vision vision_sensor;

#endif //_ROBOT_CONFIG_H_   
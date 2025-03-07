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
#define DRIVEBASE_WHEEL_DIAMETER     lemlib::Omniwheel::NEW_4 // inches
#define DRIVEBASE_GEAR_SIZE_ON_MOTOR 36.0   // 36 tooth gear
#define DRIVEBASE_GEAR_SIZE_ON_WHEEL 72.0   // 60 tooth gear
#define DRIVEBASE_GEAR_RATIO         DRIVEBASE_GEAR_SIZE_ON_MOTOR / DRIVEBASE_GEAR_SIZE_ON_WHEEL
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
extern pros::Motor lf; 
extern pros::Motor lm;
extern pros::Motor lb;
extern pros::Motor rf; 
extern pros::Motor rm;
extern pros::Motor rb;
extern pros::MotorGroup left_side_motors;              // Left drivetrain of drivebase
extern pros::MotorGroup right_side_motors;             // Right drivetrain of drivebase

//Motors
extern pros::Motor intake;

extern pros::Motor ladybrownL;
extern pros::Motor ladybrownR;
extern pros::MotorGroup ladybrown;

//extern pros::Motor lift;

//pistons
extern pros::adi::Pneumatics mogo_clamp;
extern pros::adi::Pneumatics left_sweeper;
extern pros::adi::Pneumatics right_sweeper;
extern pros::adi::Pneumatics intake_lift;



//Sensors
//extern pros::Optical intake_color;
extern pros::Optical intake_color;
extern pros::Distance intake_dist;
extern pros::Optical intake_color2;
extern pros::Distance LB_dist;

extern pros::Distance distance_rb;
extern pros::Distance distance_rf;
extern pros::Distance distance_lb;
extern pros::Distance distance_lf;
extern pros::Distance distance_bl;
extern pros::Distance distance_br;
extern pros::Distance distance_proxi;
extern pros::Distance distance_front;
extern pros::Distance distance_left;
extern pros::Distance distance_back;
extern pros::GPS gps;
extern pros::IMU imu;
extern pros::adi::Button limitSwitch;
extern pros::Rotation lift_rotation;
extern pros::Vision vision_sensor;

#endif //_ROBOT_CONFIG_H_   
#include "main.h"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "robot_config.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);


//  need to add all motors
pros::Motor left_front_motor(4, pros::E_MOTOR_GEARSET_06);  // port 13, reversed
pros::Motor left_back_motor(13, pros::E_MOTOR_GEARSET_06);  // port 13, reversed

pros::Motor right_front_motor(-8, pros::E_MOTOR_GEARSET_06); // port 18, not reversed
pros::Motor right_back_motor(-17, pros::E_MOTOR_GEARSET_06); // port 18, not reversed


// drivetrain motor groups
pros::MotorGroup left_side_motors({left_front_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_back_motor});

pros::Distance distance_left(9);
pros::Distance distance_right(7);
pros::Distance distance_back(19);

pros::GPS gps(12);
pros::IMU imu(2);
 
lemlib::Drivetrain drivetrain(
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    10.5, // track width
    4.0,// wheel diameter
    200, // wheel rpm
	8 //chase Power
);
// left tracking wheel encoder
// right tracking wheel encoder
pros::Rotation vertical_rot(13, true); // port 1, not reversed
pros::Rotation horizontal_rot(6, true); // port 1, not reversed

// back tracking wheel encoder  
 
// vertical tracking wheel
//lemlib::TrackingWheel vertical_tracking_wheel(&test, 3.75, 5.25,450); // 2.00" wheel diameter, -4.6" offset from tracking center 
// horizontal tracking wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rot, 2.00, 6.875); // 2.00" wheel diameter, 4.5" offset from tracking center

// odometry struct
lemlib::OdomSensors sensors(
    nullptr, //SKILLSa  
    nullptr, // vertical tracking wheel 2
    nullptr,//SKILLS
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu // inertial sensor
);  
 
// forward/backward PID
lemlib::ControllerSettings linearController
(   8, // proportional gain (kP)
    0, // integral gain (kI)
    56, // derivative gain (kD)
    0, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    5, // large error range, in inches
    1000, // large error range timeout, in milliseconds
    16 // maximum acceleration (slew)
);
// turning PID
lemlib::ControllerSettings angularController(
    4, // kP
    0, // integral gain (kI)
    44.5, // kD
    3, // anti windup
    1, // smallErrorRange
    50, // smallErrorTimeout
    3, // largeErrorRange
    300, // largeErrorTimeou
    12 // slew rate
);
 
// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);


// TODO: Figure out why it gives a segmentation fault if this declaration is moved from here to "dashboard.cpp"
    #include "dashboard.h"
    // define the auton menu buttons
    std::vector <dashboard_motor_display> dashboard_motor_display_items = 
    {
        dashboard_motor_display {  5, 135, "DB-LF", left_front_motor},
        dashboard_motor_display {  5, 190, "DB-RF", right_front_motor},
        //dashboard_motor_display {110, 135, "DB-LM", left_mid_motor},
        //dashboard_motor_display {110, 190, "DB-RM", right_mid_motor},
        //dashboard_motor_display {320, 135, "DB-L5.5", left_extra_motor},
        //dashboard_motor_display {320, 190, "DB-R5.5", right_extra_motor},
        dashboard_motor_display {215, 135, "DB-LB", left_back_motor},
        dashboard_motor_display {215, 190, "DB-RB",  right_back_motor}
        
    };


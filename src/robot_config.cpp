#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "robot_config.h"
#include "dashboard.h"

AutonColor selectedColor = AutonColor::RED; //Set default color of auton    
pros::Controller master (pros::E_CONTROLLER_MASTER);

//Need to add all motors
pros::Motor lf(-18, pros::v5::MotorGears::blue);  // port 18, reversed
pros::Motor lm(10, pros::v5::MotorGears::blue);  // port 10, forward
pros::Motor lb(-9, pros::v5::MotorGears::blue);  // port 9, reversed
pros::Motor rf(13, pros::v5::MotorGears::blue); // port 13, forward
pros::Motor rm(-1, pros::v5::MotorGears::blue); // port 1, reversed
pros::Motor rb(2, pros::v5::MotorGears::blue); // port 2, forward

// drivetrain motor groups  
pros::MotorGroup left_side_motors({-18, 10, -9}, pros::v5::MotorGears::blue);
pros::MotorGroup right_side_motors({16, -1, 2}, pros::v5::MotorGears::blue);

// intake motor group
pros::Motor intakeL(20, pros::v5::MotorGears::red);  // port 4, reversed
pros::Motor intakeR(11, pros::v5::MotorGears::red);

//Other Motor
//pros::Motor lift(-6, pros::v5::MotorGears::red);  // Robot v1, ignore

//Pistons
pros::adi::Pneumatics hood1('d', true);
pros::adi::Pneumatics hood2('h', true);

pros::adi::Pneumatics mogo_clamp('a', false);
//pros::adi::Pneumatics mogo_clamp2('d', false);

pros::adi::Pneumatics intake_lift('f', false); // need to add on robot
//pros::adi::Pneumatics mogo_rush('b', false);
//pros::adi::Pneumatics lastring('a', false);

pros::adi::Pneumatics redirect1('e', true);
//pros::adi::Pneumatics redirect2('h', true);

pros::adi::Pneumatics lift_helper1('b', false);
pros::adi::Pneumatics lift_helper2('c', false);


/* SENSORS */
pros::Distance intake_dist(2);
pros::Distance distance_lf(19);
pros::Distance distance_lb(20);
pros::Distance distance_rf(8);
pros::Distance distance_rb(18);
pros::Distance distance_bl(7);
pros::Distance distance_br(10);
pros::Distance distance_front(5);

pros::GPS gps(12);
pros::IMU imu(3);

pros::adi::Button limitSwitch('B');
 
lemlib::Drivetrain drivetrain(
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    12, // track width
    lemlib::Omniwheel::NEW_4,// wheel diameter
    300, // wheel rpm
	8 //chase Power
);
// left tracking wheel encoder
// right tracking wheel encoder
pros::Rotation vertical_rot(13); // port 1, not reversed
pros::Rotation horizontal_rot(18); // port 1, not reversed

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
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              59, // derivative gain (kD)
                                              0, // anti windup
                                              2, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              15  // maximum acceleration (slew)
);
// turning PID
lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              57, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);



// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(0, //joystick deadband out of 127
                                     10, //  minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(0, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);
    // define the auton menu buttons
    std::vector <dashboard_motor_display> dashboard_motor_display_items = 
    {
        dashboard_motor_display {  5, 135, "DB-LF", lf},
        dashboard_motor_display {  5, 190, "DB-RF", rf},
        dashboard_motor_display {110, 135, "DB-LM", lm},
        dashboard_motor_display {110, 190, "DB-RM", rm},
        dashboard_motor_display {215, 135, "DB-LB", lb},
        dashboard_motor_display {215, 190, "DB-RB",  rb},
        dashboard_motor_display {320, 135, "Intake L", intakeL},
        dashboard_motor_display {320, 190, "Intake R", intakeR}
        
        
    };


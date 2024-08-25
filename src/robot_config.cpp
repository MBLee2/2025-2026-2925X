#include "main.h"
#include "pros/abstract_motor.hpp"
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
pros::Motor lf(-11, pros::v5::MotorGears::blue);  // port 13, reversed
pros::Motor lm(12, pros::v5::MotorGears::blue);  // port 13, reversed
pros::Motor lb(-17, pros::v5::MotorGears::blue);  // port 13, reversed
pros::Motor rf(16, pros::v5::MotorGears::blue); // port 18, not reversed
pros::Motor rm(-14, pros::v5::MotorGears::blue); // port 18, not reversed
pros::Motor rb(15, pros::v5::MotorGears::blue); // port 18, not reversed

// drivetrain motor groups  
pros::MotorGroup left_side_motors({-11, 12, -17}, pros::v5::MotorGears::blue);
pros::MotorGroup right_side_motors({16, -14, 15}, pros::v5::MotorGears::blue);

// intake motor group
pros::Motor flex(4, pros::v5::MotorGears::green);  // port 13, reversed
pros::Motor hook(3, pros::v5::MotorGears::green);  // port 13, reversed
pros::MotorGroup intake({4, 3}, pros::v5::MotorGears::green);

//Other Motor
pros::Motor lift(6, pros::v5::MotorGears::red);  // port 13, reversed

//Pistons
pros::adi::Pneumatics hood1('a', false);
pros::adi::Pneumatics mogo_clamp('b', false); //DONE
pros::adi::Pneumatics intake_puncher('c',false);
pros::adi::Pneumatics hood2('d', false);

pros::adi::Pneumatics intake_lift('e', false);
pros::adi::Pneumatics mogo_rush('f', false);
pros::adi::Pneumatics climb('g', false);


/* SENSORS */
pros::Optical intake_color(2);
pros::Distance distance_left(9);
pros::Distance distance_right(7);
pros::Distance distance_back(19);

pros::GPS gps(12);
pros::IMU imu(2);
 
lemlib::Drivetrain drivetrain(
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    12, // track width
    4.0,// wheel diameter
    300, // wheel rpm
	8 //chase Power
);
// left tracking wheel encoder
// right tracking wheel encoder
pros::Rotation vertical_rot(13); // port 1, not reversed
pros::Rotation horizontal_rot(6); // port 1, not reversed

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
lemlib::ControllerSettings lateral_controller(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              45, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
// turning PID
lemlib::ControllerSettings angular_controller(12, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
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
        dashboard_motor_display {320, 135, "Flex", flex},
        dashboard_motor_display {320, 190, "Hook", hook}
        
        
    };


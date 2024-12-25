#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/vision.h"
#include "robot_config.h"
#include "dashboard.h"

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
pros::Motor intakeL(-20, pros::v5::MotorGears::red);  // port 4, reversed
pros::Motor intakeR(11, pros::v5::MotorGears::red);

//Other Motor
//pros::Motor lift(-6, pros::v5::MotorGears::red);  // Robot v1, ignore

//Pistons
pros::adi::Pneumatics hood1('d', true);
pros::adi::Pneumatics hood2('h', true);

pros::adi::Pneumatics mogo_clamp('e', false);
//pros::adi::Pneumatics mogo_clamp2('d', false);

pros::adi::Pneumatics intake_lift('b', false); // need to add on robot
pros::adi::Pneumatics mogo_rush('g', false);
//pros::adi::Pneumatics lastring('a', false);

pros::adi::Pneumatics redirect1('a', false);

pros::adi::Pneumatics lift_helper1('f', false);
pros::adi::Pneumatics lift_helper2('c',false );


/* SENSORS */
pros::Optical intake_color(6);
pros::Optical intake_color2(8);
pros::Distance distance_lf(12);
pros::Distance distance_lb(20);
pros::Distance distance_rf(8);
pros::Distance distance_rb(18);
pros::Distance distance_bl(21);
pros::Distance distance_br(10);

pros::Distance distance_front(16);
pros::Distance distance_left(15);

pros::GPS gps(3);
pros::IMU imu(19);

pros::adi::Button limitSwitch('B');
pros::Rotation lift_rotation(7); 

pros::Vision vision_sensor(5);
pros::vision_signature_s_t BLUE_SIG = {1, {1, 0, 0}, 3.000, -3335, -2565, -2950, 4167, 5765, 4966, 0, 0};
pros::vision_signature_s_t RED_SIG = {2, {1, 0, 0}, 3.000, 8667, 10051, 9358, -1583, -853, -1218, 0, 0};


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
pros::Rotation vertical_rot(14); // port 1, not reversed
pros::Rotation horizontal_rot(17); // port 1, not reversed

// back tracking wheel encoder  
 
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rot,lemlib::Omniwheel::NEW_275, 1.25); // 2.00" wheel diameter, 1.25" offset from tracking center 
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rot,lemlib::Omniwheel::NEW_275, 1.00); // 2.00" wheel diameter, 1.00" offset from tracking center

// odometry struct
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, //SKILLSa  
    nullptr, // vertical tracking wheel 2
    &horizontal_tracking_wheel,//SKILLS
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu // inertial sensor
);  
 
// forward/backward PID
lemlib::ControllerSettings lateral_controller(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              36.5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              00, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// turning PID
lemlib::ControllerSettings angular_controller(5 , // proportional gain (kP)
                                              0, // integral gain (kI)
                                              37, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              127 // maximum acceleration (slew)
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


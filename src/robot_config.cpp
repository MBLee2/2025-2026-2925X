#include "lemlib/chassis/trackingWheel.hpp"
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
#include <cstddef>

pros::Controller master (pros::E_CONTROLLER_MASTER);

//Need to add all motors
pros::Motor lf(15, pros::v5::MotorGears::blue);  // port 15, forward
pros::Motor lm(4, pros::v5::MotorGears::blue);  // port 4, forward
pros::Motor lb(-18, pros::v5::MotorGears::blue);  // port 9, reversed
pros::Motor rf(-9, pros::v5::MotorGears::blue); // port 13, forward
pros::Motor rm( -1, pros::v5::MotorGears::blue); // port 1, reversed
pros::Motor rb(10, pros::v5::MotorGears::blue); // port 2, forward

// drivetrain motor groups  
pros::MotorGroup left_side_motors({15, 4, -18}, pros::v5::MotorGears::blue);
pros::MotorGroup right_side_motors({-9, -1, 10}, pros::v5::MotorGears::blue);


// intake motor 
pros::Motor intake(-20, pros::v5::MotorGears::red);  // port 4, reversed

//lady brown group
pros::Motor ladybrownL(14, pros::v5::MotorGears::green);  // port 4, reversed
pros::Motor ladybrownR(-11, pros::v5::MotorGears::green);  // port 4, reversed
pros::MotorGroup ladybrown({14, -11}, pros::v5::MotorGears::green);


//Other Motor
//pros::Motor lift(-6, pros::v5::MotorGears::red);  // Robot v1, ignore

//Pistons
pros::adi::Pneumatics mogo_clamp('d', false);
pros::adi::Pneumatics left_sweeper('b', false);
pros::adi::Pneumatics right_sweeper('a', false);
pros::adi::Pneumatics intake_lift('g', false);
pros::adi::Pneumatics odom_lift('e', false);



/* SENSORS */

pros::Distance LB_dist(12);
pros::Distance distance_proxi(7);

pros::Optical intake_color(3);
pros::Distance intake_dist(16);
pros::Distance distance_back(2);
pros::Distance distance_left(17);
pros::Distance distance_right(5);

pros::Optical intake_color2(22); //deprecated
pros::Distance distance_front(22);
pros::Distance distance_lf(22);
pros::Distance distance_lb(22);
pros::Distance distance_rf(22);
pros::Distance distance_rb(22);
pros::Distance distance_bl(22);
pros::Distance distance_br(22);



pros::GPS gps(3);
pros::IMU imu(6);

pros::adi::Button LB_limit('f');
pros::Rotation lift_rotation(6); 

pros::Vision vision_sensor(19);
//pros::vision_signature_s_t BLUE_SIG = {1, {1, 0, 0}, 3.000, -3335, -2565, -2950, 4167, 5765, 4966, 0, 0};
//pros::vision_signature_s_t RED_SIG = {2, {1, 0, 0}, 3.000, 8667, 10051, 9358, -1583, -853, -1218, 0, 0};

pros::adi::AnalogIn lineRight({21, 'a'});
pros::adi::AnalogIn lineLeft({21, 'b'});


lemlib::Drivetrain drivetrain(
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    12, // track width
    lemlib::Omniwheel::NEW_275,// wheel diameter
    600, // wheel rpm
	8 //chase Power
);
// left tracking wheel encoder
// right tracking wheel encoder
pros::Rotation vertical_rot(12); // NOT ON BOT
pros::Rotation horizontal_rot(8); // port 1, not reversed

// back tracking wheel encoder  
 
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rot,lemlib::Omniwheel::NEW_275, 1.25); // 2.00" wheel diameter, 1.25" offset from tracking center 
// horizontal tracking wheel

/*
NOT SET UP YET
*/
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rot,lemlib::Omniwheel::NEW_275_HALF, -3.50); // 2.00" wheel diameter, 1.00" offset from tracking center

// odometry struct
lemlib::OdomSensors sensors(
    nullptr, //&vertical_tracking_wheel, //SKILLSa  
    nullptr, // vertical tracking wheel 2
    nullptr, //&horizontal_tracking_wheel,//SKILLS
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu // inertial sensor
);  
 
// forward/backward PID
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              0.25, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              300, // large error range timeout, in millisecond
                                              0 // maximum acceleration (slew)
);

// turning PID
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              18.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (sle
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
        dashboard_motor_display {320, 135, "Intake", intake},
        dashboard_motor_display {320, 190, "lbrown", ladybrownL}
        
        
    };


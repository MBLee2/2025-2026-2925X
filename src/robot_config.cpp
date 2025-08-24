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
// pros::Motor lf(-2, pros::v5::MotorGears::blue);  // port 15, forward
// pros::Motor lm(21, pros::v5::MotorGears::blue);
// pros::Motor lb(-12, pros::v5::MotorGears::blue);  // port 9, reversed

// pros::Motor rf(10, pros::v5::MotorGears::blue); // port 13, forward
// pros::Motor rm(21, pros::v5::MotorGears::blue);
// pros::Motor rb(20, pros::v5::MotorGears::blue); // port 2, forward

// // drivetrain motor groups  
// pros::MotorGroup left_side_motors({-2, -12}, pros::v5::MotorGears::blue);
// pros::MotorGroup right_side_motors({10, 20}, pros::v5::MotorGears::blue);

pros::Motor lf(11, pros::v5::MotorGears::blue);  // port 15, forward
pros::Motor lb(5, pros::v5::MotorGears::blue);  // port 9, reversed

pros::Motor rf(-7, pros::v5::MotorGears::blue); // port 13, forward
pros::Motor rb(-19, pros::v5::MotorGears::blue); // port 2, forward

// drivetrain motor groups  
pros::MotorGroup left_side_motors({11, -1, 5}, pros::v5::MotorGears::blue);
pros::MotorGroup right_side_motors({-7, 18, -19}, pros::v5::MotorGears::blue);


// intake motor 
pros::Motor intake(-21, pros::v5::MotorGears::red);  // port 4, reversed
// pros::Motor intake(9, pros::v5::MotorGears::blue);  // port 4, reversed
 pros::Motor scoring(-4, pros::v5::MotorGears::green);
 pros::Motor storage(-15, pros::v5::MotorGears::blue);
 pros::Motor reload(-17, pros::v5::MotorGears::green); //19

//Pistons NOT DONE


/* SENSORS */ // NOT DONE
pros::IMU imu(16); //DONE

//Color Sort


// pros::Vision vision_sensor(19);
//pros::vision_signature_s_t BLUE_SIG = {1, {1, 0, 0}, 3.000, -3335, -2565, -2950, 4167, 5765, 4966, 0, 0};
//pros::vision_signature_s_t RED_SIG = {2, {1, 0, 0}, 3.000, 8667, 10051, 9358, -1583, -853, -1218, 0, 0};


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
pros::Rotation horizontal_rot(13); // port 1, not reversed
// back tracking wheel encoder  
 
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rot,lemlib::Omniwheel::NEW_275, 1.25); // 2.00" wheel diameter, 1.25" offset from tracking center 
// horizontal tracking wheel

/*
NOT SET UP YET
*/
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rot,lemlib::Omniwheel::NEW_275_HALF, -3.750); // 2.00" wheel diameter, 1.00" offset from tracking center

// odometry struct
lemlib::OdomSensors sensors(
    nullptr, //&vertical_tracking_wheel, //SKILLSa  
    nullptr, // vertical tracking wheel 2
    &horizontal_tracking_wheel, //&horizontal_tracking_wheel,//SKILLS
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu // inertial sensor
);  
 
// forward/backward PID
lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP)
                                              0.15, // integral gain (kI)
                                              16, // derivative gain (kD)
                                              2, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in millisecond
                                              60 // maximum acceleration (slew)
);

// turning PID
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              17, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
); 

// forward/backward PID
lemlib::ControllerSettings lateral_controller_with_goal(6, // proportional gain (kP)
                                              0.25, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              2.5, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in millisecond
                                              60 // maximum acceleration (slew)
);

// turning PID
lemlib::ControllerSettings angular_controller_with_goal(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
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

lemlib::Chassis chassisWithGoal(drivetrain,
                        lateral_controller_with_goal,
                        angular_controller_with_goal,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

    // define the auton menu buttons
    std::vector <dashboard_motor_display> dashboard_motor_display_items = 
    {
        dashboard_motor_display {  5, 135, "DB-LF", lf},
        dashboard_motor_display {  5, 190, "DB-RF", rf},
        // dashboard_motor_display {110, 135, "DB-LM", lm},
        // dashboard_motor_display {110, 190, "DB-RM", rm},
        dashboard_motor_display {110, 135, "DB-LB", lb},
        dashboard_motor_display {110, 190, "DB-RB",  rb},
        dashboard_motor_display {215, 135, "Intake", intake}
        // dashboard_motor_display {320, 190, "lbrown", ladybrownL}
        
        
    };


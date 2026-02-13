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
pros::Motor lf(10, pros::v5::MotorGears::blue);  // port 15, forward
pros::Motor lm(-9, pros::v5::MotorGears::blue);
pros::Motor lb(-8, pros::v5::MotorGears::blue);  // port 9, reversed

pros::Motor rf(-20, pros::v5::MotorGears::blue); // port 13, forward
pros::Motor rm(19, pros::v5::MotorGears::blue);
pros::Motor rb(18, pros::v5::MotorGears::blue); // port 2, forward

// drivetrain motor groups  
pros::MotorGroup left_side_motors({10, -9, -8}, pros::v5::MotorGears::blue);
pros::MotorGroup right_side_motors({-20, 19, 18}, pros::v5::MotorGears::blue);


// intake motor 
pros::Motor intake(5, pros::v5::MotorGears::blue);  // port 4, reversed
pros::Motor intake2(-11, pros::v5::MotorGears::blue);
//pros::Motor scoringR(-4, pros::v5::MotorGears::green);
//pros::Motor scoringL(5, pros::v5::MotorGears::green);
//pros::Motor storage(-15, pros::v5::MotorGears::green);
//pros::Motor reload(-19, pros::v5::MotorGears::green);

//Pistons NOT DONE
pros::adi::Pneumatics loader('a', false);
pros::adi::Pneumatics goal_switch('g', false);
pros::adi::Pneumatics stopper('h', false);
pros::adi::Pneumatics descore('c', false);
//pros::adi::Pneumatics hood('d', false);

/* SENSORS */ // NOT DONE
pros::IMU imu(3); //DONE

//Color Sort
pros::Optical intake_color(13);
pros::Distance intake_dist(2);
pros::Distance intake_dist_low(1);

pros::Distance dist_r(15);
pros::Distance dist_b(14);
pros::Distance dist_l(4);

// pros::Vision vision_sensor(19);
//pros::vision_signature_s_t BLUE_SIG = {1, {1, 0, 0}, 3.000, -3335, -2565, -2950, 4167, 5765, 4966, 0, 0};
//pros::vision_signature_s_t RED_SIG = {2, {1, 0, 0}, 3.000, 8667, 10051, 9358, -1583, -853, -1218, 0, 0};


lemlib::Drivetrain drivetrain(
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    10.25, // track width
    2.5,// wheel diameter
    600, // wheel rpm
	8 //chase Power
);
// left tracking wheel encoder
// right tracking wheel encoder
pros::Rotation vertical_rot(-6); // NOT ON BOT
pros::Rotation horizontal_rot(16); // port 1, not reversed
// back tracking wheel encoder  
 
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rot,2, -1); // 2.00" wheel diameter, 1.25" offset from tracking center 
// horizontal tracking wheel

/*
NOT SET UP YET
*/
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rot,2, 4.25); // 2.00" wheel diameter, 1.00" offset from tracking center

// odometry struct 
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, //SKILLSa  
    nullptr,//&vertical_tracking_wheel, // vertical tracking wheel 2
    nullptr, //&horizontal_tracking_wheel,//SKILLS
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu // inertial sensor
);  
 
// forward/backward PID
lemlib::ControllerSettings lateral_controller(6,//6, // proportional gain (kP) 6.8
                                              0.0008,//0.0008, // integral gain (kI)
                                              35,//35,// derivative gain (kD) 30
                                              0, // anti windupx
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              300, // large error range timeout, in milliseconds
                                              90 // maximum acceleration (slew)
); 

// turning PID
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP) 4
                                              0.001812, // integral gain (kI)s 0.00235
                                              27.02, // derivative gain (kD) 34.021
                                              5.5, // anti windup
                                              2, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              4, // large error range, in inches
                                              200, // large error range timeout, in milliseconds
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
        dashboard_motor_display {320, 135, "Intake", intake},
        dashboard_motor_display {320, 190, "Intake2", intake2}
        
        
    };


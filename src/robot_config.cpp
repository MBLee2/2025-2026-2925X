#include "main.h"
#include "robot_config.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);


//  need to add all motors
pros::Motor left_front_motor(19, pros::E_MOTOR_GEARSET_06);  // port 13, reversed
pros::Motor left_extra_motor(14, pros::E_MOTOR_GEARSET_06);   // port 12, reversed
pros::Motor left_mid_motor(18, pros::E_MOTOR_GEARSET_06);   // port 12, reversed
pros::Motor left_back_motor(-8, pros::E_MOTOR_GEARSET_06); // port 11, reversed

pros::Motor right_front_motor(-12, pros::E_MOTOR_GEARSET_06); // port 18, not reversed
pros::Motor right_extra_motor(-13, pros::E_MOTOR_GEARSET_06);   // port 12, reversed
pros::Motor right_mid_motor(-15, pros::E_MOTOR_GEARSET_06);    // port 19, not reversed
pros::Motor right_back_motor(1, pros::E_MOTOR_GEARSET_06);  // port 20, not reversed


// other motors
pros::Motor intake_mtr(5, pros::E_MOTOR_GEARSET_06);
//pros::Motor intake_mtr1(3, pros::E_MOTOR_GEARSET_18, false);
//pros::Motor cata_mtr1(14, pros::E_MOTOR_GEARSET_36, false); //UNCOMENT THIS WHEN USING DURING MATCHES
pros::Motor cata_mtr1(10, pros::E_MOTOR_GEARSET_18, false); //UNCOMENT THIS WHEN USING DURING SKIILS
pros::Motor cata_mtr2(15, pros::E_MOTOR_GEARSET_18, true);  //UNCOMENT THIS WHEN USING DURING SKIILS

pros::MotorGroup cata_motors({cata_mtr1, cata_mtr2});


// drivetrain motor groups
pros::MotorGroup left_side_motors({left_front_motor, left_extra_motor, left_mid_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_mid_motor, right_extra_motor, right_back_motor});
pros::MotorGroup test({right_mid_motor});

//Pistons
pros::ADIDigitalOut lift_pistons ('F');
//wings
pros::ADIDigitalOut left_piston ('G');
pros::ADIDigitalOut right_piston ('E');


// inertial sensor
pros::Imu imu(17); // port 21
pros::Gps gps(0,0,0); // offsets in meters
pros::ADIDigitalIn limitSwitch('A');
pros::Distance distance_x(7);
pros::Distance distance_y(6);
pros::Distance distance_sensor(3);

 
lemlib::Drivetrain drivetrain(
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    10.5, // track width
    4.40,// wheel diameter
    450, // wheel rpm
	2 //chase Power
);
// left tracking wheel encoder
// right tracking wheel encoder
pros::Rotation vertical_rot(13, true); // port 1, not reversed
pros::Rotation horizontal_rot(6, true); // port 1, not reversed

// back tracking wheel encoder  
 
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&test, 3.75, 5.25,450); // 2.00" wheel diameter, -4.6" offset from tracking center 
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rot, 2.00, 6.875); // 2.00" wheel diameter, 4.5" offset from tracking center

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
        dashboard_motor_display {110, 135, "DB-LM", left_mid_motor},
        dashboard_motor_display {110, 190, "DB-RM", right_mid_motor},
        dashboard_motor_display {320, 135, "DB-L5.5", left_extra_motor},
        dashboard_motor_display {320, 190, "DB-R5.5", right_extra_motor},
        dashboard_motor_display {215, 135, "DB-LB", left_back_motor},
        dashboard_motor_display {215, 190, "DB-RB",  right_back_motor}
        
    };


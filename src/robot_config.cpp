#include "main.h"
#include "robot_config.h"

using namespace pros;

pros::Controller master (pros::E_CONTROLLER_MASTER);


//  need to add all motors
Motor left_front_motor(13,true);  // port 13, reversed
Motor left_mid_motor(12,true);   // port 12, reversed
Motor left_back_motor(11, true); // port 11, reversed
Motor right_front_motor(18, false); // port 18, not reversed
Motor right_mid_motor(19,false);    // port 19, not reversed
Motor right_back_motor(20, false);  // port 20, not reversed

// other motors
pros::Motor intake_mtr(15, pros::E_MOTOR_GEARSET_06, true);
//pros::Motor cata_mtr1(14, pros::E_MOTOR_GEARSET_36, false); //UNCOMENT THIS WHEN USING DURING MATCHES
pros::Motor cata_mtr1(14, pros::E_MOTOR_GEARSET_18, false); //UNCOMENT THIS WHEN USING DURING SKIILS
pros::Motor cata_mtr2(3, pros::E_MOTOR_GEARSET_18, true);  //UNCOMENT THIS WHEN USING DURING SKIILS

MotorGroup cata_motors({cata_mtr1, cata_mtr2});


// drivetrain motor groups
MotorGroup left_side_motors({left_front_motor, left_mid_motor, left_back_motor});
MotorGroup right_side_motors({right_front_motor, right_mid_motor, right_back_motor});

//Pistons
pros::ADIDigitalOut lift_pistons ('G');
pros::ADIDigitalOut PTO_piston ('F');
pros::ADIDigitalOut left_piston ('H');

//Unused but left for later if we use
pros::ADIDigitalOut back_wing_piston ('E');
pros::ADIDigitalOut right_piston ('C');


// inertial sensor
Imu inertial_sensor(2); // port 5
Gps gps(0,0,0); // offsets in meters
 
lemlib::Drivetrain drivetrain(
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    10.75, // track width
    lemlib::Omniwheel::NEW_275,// wheel diameter
    600, // wheel rpm
	8 //chase Power
);
// left tracking wheel encoder
pros::ADIEncoder left_enc('A', 'B', true); // ports A and B, reversed
// right tracking wheel encoder
pros::Rotation right_rot(1, false); // port 1, not reversed
// back tracking wheel encoder
pros::ADIEncoder back_enc('C', 'D', false); // ports C and D, not reversed
 
// left tracking wheel
lemlib::TrackingWheel left_tracking_wheel(&left_enc, 2.75, -4.6); // 2.75" wheel diameter, -4.6" offset from tracking center
// right tracking wheel
lemlib::TrackingWheel right_tracking_wheel(&right_rot, 2.75, 1.7); // 2.75" wheel diameter, 1.7" offset from tracking center
lemlib::TrackingWheel back_tracking_wheel(&back_enc, 2.75, 4.5); // 2.75" wheel diameter, 4.5" offset from tracking center
 
// odometry struct
lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &inertial_sensor // inertial sensor
);
 
// forward/backward PID
lemlib::ControllerSettings linearController(
    11, // kP
    0, // integral gain (kI)
    90.2, // kD
    3, // anti windup
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    40 // slew rate; 
);
// turning PID
lemlib::ControllerSettings angularController(
    7, // kP
    0, // integral gain (kI)
    63, // kD
    3, // anti windup
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeou
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
        dashboard_motor_display {  5, 190, "DB-LM", left_mid_motor},
        dashboard_motor_display {110, 135, "DB-LB", left_back_motor},
        dashboard_motor_display {110, 190, "DB-RF", right_front_motor},
        dashboard_motor_display {320, 135, "DB-RM", right_mid_motor},
        dashboard_motor_display {320, 190, "DB-RB", right_back_motor},
        dashboard_motor_display {215, 135, "CataM", cata_mtr1},
        dashboard_motor_display {215, 190, "Intk",  intake_mtr}
        
    };


#include "main.h"
#include "robot_config.h"

using namespace pros;
pros::Controller master (pros::E_CONTROLLER_MASTER);


//  need to add all motors
Motor left_front_motor(18,true);  // port 13, reversed
Motor left_mid_motor(20,true);   // port 12, reversed
Motor left_back_motor(19, true); // port 11, reversed
Motor right_front_motor(7, false); // port 18, not reversed
Motor right_mid_motor(8,false);    // port 19, not reversed
Motor right_back_motor(9, false);  // port 20, not reversed


// other motors
pros::Motor intake_mtr(11, pros::E_MOTOR_GEARSET_06, true);
//pros::Motor intake_mtr1(3, pros::E_MOTOR_GEARSET_18, false);
//pros::Motor cata_mtr1(14, pros::E_MOTOR_GEARSET_36, false); //UNCOMENT THIS WHEN USING DURING MATCHES
pros::Motor cata_mtr1(10, pros::E_MOTOR_GEARSET_18, false); //UNCOMENT THIS WHEN USING DURING SKIILS
pros::Motor cata_mtr2(15, pros::E_MOTOR_GEARSET_18, true);  //UNCOMENT THIS WHEN USING DURING SKIILS

MotorGroup cata_motors({cata_mtr1, cata_mtr2});


// drivetrain motor groups
MotorGroup left_side_motors({left_front_motor, left_mid_motor, left_back_motor});
MotorGroup right_side_motors({right_front_motor, right_mid_motor, right_back_motor});

//Pistons
pros::ADIDigitalOut lift_pistons ('F');
//wings
pros::ADIDigitalOut left_piston ('G');
pros::ADIDigitalOut right_piston ('E');


// inertial sensor
Imu imu(21); // port 21
Gps gps(0,0,0); // offsets in meters
pros::ADIDigitalIn limitSwitch('A');
pros::Distance distance_sensor(6);

 
lemlib::Drivetrain drivetrain(
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    10.75, // track width
    lemlib::Omniwheel::NEW_275,// wheel diameter
    600, // wheel rpm
	2 //chase Power
);
// left tracking wheel encoder
// right tracking wheel encoder
pros::Rotation vertical_rot(13, true); // port 1, not reversed
pros::Rotation horizontal_rot(6, true); // port 1, not reversed

// back tracking wheel encoder  
 
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rot, 2.00, -2.75); // 2.00" wheel diameter, -4.6" offset from tracking center 
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
lemlib::ControllerSettings linearController(
    17, // kP
    0, // integral gain (kI)
    141, // kD
    3, // anti windup
    1, // smallErrorRange
    50, // smallErrorTimeout
    3, // largeErrorRange
    300, // largeErrorTimeout
    8 // slew rate; 
);
// turning PID
lemlib::ControllerSettings angularController(
    7, // kP
    0, // integral gain (kI)
    52, // kD
    3, // anti windup
    1, // smallErrorRange
    50, // smallErrorTimeout
    3, // largeErrorRange
    300, // largeErrorTimeou
    10 // slew rate
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


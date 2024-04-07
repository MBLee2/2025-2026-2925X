#include "auton_basics.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "controls.h"
#include "lemlib/api.hpp"

ASSET(matchloadturn4ball_txt);
ASSET(touchbar_txt);

// Auton routine start positions
auton_routine null_routine  {    0,     0,   0,   "None - Invalid Routine",      nullptr                 };
auton_routine near_driver_qual { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_near_driver_qual};
auton_routine near_driver_qual2 { 1.000, -1.300, 190, "15S Auton - Near Driver # 2", &rushWP}; 
auton_routine near_driver_elim { 1.000, -1.300, 190, "15S Auton - Near Driver # 2",&DescoreRushElim};  // to be updated
auton_routine near_driver_elim2 { 1.000, -1.300, 190, "15S Auton - Near Driver # 2", &rushelim};  // to be updated



auton_routine far_from_driver_qual { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_far_driver_qual};
auton_routine far_from_driver_elim { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_far_driver_elim};
auton_routine far_from_driver_elim2  { -0.600, 0.600, 180, "extra_1", &safe_6_ball};


auton_routine skills_1 		{ -0.600, 0.600, 180, "60S Auton - Skills # 1", &auton_60s_skills_1};
auton_routine skills_2 		{ -0.600, 0.600, 180, "60S Auton - Skills # 2", &auton_60s_skills_2};



//FULLY DONE
void auton_15s_near_driver_qual()
{
	printf("%s(): Exiting\n", __func__);
    chassis.setPose(50.5,55.5,315);
    right_piston.set_value(true);
    pros::delay(300);
    chassis.moveToPose(36, 60, 270, 1300,{},false);
    right_piston.set_value(false);
    intake_mtr.move(127);
    pros::delay(500);
    intake_mtr.move(-127);
    chassis.moveToPoint(8,60,3000,{.maxSpeed=40});

    
    /*chassis.moveToPoint(0,60,3000,{.maxSpeed=40});
    // wait until either triball is in intake or the motion stops
    while(distance_sensor.get()>85 && chassis.isInMotion()) {
    pros::delay(20);
    }
    chassis.cancelMotion();//*/

}
void rushWP(){
	printf("%s(): Exiting\n", __func__);

	printf("%s(): Exiting\n", __func__);
    chassis.setPose(50.5,55.5,315);
    right_piston.set_value(true);
    pros::delay(300);
    chassis.moveToPose(36, 60, 270, 1300,{},false);
    right_piston.set_value(false);
    intake_mtr.move(127);
    pros::delay(500);
    intake_mtr.move(-127);
    pros::delay(500);
    intake_mtr.move(127);
    chassis.turnToPoint(24,6,1000);
    
    chassis.moveToPoint(27, 10, 1200);
    //chassis.moveToPoint(24, 12, 1000,{.forwards=false});
    chassis.turnToHeading(270,1000);
    left_piston.set_value(true);
    chassis.moveToPoint(8, 8, 1000);
    chassis.turnToHeading(0,1000);
    chassis.moveToPoint(4, 34, 1000,{},false);
    intake_mtr.move(-127);
    left_piston.set_value(false);
    pros::delay(300);
    chassis.turnToHeading(90,1000);
    chassis.moveToPoint(21, 42, 1000);
    chassis.turnToHeading(0,1000);
    intake_mtr.move(127);
    chassis.moveToPoint(22, 64, 1000);
    chassis.turnToHeading(270,1000);
    chassis.moveToPoint(14, 64, 1200);
    /*chassis.moveToPoint(36, 60, 1000);

    //chassis.moveToPoint(8,60,3000,{.maxSpeed=40});//*/
    

}
void auton_15s_far_driver_qual()
{

}

void DescoreRushElim()
{

}
void auton_15s_far_driver_elim(){
	printf("%s(): Exiting\n", __func__);

}
void rushelim(){
	printf("%s(): Exiting\n", __func__);

	
}
void safe_6_ball()
{
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake_mtr.move(127);
    pros::delay(700);
    chassis.setPose(-12.4,60,90);

    chassis.moveToPoint(-4,60,1000);
    chassis.moveToPose(-40,58,90,1000,{.forwards=false,.minSpeed = 98,.earlyExitRange = 2});
    chassis.turnToHeading(55,500,{.minSpeed = 60,.earlyExitRange = 4});
    intake_mtr.move(0);
    chassis.waitUntil(12);
    left_piston.set_value(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-52,51,1400,{.forwards=false,.minSpeed=50,.earlyExitRange=2});
    chassis.turnToHeading(0,1000,{.minSpeed=50,.earlyExitRange=2});
    chassis.waitUntilDone();
    left_piston.set_value(false);
    chassis.turnToHeading(45,500,{.minSpeed=50,.earlyExitRange=2});

    chassis.moveToPoint(-65,20,1000,{.forwards=false});
    intake_mtr.move(-20);

    //chassis.turnToPoint(-65,0,1000);
    chassis.moveToPoint(-62, 44, 600,{.minSpeed=30,.earlyExitRange=2});
    chassis.turnToPoint(-65, 20, 600,{.minSpeed=30,.earlyExitRange=2});
    chassis.moveToPoint(-65, 20, 600,{.minSpeed=30,.earlyExitRange=2});
    chassis.moveToPoint(-50, 44, 1000,{.forwards=false,.minSpeed=30,.earlyExitRange=2});
    chassis.turnToPoint(-24,23,1000,{.minSpeed=30,.earlyExitRange=2});
    chassis.moveToPoint(-24, 20, 1000,{.minSpeed=30,.earlyExitRange=2});
    chassis.turnToHeading(90,1000,{.minSpeed=30,.earlyExitRange=2});
    intake_mtr.move(127);
    chassis.moveToPoint(0, 20, 600,{.minSpeed=30,.earlyExitRange=2});
    chassis.moveToPoint(-22, 12,1000,{.forwards=false,.minSpeed=30,.earlyExitRange=2});//maybe not needed
    chassis.turnToPoint(-60,0,1000,{.minSpeed=30,.earlyExitRange=2},false);
    intake_mtr.move(-127);
    pros::delay(400);
    chassis.turnToPoint(0,0,1000,{.minSpeed=30,.earlyExitRange=2});
    intake_mtr.move(127);
    chassis.moveToPoint(-3,0,1000,{.minSpeed=30,.earlyExitRange=2});
    chassis.turnToHeading(90,500,{.minSpeed=30,.earlyExitRange=2});
    left_piston.set_value(true);
    right_piston.set_value(true);
    chassis.moveToPoint(-60, 4,1000,{.forwards=false,.minSpeed=30,.earlyExitRange=2});
    chassis.moveToPoint(-36, 4, 1000,{.minSpeed=30,.earlyExitRange=2});
    chassis.turnToPoint(-60,0,1000,{.minSpeed=30,.earlyExitRange=2});
    intake_mtr.move(-127);
    pros::delay(400);
    chassis.moveToPoint(-60, 4,1000,{.minSpeed=30,.earlyExitRange=2});
    chassis.moveToPoint(-36, 4, 1000,{.forwards=false,.minSpeed=30,.earlyExitRange=2});

    //*/
}	
// STILL WORKING ON IT

ASSET(skillsPathPart1_txt);
void auton_60s_skills_1()
{
	
} // end auton_60s_skills_1()

// Auton skills number 2 fully done
void auton_60s_skills_2()	
{	
	
}





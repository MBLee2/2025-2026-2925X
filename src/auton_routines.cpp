#include "auton_basics.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "robot_config.h"
#include "controls.h"
#include "lemlib/api.hpp"

ASSET(matchloadturn4ball_txt);
ASSET(touchbar_txt);

// Auton routine start positions
auton_routine null_routine  {    0,     0,   0,   "None - Invalid Routine",      nullptr                 };
auton_routine near_driver_qual { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_near_driver_qual};
auton_routine near_driver_qual2 { 1.000, -1.300, 190, "15S Auton - Near Driver # 2", &rushWP}; 
auton_routine near_driver_elim { 1.000, -1.300, 190, "15S Auton - Near Driver # 2",&auton_15s_near_driver_elim};  // to be updated
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

}
void rushWP(){
	printf("%s(): Exiting\n", __func__);

}
void auton_15s_far_driver_qual()
{

}

void auton_15s_near_driver_elim()
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
	//chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    //intake_mtr.move(127);
    pros::delay(700);
    //chassis.setPose(-12.4,60,90);

    /*chassis.moveToPoint(-4,60,1000);
    intake_mtr.move(127); 
    chassis.moveToPose(-32,60,90,1000,{.forwards=false,.minSpeed = 98,.earlyExitRange = 2});
    chassis.turnToHeading(55,500,{.minSpeed = 60,.earlyExitRange = 4});
    intake_mtr.move(0);
    chassis.waitUntil(15);
    intake_mtr.move(0);
    left_piston.set_value(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-42.4,52.5,1400,{.forwards=false,.maxSpeed=70,.minSpeed=30,.earlyExitRange=2});

    chassis.turnToHeading(0,1000,{.minSpeed=60,.earlyExitRange=2});
    chassis.moveToPoint(-56, 36, 1000,{.forwards=false});
    chassis.waitUntilDone();
    left_piston.set_value(false);
    //chassis.turnToPoint(-58,24,1000,{.forwards=false});
    /*chassis.moveToPoint(-60, 24, 600,{.forwards=false});
    //chassis.turnToPoint(-65,0,1000);
    chassis.moveToPoint(-60, 42, 1000);
    chassis.turnToPoint(-60, 24, 1000);
    chassis.moveToPoint(-60, 24, 1000);//*/
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





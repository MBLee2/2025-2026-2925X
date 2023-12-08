#include "auton_basics.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "robot_config.h"
#include "controls.h"
#include "lemlib/api.hpp"

ASSET(matchloadturn4ball_txt);


// Auton routine start positions
auton_routine null_routine  {    0,     0,   0,   "None - Invalid Routine",      nullptr                 };
auton_routine near_driver_qual { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_near_driver_qual};
auton_routine near_driver_elim { 1.000, -1.300, 190, "15S Auton - Near Driver # 2", &auton_15s_near_driver_elim};  // to be updated
auton_routine far_from_driver_qual { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_far_driver_qual};
auton_routine far_from_driver_elim { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_far_driver_elim};
auton_routine skills_1 		{ -0.600, 0.600, 180, "60S Auton - Skills # 1", &auton_60s_skills_1};
auton_routine skills_2 		{ -0.600, 0.600, 180, "60S Auton - Skills # 1", &auton_60s_skills_2};


//elim auton near finalized fully done 
void auton_15s_near_driver_qual()
{	
	printf("%s(): Exiting\n", __func__);
    
}

// STILL WORKING ON IT

void auton_15s_near_driver_elim()
{
	printf("%s(): Exiting\n", __func__);
	move_with_motor_encoder(12,2000);

}

//qual auton far finalized fully done 
void auton_15s_far_driver_qual()
{
	chassis.setPose(-10.5,60,90);
	intake_mtr.move(127);
	chassis.moveTo(2,60,90,500);
	//chassis.follow(matchloadturn4ball_txt,5000, 5,false,false);
	chassis.moveTo(-34,58,90,2000,false,false,0,0.6,80);
	chassis.waitUntilDist(-30);
	intake_mtr.move(0);
	chassis.turnTo(-59,38,1000,false,true);
	back_wing_piston.set_value(true);
	chassis.moveTo(-55,36,0,2000,true,false);
	chassis.waitUntilDist(17);
	back_wing_piston.set_value(false);//*/
	chassis.waitUntilDist(1000);	
	chassis.turnTo(-67,12,1000,false,true);
	chassis.arcade(-127, 0);
	pros::delay(400);
	chassis.arcade(70,0);
	pros::delay(200);
	chassis.turnTo(-62,0,1000,false,false);
	chassis.arcade(100, 0);
	pros::delay(500);
	chassis.arcade(0, 0);
	chassis.moveTo(43,48,120,1000);
	chassis.turnTo(0,12,1000);
	intake_mtr.move(127);
	chassis.moveTo(-3,48,120,1000);




	/*chassis.arcade(127, 0);
	pros::delay(400);
	chassis.arcade(-127, 0);
	pros::delay(400);
	chassis.arcade(127, 0);
	pros::delay(400);//*/

	printf("%s(): Exiting\n", __func__);
}

//elim auton finalized fully done
void auton_15s_far_driver_elim()
{	 
	printf("%s(): Exiting\n", __func__);
}

// Skills Auton number 1
void auton_60s_skills_1()
{	
	printf("%s(): Exiting\n", __func__);
    
} // end auton_60s_skills_1()

// Auton skills number 2 fully done
void auton_60s_skills_2()
{

	printf("%s(): Exiting\n", __func__);

}

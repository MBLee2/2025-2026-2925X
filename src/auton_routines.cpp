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
auton_routine near_driver_elim { 1.000, -1.300, 190, "15S Auton - Near Driver # 2", &auton_15s_near_driver_elim};  // to be updated
auton_routine far_from_driver_qual { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_far_driver_qual};
auton_routine far_from_driver_elim { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_far_driver_elim};
auton_routine skills_1 		{ -0.600, 0.600, 180, "60S Auton - Skills # 1", &auton_60s_skills_1};
auton_routine skills_2 		{ -0.600, 0.600, 180, "60S Auton - Skills # 1", &auton_60s_skills_2};


//FULLY DONE
void auton_15s_near_driver_qual()
{
	printf("%s(): Exiting\n", __func__);
	chassis.setPose(47,59,315);
	chassis.moveTo(60,26,0,1000,false,false,0.0,0.6,100);
	pros::delay(100);
	chassis.arcade(-127,0);
	pros::delay(300);
	chassis.arcade(80,0);
	pros::delay(100);
	chassis.arcade(0,0);

	chassis.turnTo(72,60,1000,false,true);
	back_wing_piston.set_value(true);
	chassis.moveTo(24,58,80,1500,true,false);
	chassis.waitUntilDist(15);
	
	chassis.waitUntilDist(1000);
	back_wing_piston.set_value(false);
	chassis.moveTo(7.8,60,90,1400,false,false);
	lift_pistons.set_value(true);
	
	

    
}

// STILL WORKING ON IT
void auton_15s_near_driver_elim()
{
	printf("%s(): Exiting\n", __func__);
	chassis.setPose(36,48,180);
	chassis.moveTo(36,6,180,1000);
	left_piston.set_value(true);
	chassis.turnTo(-12,5,1000);
	chassis.arcade(127,0);
	pros::delay(500);
	chassis.arcade(-50,0);
	pros::delay(100);
	chassis.arcade(0,0);
	pros::delay(100);


	
}

// STILL WORKING ON IT
void auton_15s_far_driver_qual()
{
	//chassis.calibrate(); // calibrate the chassis
	chassis.setPose(-2,60,90);
	intake_mtr.move(127);
	chassis.moveTo(0,60,90,500);
	//chassis.follow(matchloadturn4ball_txt,5000, 5,false,false)
	chassis.moveTo(-34,58,90,2000,true,false,0,0.6,80);
	chassis.waitUntilDist(10);
	chassis.waitUntilDist(1000);	
	chassis.turnTo(-59,38,1000,false,true);
	back_wing_piston.set_value(true);
	chassis.moveTo(-56,36,0,2000,true,false);
	chassis.waitUntilDist(17);
	back_wing_piston.set_value(false);
	chassis.waitUntilDist(1000);	
	chassis.turnTo(-64,12,1000,false,true);
	chassis.arcade(-127, 0);
	pros::delay(420);
	chassis.arcade(0, 0);
	chassis.turnTo(-62,0,1000,false,false);
	chassis.arcade(100, 0);
	pros::delay(500);
	chassis.arcade(-127, 0);
	pros::delay(370);
	chassis.arcade(0, 0);
	chassis.turnTo(0,24,1000);
	intake_mtr.move(127);
	chassis.moveTo(3,24,105,1000);
	chassis.arcade(-127, 0);
	pros::delay(200);
	chassis.turnTo(-72,-2,1000);
	chassis.arcade(-10, 0);
	chassis.arcade(0,0);	
	intake_mtr.move(-40);
	chassis.arcade(127,0);
	pros::delay(700);
	//chassis.follow(touchbar_txt,1500,8);
	chassis.arcade(-60,0);
	pros::delay(300);
	chassis.moveTo(-12,40,270,2000,false,false);
	back_wing_piston.set_value(true);
	pros::delay(1000);
	chassis.arcade(-50,0);
	pros::delay(200);
	chassis.arcade(0,0);//*/

	
	



	printf("%s(): Exiting\n", __func__);
}

// STILL WORKING ON IT
void auton_15s_far_driver_elim()
{	 
	chassis.setPose(-10.5,60,90);
	intake_mtr.move(127);
	chassis.moveTo(3,60,90,500,false,true,0.0,0.6,60);
	//chassis.follow(matchloadturn4ball_txt,5000, 5,false,false)
	chassis.moveTo(-34,58,90,2000,true,false,0,0.6,80);
	chassis.waitUntilDist(10);
	intake_mtr.move(40);
	chassis.waitUntilDist(1000);	
	chassis.turnTo(-59,38,1000,false,true);
	back_wing_piston.set_value(true);
	chassis.moveTo(-55,36,0,2000,true,false);
	chassis.waitUntilDist(17);
	back_wing_piston.set_value(false);
	chassis.waitUntilDist(1000);	
	chassis.turnTo(-65,12,1000,false,true);
	chassis.arcade(-127, 0);
	pros::delay(420);
	chassis.arcade(127, 0);
	pros::delay(200);
	chassis.arcade(0, 0);
	chassis.turnTo(-67,0,1000,false,false);
	intake_mtr.move(-127);
	chassis.arcade(127, 0);
	pros::delay(500);
	chassis.arcade(-127, 0);
	pros::delay(300);
	chassis.arcade(0, 0);
	intake_mtr.move(127);
	//chassis.turnTo(0,24,1000);
	chassis.moveTo(2,24,0,1500);
	chassis.turnTo(70,0,1000);	
	chassis.arcade(127, 0);
	pros::delay(200);
	//chassis.turnTo(60,0,1000);
	/*chassis.moveToPose(-64,20,180,1000);
	intake_mtr.move(0);
	chassis.arcade(127,0);
	pros::delay(500);
	chassis.arcade(-80,0);
	pros::delay(250);
	chassis.arcade(0,0);
	//chassis.follow(touchbar_txt,1500,8);
	intake_mtr.move(127);
	chassis.moveToPose(-12,0,270,1000);
	//chassis.turnTo(-72,0,1000);
	//chassis.arcade(127,0);
	//pros::delay(500);
	/*chassis.moveToPose(2,36,0,20000,true,false);
	//back_wing_piston.set_value(true);*/
	printf("%s(): Exiting\n", __func__);
}

// STILL WORKING ON IT
void auton_60s_skills_1()
{	
    int shooting_time = 30000;	
	chassis.setPose(-48,60,45);
	chassis.moveTo(-61,26,0,1000,false,false);
	pros::delay(100);
	flywheel_mtr = 105;
	chassis.arcade(-100,0);
	pros::delay(350);
	chassis.arcade(80,0);
	pros::delay(280);
	chassis.turnTo(50,10,500);
	back_wing_piston.set_value(true);
	lift_pistons.set_value(true);
	pros::delay(shooting_time);
	//pros::delay(1500);
	back_wing_piston.set_value(false);
	lift_pistons.set_value(false);
	chassis.moveTo(-28,58,80,1500,false,true);
	flywheel_mtr = 0;
	chassis.moveTo(34,56,90,3000);
	chassis.moveTo(58,32,180,1000);
	chassis.turnTo(60,0,1300,false,false);
	chassis.arcade(127,0);
	pros::delay(600);
	chassis.arcade(-50,0);
	pros::delay(300);
	chassis.turnTo(60,0,1300,false,true);
	chassis.arcade(-127,0);
	pros::delay(600);
	chassis.arcade(50,0);
	pros::delay(300);
	chassis.arcade(0,0);
	chassis.turnTo(0,28,1000);
	chassis.arcade(127,0);
	pros::delay(200);
    chassis.arcade(0,0);
	chassis.moveTo(12,12,180,1800);
	left_piston.set_value(true);
	chassis.moveTo(60,-4,90,1400);
	chassis.arcade(127,0);
	pros::delay(600);
	chassis.arcade(-80,0);
	pros::delay(400);
	chassis.arcade(0,0);
	chassis.arcade(127,0);
	pros::delay(600);
	chassis.arcade(-80,0);
	pros::delay(400);
	chassis.arcade(127,0);
	pros::delay(600);
	chassis.arcade(-80,0);
	pros::delay(400);
	chassis.arcade(-80,0);
	chassis.arcade(0,0);


	/*chassis.moveTo(12,5,180,1000,false);
	chassis.waitUntilDist(60);
	left_piston.set_value(true);
	pros::delay(200);
	/*
	chassis.moveTo(50,0,110,2000);
	chassis.arcade(127,0);
	left_piston.set_value(false);
	pros::delay(200);
	chassis.arcade(0,0);
	chassis.moveTo(12,-6,180,1000,false),false;
	left_piston.set_value(true);
	pros::delay(200);
	chassis.moveTo(50,-12,110,2000,false,false);
	chassis.arcade(127,0);
	pros::delay(500);
	chassis.arcade(-50,0);
	pros::delay(500);
	chassis.arcade(0,0);//*/


	printf("%s(): Exiting\n", __func__);
    
} // end auton_60s_skills_1()

// Auton skills number 2 fully done
void auton_60s_skills_2()
{
	chassis.setPose(-48,60,45);
	chassis.setPose(-48,60,90);
	/*int shooting_time = 600000;	
	chassis.moveTo(-61,26,0,1000,false,false);
	pros::delay(100);
	flywheel_mtr = 100;
	chassis.arcade(-100,0);
	pros::delay(350);
	chassis.arcade(80,0);
	pros::delay(300);
	chassis.turnTo(50,10,500);
	back_wing_piston.set_value(true);
	//pros::delay(shooting_time);
	pros::delay(1500); 
	back_wing_piston.set_value(false);
	chassis.arcade(127,0);
	pros::delay(50);
	chassis.arcade(0,0);
	chassis.moveTo(-48,56,270,1000);
	chassis.moveTo(26,56,270,4000,false,false,0,0.6,90);
	chassis.arcade(0,0);
	chassis.turnTo(60,30,100);
	chassis.moveTo(60,30,315,1000,false,false);//*/
	


		
	
	printf("%s(): Exiting\n", __func__);

}

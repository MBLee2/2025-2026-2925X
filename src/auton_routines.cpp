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
	chassis.arcade(-40,0);
	pros::delay(200);
	chassis.arcade(0,0);
	back_wing_piston.set_value(true);
	pros::delay(300);
	chassis.arcade(50,-45);
	pros::delay(420);
	chassis.arcade(0,0);
	back_wing_piston.set_value(false);
	pros::delay(300);
	chassis.moveTo(60,26,0,1000,false,false,0.0,0.6,100);
	pros::delay(100);
	chassis.arcade(-127,0);
	pros::delay(300);
	chassis.arcade(80,0);
	pros::delay(100);
	chassis.arcade(0,0);
	intake_mtr.move(80);
	//chassis.turnTo(48,60,1000,false,false);
	chassis.moveTo(42,57,315,1300);
	chassis.turnTo(0,58,1000);
	chassis.moveTo(8,58,270,1500);
	intake_mtr.move(-127);
	

	/*chassis.moveTo(24,58,80,1500,true,true);
	chassis.waitUntilDist(15);
	
	chassis.waitUntilDist(1000);
	back_wing_piston.set_value(false);
	chassis.moveTo(7.8,60,90,1400,false,false);
	lift_pistons.set_value(true);*/
	
	

    
}

// STILL WORKING ON IT
void auton_15s_near_driver_elim()
{
	printf("%s(): Exiting\n", __func__);
	chassis.setPose(36,54,180);
	intake_mtr.move(80);
	chassis.moveTo(36,11,180,1000);
	left_piston.set_value(true);
	chassis.turnTo(-12,11,1000);
	chassis.arcade(127,0);
	pros::delay(500);
	chassis.arcade(-50,0);
	pros::delay(100);
	chassis.arcade(0,0);
	pros::delay(100);
	left_piston.set_value(false);
	chassis.turnTo(60,11,1000);
	chassis.moveTo(39,11,90,700);
	intake_mtr.move(-80);	
	pros::delay(500);
	chassis.arcade(-80,0);
	pros::delay(200);
	chassis.arcade(0,0);

	chassis.turnTo(-12,11,1000);
	chassis.arcade(-80,0);
	pros::delay(500);
	chassis.arcade(0,0);
	pros::delay(200);
	chassis.moveTo(24,11,270,1000);
	chassis.turnTo(40,58,1000);
	chassis.moveTo(39,53,0,1300);
	chassis.turnTo(0,53,1000);
	intake_mtr.move(127);
	chassis.arcade(0,0);
	chassis.moveTo(6,53,270,1000);
	intake_mtr.move(-127);



	
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
    int shooting_time = 31000;	
	chassis.setPose(-48,58,45);
	chassis.moveTo(-61,26,0,1000,false,false);
	pros::delay(100);
	chassis.arcade(-100,0);
	pros::delay(350);
	chassis.arcade(80,0);
	pros::delay(280);
	chassis.turnTo(50,4,500);
	cata_motors = 82.55;
	//pros::delay(shooting_time);
	pros::delay(1500);
	cata_motors = 0;
	chassis.moveTo(-28,58,90,1500);
	chassis.turnTo(24,60,500);
	chassis.arcade(100,0);
	pros::delay(1000);
	chassis.arcade(0,0);
	chassis.moveTo(34,56,90,1000);
	chassis.moveTo(60,32,180,1000);
	chassis.turnTo(56,0,1300,false,false);
	chassis.arcade(80,0);
	pros::delay(600);
	chassis.arcade(-50,0);
	pros::delay(300);
	chassis.turnTo(60,0,1300,false,true);
	chassis.arcade(-80,0);
	pros::delay(600);
	chassis.arcade(45,0);
	pros::delay(300);
	chassis.arcade(0,0);
	chassis.moveTo(40,46,315,600);
	chassis.turnTo(12,12,600);
	chassis.arcade(100,0);
	pros::delay(600);
	chassis.arcade(0,0);

	//chassis.moveTo(10,10,225,700);
	//chassis.moveTo(12,12,180,1000,false,true,0,0.35,100);
	left_piston.set_value(true);
	chassis.moveTo(60,4,100,1400);
	chassis.arcade(127,0);
	pros::delay(600);
	chassis.arcade(-80,0);
	left_piston.set_value(false);
	/*chassis.turnTo(12,0,1000,false,false,76.2);
	chassis.moveTo(16,0,180,1000);
	chassis.turnTo(60,0,1000);
	/*chassis.arcade(0,0);
	left_piston.set_value(true);
	chassis.arcade(127,0);
	pros::delay(600);
	chassis.arcade(-80,0);
	left_piston.set_value(false);//*/
	/*
	chassis.moveTo(10,-12,)
	pros::delay(400);
	chassis.arcade(127,0);
	pros::delay(600);
	chassis.arcade(-80,0);
	pros::delay(400);
	chassis.arcade(-80,0);
	chassis.arcade(0,0); //*/


	printf("%s(): Exiting\n", __func__);
    
} // end auton_60s_skills_1()

// Auton skills number 2 fully done
void auton_60s_skills_2()
{	
	printf("%s(): Exiting\n", __func__);
	int shooting_time = 1000;	
	int volts = 7800;

	chassis.setPose(-48,54,0);
	chassis.moveTo(-63,27,0,1000,false,false,0,0.6,127);
	chassis.turnTo(-63,0,700,false,true);
	chassis.arcade(-70,0);
	pros::delay(500);	
	chassis.arcade(80,0);
	pros::delay(280);
	chassis.arcade(0,0);
	chassis.moveTo(58,45,90,500);
	chassis.turnTo(50,14,500);
	chassis.arcade(-30,0);
	back_wing_piston.set_value(true);
	pros::delay(250);
	chassis.arcade(0,0);
	cata_motors.move_voltage(volts);
	pros::delay(shooting_time);
	//pros::delay(400);
	cata_motors = 0;
	chassis.arcade(30,0);
	pros::delay(250);
	chassis.arcade(0,0);
	back_wing_piston.set_value(false);
	chassis.turnTo(-36,63,800);
	chassis.moveTo(-36,64,90,1500);
	chassis.moveTo(35,63,90,5000);
	chassis.turnTo(69,27,700,false,true);
	chassis.arcade(-90,0);
	pros::delay(475);   
	chassis.arcade(0,0);
	chassis.turnTo(56,0,1200,false,true);
	pros::delay(100);
	chassis.arcade(-127,0);
	pros::delay(400);
	chassis.arcade(50,0);
	pros::delay(330);
	chassis.arcade(0,0);
	pros::delay(300);
	chassis.arcade(-127,0);
	pros::delay(400);
	chassis.arcade(50,0);
	pros::delay(300);
	chassis.arcade(0,0);
	chassis.setPose(65.5,39.5,0);

	/*chassis.turnTo(0,38,1500);
	chassis.arcade(80,0);
	pros::delay(900);
	chassis.arcade(0,0);//*/

	chassis.moveTo(2,14,180,1000);
	/*chassis.turnTo(30,12,700);
	left_piston.set_value(true);
	pros::delay(500);
	chassis.moveTo(28,12,90,1000);	
	chassis.arcade(120,0);
	pros::delay(300);
	chassis.arcade(-60,0);
	pros::delay(300);
	chassis.arcade(0,0);
	left_piston.set_value(false);
	/*
	chassis.moveTo(3,14,180,1400,false,true);
	chassis.turnTo(3,-4,700);
	chassis.moveTo(3,-5, 180,1000);
	chassis.turnTo(30,-5,700);
	left_piston.set_value(true);
	pros::delay(300);
	chassis.moveTo(28,-5,90,1200);
	chassis.arcade(120,0);
	pros::delay(200);
	chassis.arcade(-60,0);
	pros::delay(300);
	chassis.arcade(0,0);
	chassis.moveTo(3,-5,90,1400,false,false);
	left_piston.set_value(false);
	pros::delay(300);

	chassis.moveTo(5,9,90,1500);
	left_piston.set_value(true);
	pros::delay(300);
	chassis.arcade(120,0);
	pros::delay(700);
	chassis.arcade(-60,0);
	pros::delay(300);
	chassis.arcade(0,0);
	left_piston.set_value(false);
	chassis.turnTo(12,-48,800);
	chassis.moveTo(48,-48,135,2000);
	chassis.turnTo(60,-26,900,false,true);
	//*/
}

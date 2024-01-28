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
auton_routine near_driver_elim { 1.000, -1.300, 190, "15S Auton - Near Driver # 2", &auton_15s_near_driver_elim};  // to be updated


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
	chassis.moveToPoint(53,53,1000,false,127,false);
	left_piston.set_value(true);
	pros::delay(500);
	chassis.turnTo(0,60,900);
	chassis.arcade(50,0);
	pros::delay(100);
	chassis.arcade(0,0);
	pros::delay(500);
	left_piston.set_value(false);
	chassis.turnTo(0,60,1000);
	intake_mtr.move(127);

	chassis.moveToPose(62,27,0,1000,{.forwards=false,.minSpeed=90});
	chassis.turnTo(62,0,700,false);
	chassis.arcade(-100,0);
	pros::delay(200);
	chassis.arcade(80,0);
	pros::delay(200);
	chassis.moveToPoint(36,58,2000);
	chassis.turnTo(0,58,1000);
	chassis.moveToPose(9.25,58,270,1500,{},false);//*/
	intake_mtr.move(127);
	pros::delay(700);
	intake_mtr.move(-127);

	
}

void rushWP(){
	printf("%s(): Exiting\n", __func__);
	chassis.setPose(50.5,55.5,315);
	chassis.moveToPoint(53,53,1000,false,127,false);
	left_piston.set_value(true);
	pros::delay(150);
	chassis.turnTo(0,60,900,true,127,false);
	left_piston.set_value(false);
	chassis.moveToPoint(0,60,120);
	pros::delay(300);
	intake_mtr.move(127);
	chassis.turnTo(32,0,800);
	chassis.moveToPoint(30,3,1600);
	chassis.moveToPoint(46,62,1600,false,127);
	pros::delay(200);
	chassis.turnTo(0,60,700,true,127,false);
	intake_mtr.move(-127);
	pros::delay(500);
	chassis.moveToPose(69,27,0,1000,{.forwards=false});
	chassis.moveToPoint(64,22,1000,false);
	chassis.moveToPoint(36,56,1000);
	chassis.turnTo(0,56,700);
	left_piston.set_value(true);
	chassis.moveToPoint(14,56,1000,true);
	/*chassis.moveToPoint(36,36,1000);
	/*chassis.turnTo(0,60,1000);
	chassis.moveToPose(9.25,60,270,1500,{},false);//*/
}

// STILL WORKING ON IT
void auton_15s_near_driver_elim()
{
	printf("%s(): Exiting\n", __func__);
	chassis.setPose(35.5,50.25,180);
	chassis.moveToPoint(24,3,1500);
	chassis.waitUntil(0.5);
	left_piston.set_value(true);
	intake_mtr.move(127);
	chassis.waitUntil(8);
	left_piston.set_value(false);
	chassis.waitUntilDone();
	chassis.moveToPoint(24,6,1500,false);
	chassis.turnTo(-12,6,900);
	left_piston.set_value(true);
	intake_mtr.move(-127);
	chassis.moveToPoint(-6,6,900,true,127,false);
	chassis.moveToPoint(12,6,1000,false);
	left_piston.set_value(false);
	chassis.moveToPoint(38,38,1000,false);
	chassis.turnTo(60,40,1000,false);
	chassis.moveToPoint(52,40,1500,false);
	chassis.turnTo(57,0,1000);
	chassis.moveToPoint(60,0,1000,false);
	/*chassis.turnTo(36,58,1000);
	left_piston.set_value(true);
	chassis.moveToPose(40,57,270,1000,{},false);
	left_piston.set_value(false);
	chassis.moveToPoint(6,57,1600);*/



	/*chassis.turnTo(0,80,1000,true,127,false);
	left_piston.set_value(true);
	chassis.turnTo(0,50,1000,true,127,false);
	left_piston.set_value(false);
	chassis.moveToPoint(6,57,1000);//*/



	/*chassis.setPose(36,54,180);
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
	intake_mtr.move(-127);//*/



	
}

// STILL WORKING ON IT
void auton_15s_far_driver_qual()
{
	chassis.setPose(-49.5,55,225);
	left_piston.set_value(true);
	pros::delay(300);
	chassis.moveToPoint(-58,44,1500);
	chassis.moveToPose(-60,15,180,1300);
	left_piston.set_value(false);
	intake_mtr.move(-127);
	/*chassis.moveToPoint(-60,44,1000,false);
	chassis.moveToPose(-60,15,180,1300);//*/

	chassis.moveToPoint(-48,48,1000,false);
	chassis.turnTo(0,0,700);
	intake_mtr.move(127);	
	chassis.moveToPose(-5,24,90,1500);
	chassis.turnTo(-50,0,1000,true,127,false);
	chassis.moveToPoint(-24,16,1000,true,127,false);
	//chassis.moveToPoint(-40,0,1000,true,127,false);
	intake_mtr.move(-127);
	pros::delay(300);
	chassis.turnTo(-16,0,800);
	intake_mtr.move(127);

	chassis.moveToPoint(-16,0,1000,true,127,false);
	pros::delay(200);
	chassis.turnTo(-60,6,1000,true,127,false);
	left_piston.set_value(true);
	chassis.moveToPoint(-60,6,1200,true,127,false);
	left_piston.set_value(false);
	chassis.moveToPoint(-10,36,1600,false,127,false);
	chassis.turnTo(10,40,1000,true,127,false);
	chassis.moveToPoint(20,42,1000,true,127,false);
	left_piston.set_value(true);

	/*chassis.moveToPoint(-36,60,1000,false);
	chassis.moveToPoint(-4,60,1000);
	chassis.moveToPoint(-36,60,1000,false);
	chassis.turnTo(-60,30,1000);
	chassis.moveToPose(-60,30,180,1000,{.lead=0.8});
	chassis.moveToPoint(-60,0,1400);
	chassis.moveToPoint(-56,44,1500,false);
	intake_mtr.move(-127);
	chassis.turnTo(-36,0,600);
	pros::delay(300);
	left_piston.set_value(false);
	intake_mtr.move(127);
	chassis.moveToPoint(-8,22,1400);
	chassis.moveToPoint(-36,12,1000,false);
	chassis.turnTo(-60,12,600);
	chassis.moveToPoint(-60,12,900);
	chassis.moveToPoint(-24,40,1200,false);
	chassis.turnTo(0,44,1000);
	chassis.moveToPose(12,44,90,1500,{},false);
	left_piston.set_value(true);//*/


	printf("%s(): Exiting\n", __func__);
}

// STILL WORKING ON IT
void auton_15s_far_driver_elim()
{	 
	chassis.setPose(-35.5,50.25,180);
	chassis.moveToPoint(-24,0,1500);
	chassis.waitUntil(0.5);
	left_piston.set_value(true);
	intake_mtr.move(127);
	chassis.waitUntil(7);
	left_piston.set_value(false);
	chassis.waitUntilDone();
	chassis.moveToPoint(-42,56,1600,false);//*/
	chassis.turnTo(-65,27,600);
	left_piston.set_value(true);
	pros::delay(200);
	chassis.moveToPoint(-56,44,1500);
	intake_mtr.move(-127);
	chassis.turnTo(-36,0,600,true,127,false);
	left_piston.set_value(false);

	/*chassis.moveToPoint(-58,20,1000);
	chassis.moveToPoint(-56,44,1500,false);//*/

	intake_mtr.move(127);
	chassis.moveToPoint(-8,22,1400);
	chassis.moveToPose(-48,48,135,1000,{.forwards=false});
	chassis.turnTo(-60,0,600,true,127,false);
   
	//chassis.moveToPose(-55,38,180,1000,{.forwards=false});
	//chassis.turnTo(0,24,1000);
	intake_mtr.move(-127);
	chassis.moveToPoint(-36,56,1000,false);
	chassis.turnTo(0,60,600);
	intake_mtr.move(127);
	chassis.moveToPoint(4,60,1400);
	chassis.waitUntilDone();
	chassis.moveToPoint(-38,58,1000,false,127);
	chassis.turnTo(-70,0,1000,false,127,false);
	intake_mtr.move(-127);
	/*chassis.moveToPose(-57,28,0,1500);
	chassis.moveToPoint(-57,45,1000,false);
	chassis.moveToPoint(-57,28,1000);
	chassis.moveToPoint(-57,40,1000,false);//*/

	
	chassis.moveToPoint(-55,28,1300,false);
	chassis.moveToPoint(-59,45,1000);
	chassis.moveToPoint(-59,28,1000,false);
	chassis.moveToPoint(-59,40,1000);//*///Score Back



	//chassis.moveToPoint(-60,48,1000);
	//chassis.turnTo(-60,0,1000);
	//chassis.moveToPose(-60,24,180,1000);//*/

	printf("%s(): Exiting\n", __func__);
}

void safe_6_ball()
{
	chassis.setPose(-18.75,60,90);
	intake_mtr.move(127);
	chassis.moveToPoint(-7,60,1000);
	chassis.moveToPoint(-40,60,1000,false);
	chassis.turnTo(-72,60,800);
	chassis.moveToPoint(-50,51,1000);
	left_piston.set_value(true);
	chassis.turnTo(-60,0,600,true,127,false);
	left_piston.set_value(false);
	intake_mtr.move(0);
	chassis.moveToPoint(-60,0,1300);
	chassis.moveToPoint(-56,44,1500,false);
	intake_mtr.move(-127);
	chassis.turnTo(-36,0,600);
	pros::delay(300);
	left_piston.set_value(false);
	intake_mtr.move(127);
	chassis.moveToPoint(-8,22,1400);
	chassis.moveToPoint(-36,12,1000,false);
	chassis.turnTo(0,44,1000);

	

	/*chassis.turnTo(-24,24,800);
	chassis.moveToPose(-24,24,90,1000);

	/*chassis.moveToPose(-60,27,180,1000);
	chassis.waitUntil(15);
	left_piston.set_value(false);
	chassis.waitUntilDone();
	chassis.moveToPoint(-60,0,1200,true,127,false);
	chassis.moveToPose(-48,48,235,1000,{.forwards=false});
	
	chassis.turnTo(-24,24,1000);
	chassis.moveToPoint(24,24,1000);
	chassis.turnTo(0,24,1000);
	chassis.moveToPoint(-9,24,1000);//*/

}	
// STILL WORKING ON IT

ASSET(ralphskills_txt);
void auton_60s_skills_1()
{
	printf("%s(): Exiting\n", __func__);
    int shooting_time = 25000;	
	int volts = 9000;
	int funny_volts = 120000;
	chassis.setPose(-44.5,52.5,45);
	chassis.moveToPoint(-46.5,49,800,true,127,false);
	left_piston.set_value(true);
	chassis.turnTo(0,60,400,true,127,false);
	left_piston.set_value(false);
	chassis.moveToPose(-58,26,0,1000,{.forwards=false});
	chassis.moveToPoint(-58,20,750,false);
	chassis.moveToPose(-54,40,0,1000);
	chassis.turnTo(50,9,500);
	chassis.moveToPoint(-60,42,1000);
	chassis.moveToPose(-58,42,90,1000);
	cata_motors.move_voltage(volts);
	pros::delay(shooting_time);
	chassis.follow(ralphskills_txt,15,10000,true,false);
	//pros::delay(1000);


} // end auton_60s_skills_1()

// Auton skills number 2 fully done
void auton_60s_skills_2()	
{	
	printf("%s(): Exiting\n", __func__);
	int shooting_time = 23000;	
	int volts = 8800;
	int funny_volts = 120000;
	chassis.setPose(-44.5,52.5,45);
	chassis.moveToPoint(-46.5,49,800,true,127,false);
	left_piston.set_value(true);
	chassis.turnTo(0,60,400,true,127,false);
	left_piston.set_value(false);
	chassis.moveToPose(-58,26,0,1000,{.forwards=false});
	chassis.moveToPoint(-58,20,750,false);
	chassis.moveToPose(-54,40,0,1000);
	chassis.turnTo(50,10,500);
	chassis.moveToPoint(-60,42,1000);
	cata_motors.move_voltage(volts);
	pros::delay(shooting_time);
	//pros::delay(1000);
	cata_motors = 40;
	chassis.arcade(30,0);
	pros::delay(250);
	chassis.arcade(0,0);
	chassis.moveToPose(-36,57,270,1000,{.forwards=false});
	chassis.moveToPoint(36,57,3000,false,127,false);//CROSS MIDDLE
	chassis.moveToPose(63,26,0,1800,{.forwards=false, .minSpeed = 100});
	cata_motors = 0;

	chassis.moveToPoint(60,44,1000,true);	
	chassis.moveToPoint(60,20,1200,false,127,false);	
	chassis.setPose(60,32,0);

	chassis.moveToPoint(60,40,1000);
	chassis.turnTo(0,36,1000);

	chassis.moveToPoint(17,34,1400);
	chassis.turnTo(12,0,800,true,127,false);
	left_piston.set_value(true);
	chassis.moveToPose(24,13,90,1000);
	chassis.turnTo(60,13,1000);
	chassis.moveToPoint(50,13,1700,true,127,false);
	/*chassis.moveToPose(20,16,90,1000,{.forwards=false});
	chassis.moveToPoint(50,16,1700,true,127,false);//*/
	left_piston.set_value(false);

	chassis.moveToPoint(18,12,1000,false);
	chassis.turnTo(18,-12,700);
	chassis.moveToPoint(18,-15,1000);
	chassis.turnTo(60,-3,1000);
	left_piston.set_value(true);
	pros::delay(200);
	chassis.moveToPoint(50,-3,1700,true,127,false);
	left_piston.set_value(false);


	chassis.moveToPoint(18,-17,1000,false);
	chassis.turnTo(18,12,700,false);
	chassis.moveToPoint(18,8,1000,false);
	chassis.turnTo(60,10,1000);
	left_piston.set_value(true);
	pros::delay(200);
	chassis.moveToPoint(50,10,1700,true,127,false);
	chassis.moveToPoint(18,8,1000,false);
	chassis.moveToPoint(50,10,1700,true,127,false);

	chassis.moveToPoint(34,8,1000,false);
	left_piston.set_value(false);

	chassis.turnTo(36,-60,700);
	chassis.moveToPoint(44,-44,1300); 
	chassis.turnTo(60,-34,1500,true,127);
	chassis.moveToPoint(60,-34,1000);
	chassis.turnTo(60,0,1500,true,127);
	chassis.moveToPoint(60,-0,1000);
	chassis.moveToPoint(64,-38,1000,false);
	chassis.turnTo(64,0,1500,false,127);
	chassis.moveToPoint(66,-0,1000,false);
	/*chassis.moveToPose(60,-30,0,1200);
	chassis.moveToPoint(60,-0,1000);
	chassis.moveToPoint(60,-32,1000,false);
	chassis.turnTo(-70,-62,500);
	chassis.moveToPose(72,-0,180,1000,{.forwards=false});
	chassis.moveToPose(48,-57,270,500,{},false);
	/*lift_pistons.set_value(true);
	chassis.moveToPoint(-2,-57,1000,true,127,false);
	lift_pistons.set_value(false);
	PTO_piston.set_value(true);
	chassis.arcade(127,0);
	pros::delay(00);
	chassis.arcade(0,0);//*/


	/*chassis.moveToPoint(4,-57,1000);
	lift_pistons.set_value(true);
	chassis.moveToPoint(-3,-60,1000,true,127,false);
	PTO_piston.set_value(true);
	lift_pistons.set_value(false);
	chassis.arcade(127,0);
	pros::delay(600);
	chassis.arcade(0,0);//*/
}



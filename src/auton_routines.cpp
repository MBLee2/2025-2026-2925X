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
	chassis.setPose(51.5,56.5,315);
	chassis.moveToPoint(53,53,1000,false,127,false);
	right_piston.set_value(true);
	pros::delay(500);
	chassis.turnTo(0,60,900);
	chassis.arcade(50,0);
	pros::delay(100);
	chassis.arcade(0,0);
	pros::delay(500);
	right_piston.set_value(false);
	chassis.turnTo(0,60,1000);
	intake_mtr.move(127);

	chassis.moveToPose(62,27,0,1000,{.forwards=false,.minSpeed=90,});
	chassis.turnTo(62,0,700,false);
	chassis.moveToPoint(43,58,3000,true,90);
	chassis.waitUntilDone();
	chassis.turnTo(0,58,1000,true,90);
	chassis.moveToPose(22.625,58,270,1500,{},false);//*/
	intake_mtr.move(127);
	pros::delay(500);
	intake_mtr.move(-127);

	
}

void rushWP(){
	printf("%s(): Exiting\n", __func__);
	chassis.setPose(51.5,56.5,315);
	right_piston.set_value(true);
	pros::delay(500);
	chassis.turnTo(0,60,900);
	chassis.moveToPoint(45,60,1000);
	right_piston.set_value(false);
	intake_mtr.move(127);
	chassis.moveToPose(30,6,195,2500);

	chassis.moveToPoint(46,60,1600,false,127);
	pros::delay(200);	
	chassis.turnTo(0,60,700,true,127,false);
	intake_mtr.move(-127);
	pros::delay(500);
	chassis.moveToPose(62,28,0,1000,{.forwards=false,});
	chassis.moveToPoint(64,22,1000,false);
	chassis.moveToPoint(42,56,1000);
	chassis.turnTo(0,56,700);
	left_piston.set_value(true);
	chassis.moveToPoint(19.5,56,1500,true);
	/*chassis.moveToPoint(36,36,1000);
	/*chassis.turnTo(0,60,1000);
	chassis.moveToPose(9.25,60,270,1500,{},false);//*/
}

// STILL WORKING ON IT
void auton_15s_near_driver_elim()
{
	
	chassis.setPose(51.5,56.5,315);
	right_piston.set_value(true);
	pros::delay(200);
	chassis.turnTo(0,60,500);
	chassis.moveToPoint(45,60,700,true,127,false);
	right_piston.set_value(false);
	intake_mtr.move(127);
	chassis.moveToPose(30,10,195,1600,{.maxSpeed=100});
	chassis.waitUntilDone();
	chassis.moveToPoint(30,22,800);
	chassis.turnTo(-30,22,600);
	//left_piston.set_value(true);

	chassis.moveToPoint(16,22,1000);
	chassis.moveToPoint(46,53,1600,false,127);
	intake_mtr.move(0);
	chassis.turnTo(0,60,700,true,127,false);
	intake_mtr.move(-127);
	pros::delay(400); 
	/*chassis.turnTo(24,16,0,800);
	chassis.moveToPoint(28,20,1400);
	intake_mtr.move(127);
	chassis.turnTo(0,2,500);
	chassis.moveToPoint(21,16,1000);
	chassis.moveToPoint(46,53,1600,false,127);
	intake_mtr.move(0);
	chassis.turnTo(0,60,700,true,127,false);
	intake_mtr.move(-127);
	pros::delay(500);*/
	chassis.moveToPose(62,28,0,1000,{.forwards=false,});
	chassis.moveToPoint(64,22,1000,false);
	chassis.moveToPoint(42,56,1000);
	chassis.turnTo(0,56,700);
	left_piston.set_value(true);
	chassis.moveToPoint(19.5,56,1500,true);
	chassis.moveToPoint(72,54,1500,false);
	chassis.turnTo(0,84,600);
	right_piston.set_value(true);

	
	/*chassis.moveToPose(62,28,0,1000,{.forwards=false,});
	chassis.moveToPoint(64,22,1000,false);
	chassis.moveToPoint(42,56,1000);
	chassis.turnTo(0,56,700);
	left_piston.set_value(true);
	chassis.moveToPoint(15.5,56,1500,true);//*/



	
}

void rushelim()
{
	printf("%s(): Exiting\n", __func__);
	chassis.setPose(30.25,56,180);
	chassis.moveToPoint(25,25,1400);
	chassis.waitUntil(0.5);
	left_piston.set_value(true);
	intake_mtr.move(127);
	chassis.waitUntil(8);
	left_piston.set_value(false);
	chassis.waitUntilDone();
	chassis.moveToPoint(25,28,900,false);
	/*chassis.turnTo(-25,31,600);
	left_piston.set_value(true);
	right_piston.set_value(true);
	//chassis.moveToPoint(11,31,800);
	/*chassis.turnTo(44,58,600,false);
	chassis.moveToPoint(42,56,1400,false);
	left_piston.set_value(false);
	
	chassis.turnTo(0,112,900,true,127,false);
	right_piston.set_value(true);
	chassis.turnTo(-14,-62, 1200);
	chassis.moveToPoint(-14, -62,1200);//*/

	/*chassis.moveToPoint(-6,6,900,true,127,false);
	chassis.moveToPoint(12,6,1000,false);
	right_piston.set_value(false);//*/
	
	/*chassis.moveToPoint(40,50.25,1200,false);
	chassis.turnTo(0,60,700);

	pros::delay(9000);

	chassis.moveToPoint(10,60, 1000);
	chassis.moveToPose(54,50,315,1400);
	right_piston.set_value(true);

	chassis.moveToPoint(38,38,1000,false);
	chassis.turnTo(60,40,1000,false);
	chassis.moveToPoint(52,40,1500,false);
	chassis.turnTo(57,0,1000);
	chassis.moveToPoint(60,0,1000,false);
	chassis.moveToPose(36,60,270,1000);//*/
}
// STILL WORKING ON IT
void auton_15s_far_driver_qual()
{
	chassis.setPose(-49.5,55,225);
	right_piston.set_value(true);
	pros::delay(300);
	chassis.moveToPoint(-58,44,1500);
	chassis.moveToPose(-60,15,180,1300);
	right_piston.set_value(false);
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
	chassis.moveToPose(-60,30,180,1000,{	=0.8});
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
	chassis.arcade(-50,0);
	pros::delay(150);
	chassis.arcade(0,0);
	printf("%s(): Exiting\n", __func__);
}

void safe_6_ball()
{
	chassis.setPose(-12,58.25,90);
	intake_mtr.move(127);
	chassis.moveToPoint(-8,58,1000);
	chassis.moveToPose(-30,58,90,1000,{.forwards=false,.maxSpeed = 98});
	intake_mtr.move(0);
	chassis.turnTo(-64,24,500,true,127,false);
	intake_mtr.move(-127); 
	pros::delay(300);

	chassis.turnTo(-72,60,600,true,127,false);
	right_piston.set_value(true);
	chassis.moveToPoint(-41,58,1400);
	chassis.turnTo(-56,0,1500,true,127,false);
	right_piston.set_value(false);

	
	//chassis.moveToPoint(-48,60,1000,false);
	chassis.moveToPoint(-55,4,1100);
	chassis.moveToPoint(-33,45,1600,false);
	chassis.turnTo(0,0,600);	
	chassis.moveToPoint(-20,24,1300);
	intake_mtr.move(127);
	chassis.turnTo(24,23,600);
	chassis.moveToPoint(-3,23,1000);
	chassis.turnTo(-20,18,1000,false);
	chassis.moveToPoint(-15,18,1000,false);
	chassis.turnTo(-60,0,1000,true,127,false);
	intake_mtr.move(-127);
	pros::delay(300);
	chassis.moveToPoint(-18,20,1300,false);
	chassis.turnTo(0,0,1000);
	intake_mtr.move(127);
	chassis.moveToPoint(-1,10,1000);	
	chassis.turnTo(-60,10,1000,true,127,false);
	left_piston.set_value(true);
	right_piston.set_value(true);
	intake_mtr.move(-127);
	pros::delay(300);
	chassis.moveToPoint(-60,12,1000);
	chassis.moveToPoint(-30,12,1000);
	
	
		
	/*intake_mtr.move(-127);
	chassis.waitUntil(22);
	right_piston.set_value(false);

	chassis.waitUntilDone();
	chassis.moveToPoint(-48,48,1000,false);
	chassis.turnTo(-72,36,1000);	
	chassis.moveToPoint(-54,30,1000);
	chassis.moveToPose(-46,48,235,1200,{.forwards=false});
	chassis.moveToPoint(-24,24,1000);
	chassis.turnTo(24,24,500);
	intake_mtr.move(127);
	chassis.moveToPoint(-12,24,1000);
	chassis.moveToPoint(-24,12,00,false);
	chassis.turnTo(-60,0,1000);
	/*intake_mtr.move(-127);
	chassis.turnTo(0,0,600);*/





	/*left_piston.set_value(true);
	right_piston.set_value(false);
	/*
	intake_mtr.move(0);
	chassis.moveToPoint(-60,0,1300);
	chassis.moveToPoint(-56,44,1500,false);
	intake_mtr.move(-127);
	chassis.turnTo(-36,0,600,true,127,false);
	left_piston.set_value(false);

	/*chassis.moveToPoint(-58,20,1000);
	chassis.moveToPoint(-56,44,1500,false);//*/

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

ASSET(skillsPathPart1_txt);
void auton_60s_skills_1()
{
	printf("%s(): Exiting\n", __func__);
	int shooting_time = 22000;
	cata_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	//int shooting_time = 21000;	
	int volts = 9450;
	int funny_volts = 120000;
   	chassis.setPose(-48,56.8388347648,45);
   	chassis.moveToPoint(-61,38,1000,false);
   	chassis.turnTo(-61,0,800,false);
   	chassis.moveToPoint(-61,20,600,false);
   	chassis.moveToPoint(-58,48,1000);
   	chassis.turnTo(60,14,1000);
	//chassis.moveToPoint(-62,44,600);
	chassis.moveToPoint(-62,48,350,false,60,false);

	chassis.waitUntilDone();
	cata_motors.move_voltage(volts);
	pros::delay(shooting_time);
	//pros::delay(1000);
	while(true){
    if(limitSwitch.get_new_press())
    {
        cata_motors.brake();
		break;
    }
	} 
} // end auton_60s_skills_1()

// Auton skills number 2 fully done
void auton_60s_skills_2()	
{	
	printf("%s(): Exiting\n", __func__);
	int shooting_time = 22000;
	cata_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	//int shooting_time = 21000;	
	int volts = 9450;
	int funny_volts = 120000;
   	chassis.setPose(-48,56.8388347648,45);
   	chassis.moveToPoint(-61,38,1000,false);
   	chassis.turnTo(-61,0,800,false);
   	chassis.moveToPoint(-61,20,600,false);
   	chassis.moveToPoint(-58,48,1000);
   	chassis.turnTo(60,14,1000);
	//chassis.moveToPoint(-62,44,600);
	chassis.moveToPoint(-62,48,350,false,60,false);

	chassis.waitUntilDone();
	cata_motors.move_voltage(volts);
	pros::delay(shooting_time);
	//pros::delay(1000);
	while(true){
    if(limitSwitch.get_new_press())
    {
			cata_motors.brake();
			break;
    }
	}
	chassis.moveToPoint(-13,36,1300);
	chassis.turnTo(-12,0,800);
	chassis.moveToPoint(-11,-36,2300);//cross sides
	right_piston.set_value(true);
	left_piston.set_value(true);
	chassis.turnTo(-58,-39,1000);
	chassis.moveToPoint(-38,-39,1000);
	left_piston.set_value(false);
	chassis.turnTo(-30,-72,700);
	chassis.moveToPoint(-44,-62,800);

	chassis.turnTo(40,-62,1000);
	right_piston.set_value(false);

	left_piston.set_value(true);
	chassis.moveToPoint(48,-54,2800);//Barrier cross
	/*chassis.turnTo(88,0,600,true,90);	
	chassis.moveToPoint(62,-50,1000,true,90);
	cata_motors.move(0);
	chassis.turnTo(64,-20,600,true,90);*/
	chassis.moveToPose(62,-36,0,1000,{.minSpeed = 90});
	left_piston.set_value(true);
	chassis.moveToPoint(61,-10,1000);
	chassis.moveToPoint(61,-39,1000,false);
	chassis.moveToPoint(64,-10,1000);
	chassis.moveToPoint(56,-36,1000,false);//two pushes
	left_piston.set_value(false);
	chassis.turnTo(15,-28,700,false);
	chassis.moveToPoint(15,-28,1000,false);
	chassis.turnTo(15,0,600); 
	chassis.moveToPoint(15,-4,1000);
	right_piston.set_value(true);
	left_piston.set_value(true);
	chassis.turnTo(40,-4,700);
	chassis.moveToPoint(50,-4,1700);
	chassis.moveToPoint(40,-4,600,false);
	left_piston.set_value(false);

	chassis.moveToPoint(15,-10,1000,false);
	chassis.turnTo(15,0,600); 
	chassis.moveToPoint(15,15,1200);//move to other side for scoring
	chassis.turnTo(40,15,700);
	right_piston.set_value(true);
	left_piston.set_value(true);
	chassis.moveToPoint(50,15,1700);
	chassis.moveToPoint(40,15,600,false);
	right_piston.set_value(false);
	left_piston.set_value(false);

	chassis.moveToPoint(15,15,1000,false);
	chassis.turnTo(42,48,600);
	left_piston.set_value(true);
	chassis.moveToPoint(42,65,1300);
	chassis.turnTo(60,45,700);
	left_piston.set_value(false);
	chassis.moveToPose(62,28,180,1000,{.minSpeed = 100});
	right_piston.set_value(true);
	chassis.moveToPoint(62,28,1000);
	chassis.turnTo(62,20,600);
	chassis.moveToPoint(62,20,1000);
	chassis.moveToPoint(48,48,1000,false); 


	
	/*chassis.turnTo(16,-18,600);
	left_piston.set_value(false);
	chassis.moveToPoint(16,-18,1000)
	//*/
}





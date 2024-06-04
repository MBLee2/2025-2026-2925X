#include "auton_basics.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "controls.h"
#include "lemlib/api.hpp"
#include <sys/_stdint.h>
#include <type_traits>

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
void auton_15s_near_driver_qual() //DONE
{
    int time = pros::millis();
	printf("%s(): Exiting\n", __func__);
    chassis.setPose(50.5,55.5,135);
    chassis.moveToPoint(53, 52, 1000,{},false);
    intake_mtr.move(20);
    left_piston.set_value(true);
    pros::delay(300);
    chassis.moveToPose(30, 60, 90, 1300,{.forwards=false},false);
    intake_mtr.move(127);
    chassis.moveToPoint(2.25, 61,1000,{.forwards=false},false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    left_piston.set_value(false);
    while(true)
    {
        chassis.moveToPose(2.25,61,90,3000,{.forwards=false});
        pros::delay(100);
    }





    
    /*chassis.moveToPoint(0,60,3000,{.maxSpeed=40}); 
    // wait until either triball is in intake or the motion stops
    while(distance_sensor.get()>85 && chassis.isInMotion()) {
    pros::delay(20);
    }
    chassis.cancelMotion();//*/

}
void rushWP(){ // 
    int time = pros::millis();
	printf("%s(): Exiting\n", __func__);
    chassis.setPose(50.5,55.5,135);
    chassis.moveToPoint(53, 52, 1000,{},false);
    intake_mtr.move(30);
    left_piston.set_value(true);
    pros::delay(300);
    chassis.moveToPose(30, 60, 90, 1300,{.forwards=false});
    chassis.moveToPoint(22, 60, 1000,{.forwards=false});
    chassis.moveToPoint(32, 60,1000);
    left_piston.set_value(false);

    chassis.turnToPoint(24, 0, 1000, {});
    chassis.moveToPoint(25, 10, 1000);
    intake_mtr.move(127);
    left_piston.set_value(false);

    chassis.moveToPoint(42, 62,10000,{.forwards = false});
    chassis.turnToHeading(180,1000,{},false);
    reset();
    pros::delay(500);
    chassis.turnToHeading(270, 1000, {},false);
    intake_mtr.move(-127);
    pros::delay(500);
    chassis.turnToHeading(135, 1000,{});
    chassis.moveToPose(18, 62.75,90,1000,{.forwards=false},false);
    right_piston.set_value(true);
    chassis.moveToPoint(8, 62.75, 1000,{.forwards=false});
    chassis.moveToPoint(8.25, 62.75, 1000,{.minSpeed=50},false);
    right_piston.set_value(false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    while(true)
    {
        chassis.moveToPose(8.25,63,90,3000,{.forwards=false});
        pros::delay(100);
    }

}
void rushelim(){
     printf("%s(): Exiting\n", __func__);
    int start_time = pros::millis();
    double temp;
    chassis.setPose(39,56.5,193.3);
    right_piston.set_value(true);
    intake_mtr.move(95);
    chassis.moveToPoint(26, 9, 1300);
    chassis.waitUntil(6);
    right_piston.set_value(false);
    chassis.waitUntil(24);
    intake_mtr.move(127);
    chassis.waitUntilDone();
    chassis.moveToPoint(26, 30, 1000,{.forwards = false,.maxSpeed = 100});
    chassis.turnToPoint(30, 60, 1000, {.maxSpeed = 100},false);
    intake_mtr.move(-110);
    pros::delay(500);
    chassis.turnToPoint(6, 11, 1000, {.maxSpeed = 100},false);
    intake_mtr.move(127);
    chassis.moveToPoint(8.5,8.5, 200,{.maxSpeed=100,.minSpeed=20,.earlyExitRange=2});

    chassis.moveToPoint(7.25,9, 1300,{.maxSpeed=50});
    chassis.moveToPoint(48, 50, 1000,{.forwards = false,.maxSpeed = 90});
    chassis.turnToHeading(180, 1000, {.maxSpeed = 90},false);
    reset();
    pros::delay(100);
    chassis.turnToPoint(50, 51, 1000, {.forwards=false});
    chassis.moveToPoint(50, 51, 1000,{.forwards=false});
    
    //chassis.moveToPoint(temp-7, 46,1000);
    chassis.turnToHeading(315, 1000, {.minSpeed = 20,.earlyExitRange = 2},false);
    temp = chassis.getPose().x;
    right_piston.set_value(true);

    chassis.moveToPose(30, 61, 270, 1300,{},false);
    intake_mtr.move(-127);
    pros::delay(300);
    chassis.turnToPoint(44, 44, 1000,{});
    chassis.moveToPoint(44, 44, 1000,{});
    chassis.turnToPoint(30, 64, 600,{.forwards=false},false);
    chassis.moveToPoint(30, 64, 1300,{.forwards=false},false);

    chassis.turnToHeading(120, 500, {.minSpeed = 20,.earlyExitRange = 2});
    chassis.turnToHeading(90, 1000, {},false);    
    /*chassis.turnToHeading(290, 1000, {.minSpeed = 20,.earlyExitRange = 2});
    chassis.turnToHeading(270, 1000, {},false);
    intake_mtr.move(-127);
    pros::delay(300);
    left_piston.set_value(false);

   
    
    //chassis.turnToHeading(90, 1000, {});
    /*right_piston.set_value(false);
    chassis.waitUntilDone();
    //chassis.turnToHeading(90, 1000, {.maxSpeed = 90},false);
    temp = chassis.getPose().y;
    chassis.moveToPose(0, temp,270, 1000,{.maxSpeed=100});
    chassis.moveToPoint(26, temp, 1000,{.forwards=false,.maxSpeed=100},false);
    chassis.turnToHeading(180, 1000, {});
    reset();
    chassis.turnToPoint(0, temp,1000, {.forwards=false,.maxSpeed=100},false);
    temp =  chassis.getPose().y;
    chassis.moveToPose(3, temp,90,1000,{.forwards=false,.maxSpeed=100},false);
    //*/
    chassis.moveToPoint(5, 66, 1000,{.forwards=false});
    master.print(0, 0, "time: %d" ,pros::millis() - start_time);//*/

}

void auton_15s_far_driver_qual() // FAR DRIVER QUAL
{
    int start_time = pros::millis();
    int temp;
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake_mtr.move(127);
    chassis.setPose(-12.4,60,90);
    reset();
    pros::delay(400);
    temp = chassis.getPose().y;
    chassis.moveToPoint(-7, temp, 1000,{.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPoint(-44, temp, 1300,{.forwards=false,.maxSpeed=80,.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPose(-63,24,0, 1300,{.forwards=false,.lead=0.8});
    chassis.waitUntil(6);
    left_piston.set_value(true);
    chassis.moveToPoint(-65, 20, 700,{.forwards=false,.maxSpeed=127},false);
    left_piston.set_value(false);
    temp = chassis.getPose().x;
    chassis.moveToPoint(temp, 40,1000,{.minSpeed=40,.earlyExitRange=2});
    chassis.turnToHeading(340, 1000,{.minSpeed = 50,.earlyExitRange = 3});
    chassis.turnToHeading(180, 1000, {},false);
    temp = chassis.getPose().x;
    intake_mtr.move(-127);
    pros::delay(400);
    chassis.moveToPoint(temp-7, 14, 1000,{.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPoint(temp-7, 40, 1000,{.forwards=false,.minSpeed=40,.earlyExitRange=2});

    chassis.turnToHeading(90, 1000, {},false);
    reset();
    chassis.moveToPoint(-22, 21, 1000,{.minSpeed=40,.earlyExitRange=2},false);
    intake_mtr.move(127);
    temp =  chassis.getPose().y;
    chassis.turnToHeading(90, 1000, {.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPoint(- 7, temp, 600,{.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPoint(-16, temp, 1000,{.forwards=false});
    chassis.turnToHeading(180, 1000, {},false);
    intake_mtr.move(-127);

    temp=chassis.getPose().x;
    chassis.moveToPoint(temp+6, 54, 1200,{.forwards=false},false);
    left_piston.set_value(true);

}


void safe_6_ball() //FAR ELIM
{
    int start_time = pros::millis();
    int temp;
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake_mtr.move(127);
    chassis.setPose(-12.4,60,90);
    reset();
    pros::delay(400);
    temp = chassis.getPose().y;
    chassis.moveToPoint(-6, temp, 1000,{.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPoint(-44, temp, 1300,{.forwards=false,.maxSpeed=80,.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPose(-63,24,0, 1300,{.forwards=false,.lead=0.8});
    chassis.waitUntil(6);
    left_piston.set_value(true);
    chassis.moveToPoint(-65, 20, 700,{.forwards=false,.maxSpeed=127},false);
    left_piston.set_value(false);
    temp = chassis.getPose().x;
    chassis.moveToPoint(temp, 38,1000,{.minSpeed=40,.earlyExitRange=2});
    chassis.turnToHeading(200, 1000, {.minSpeed=40,.earlyExitRange=2});
    chassis.turnToPoint(-70, 16, 300, {});
    temp = chassis.getPose().x; 
    //chassis.turnToPoint(temp-6, 14, 1000, {.minSpeed=40,.earlyExitRange=2},false);
    intake_mtr.move(-127);
    pros::delay(300);
    chassis.arcade(127, 0);
    pros::delay(500);
    //chassis.moveToPoint(temp-6, 14, 1000,{.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPoint(temp-7, 36, 1000,{.forwards=false,.minSpeed=40,.earlyExitRange=2});

    chassis.turnToHeading(90, 1000, {},false);
    reset();
    pros::delay(100);
    chassis.moveToPoint(-22, 21, 1000,{.minSpeed=40,.earlyExitRange=2},false);
    intake_mtr.move(127);
    temp =  chassis.getPose().y;
    chassis.turnToHeading(90, 1000, {.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPoint(- 7, temp, 600,{.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPoint(-24,24, 600,{.forwards=false,.minSpeed=40,.earlyExitRange=2});
    chassis.turnToHeading(239 , 1000, {},false);
    chassis.waitUntil(2);
    intake_mtr.move(-127);
    pros::delay(500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-14, 24, 1000,{.forwards =false});
    chassis.turnToPoint(-3, 2, 1000,{.minSpeed=40,.earlyExitRange=2});
    intake_mtr.move(127);
    chassis.moveToPoint(-3, 0, 1000,{.minSpeed=40,.earlyExitRange=2});
    chassis.turnToHeading(90,1000, {.minSpeed=40,.earlyExitRange=2},false);
    temp = chassis.getPose().y;
    left_piston.set_value(true);
    right_piston.set_value(true);
    chassis.moveToPoint(-60, temp+15, 1000,{.forwards=false,.minSpeed=40,.earlyExitRange=2});
    chassis.moveToPoint(-35, temp+15, 1000,{.minSpeed=40,.earlyExitRange=2},false);
    master.print(0, 0, "time: %d" ,pros::millis() - start_time);

    left_piston.set_value(false);
    right_piston.set_value(false);
    /*chassis.tank(127, -30);
    pros::delay(600);
    chassis.arcade(0, 0);
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 1000,{.minSpeed=40,.earlyExitRange=2},false);*/
    chassis.turnToHeading(240, 1000,{.minSpeed=40,.earlyExitRange=2});
    chassis.turnToPoint(-70, 0, 300, {});
    intake_mtr.move(-127);    
    chassis.moveToPoint(-80, temp+15,1000);
    master.print(0, 0, "time: %d" ,pros::millis() - start_time);

    
    /*chassis.turnToPoint(-60, temp, 1000, {},false);
    intake_mtr.move(-127);
    chassis.moveToPoint(-60, temp, 1000);µ
    chassis.moveToPoint(-30, temp, 1000,{.forwards=false});
    //*/
}	

void auton_15s_far_driver_elim(){
	printf("%s(): Exiting\n", __func__);

}
void DescoreRushElim(){
    printf("%s(): Exiting\n", __func__);

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





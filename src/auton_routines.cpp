#include "auton_basics.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "controls.h"
#include "lemlib/api.hpp"
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#include <type_traits>

ASSET(matchloadturn4ball_txt);
ASSET(touchbar_txt);

// Auton routine start positions
auton_routine null_routine  {    0,     0,   0,   "None - Invalid Routine",      nullptr                 };
auton_routine near_driver_qual { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_near_driver_qual};
auton_routine blue_ring_rush { 1.000, -1.300, 190, "15S Auton - Near Driver # 2", &ring_rush}; 
auton_routine near_driver_elim { 1.000, -1.300, 190, "15S Auton - Near Driver # 2",&DescoreRushElim};  // to be updated
auton_routine near_driver_elim2 { 1.000, -1.300, 190, "15S Auton - Near Driver # 2", &rushelim};  // to be updated



auton_routine far_from_driver_qual { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_far_driver_qual};
auton_routine far_from_driver_elim { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &auton_15s_far_driver_elim};
auton_routine far_from_driver_elim2  { -0.600, 0.600, 180, "extra_1", &safe_6_ball};


auton_routine skills_1 		{ -0.600, 0.600, 180, "60S Auton - Skills # 1", &auton_60s_skills_1};
auton_routine skills_2 		{ -0.600, 0.600, 180, "60S Auton - Skills # 2", &auton_60s_skills_2};

float max_speed = 60; // forward back max speed
int max_speed1 = (int)max_speed; //turn max speed

//FULLY DONE
void auton_15s_near_driver_qual() //DONE
{

}
void rushWP(){ // 
    
}
void rushelim(){
    printf("%s(): Exiting\n", __func__);
    
}

void ring_rush() //BLUE
{
    
    chassis.setPose(48,-52.25,0);
    chassis.moveToPose(50, -10,15,3000,{.maxSpeed=max_speed});
    intake.move(-127);
    pros::delay(300);
    intake.move(0);
    lift.move(127);
    pros::delay(400);
    lift.move(0);
    intake.move(127);
    mogo_clamp.retract();
    chassis.waitUntilDone();
    chassis.moveToPoint(48, -17, 1000,{.forwards=false,.maxSpeed=max_speed});
    intake.move(0);
    chassis.turnToPoint(46, 0,1000,{.maxSpeed=max_speed1});
    chassis.moveToPoint(46, -8, 2000,{.maxSpeed=max_speed});
    intake.move(127);
    chassis.turnToHeading(270, 1000,{.maxSpeed=max_speed1});
    chassis.waitUntil(3);
    intake.move(0);
    chassis.waitUntil(6);
    mogo_rush.extend();
    chassis.waitUntilDone();
    intake.move(0);
    chassis.turnToHeading(0, 1000,{.maxSpeed=max_speed1}); 
    chassis.turnToPoint(24, -24, 1000,{.forwards=false,.maxSpeed=max_speed1});
    chassis.moveToPoint(20, -2, 1000,{.forwards=false,.maxSpeed=max_speed});
    mogo_rush.retract();
    chassis.waitUntil(18);
    mogo_clamp.set_value(true);
    /*chassis.moveToPoint(36, -24, 1000,{.maxSpeed=max_speed});
    mogo_clamp.extend();
    chassis.turnToHeading(270, 1000,{.maxSpeed=max_speed1});
    chassis.moveToPoint(2, -24, 1000,{.forwards=false, .maxSpeed=max_speed});
    chassis.waitUntilDone();
    mogo_clamp.retract();//*/


}
void auton_15s_far_driver_qual() // FAR DRIVER QUAL
{

}


void safe_6_ball() //FAR ELIM
{
   
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
    chassis.setPose(-24, -60.75, 180);
    chassis.moveToPoint(-24, -48, 1000, {.forwards = false, .maxSpeed = max_speed}); //Move towards first ring
    intake.move(-127); //outake to release basket
    chassis.waitUntil(4);
    mogo_clamp.set_value(true); //clamp on mobile goal
    chassis.waitUntil(5);
    intake.move(0); //stop intake to avoid outaking preload
    pros::Task lift_up1([=] { //raise lift (later lift may get in way near the wall)
        moveLift(300);
    });

    //intake other rings in bottom left corner
    chassis.moveToPoint(-24, -24, 2000, {.maxSpeed = max_speed});
    intake.move(127); 
    chassis.moveToPoint(-48, -24, 2000, {.maxSpeed = max_speed});
    chassis.moveToPoint(-48, -64, 3000, {.maxSpeed = max_speed - 10});
    chassis.waitUntilDone();
    chassis.moveToPoint(-40, -48, 1000, {.forwards = false, .maxSpeed = max_speed});
    chassis.moveToPoint(-60, -52, 2000, {.maxSpeed = max_speed});

    //move to place goal in corner
    chassis.turnToHeading(0, 1000, {.maxSpeed = max_speed1});
    chassis.waitUntil(30);
    intake.move(0); //save last intaked ring for basket
    chassis.moveToPoint(-62, -62, 1000, {.forwards = false, .maxSpeed = max_speed});
    pros::Task lift_down1([=] {
        moveLift(60);
        setBasket(true);
    });
    chassis.waitUntilDone();
    mogo_clamp.set_value(false);
    setBasket(true);
    pros::Task basket1(basketRings);

    chassis.moveToPose(-45, -24, 0, 1000, {.maxSpeed = max_speed});
    chassis.waitUntil(30);
    readjustHeading(2, 0);
    chassis.waitUntilDone();
    setBasket(false);
    intake.move(0);
    chassis.setPose(-72 + findDistToWall(1), -72 + findDistToWall(2), chassis.getPose().theta);
    chassis.moveToPoint(-48, 0, 2000, {.maxSpeed = max_speed});
    pros::Task lift_up2([=] {
        moveLift(480);
    });
    chassis.moveToPoint(-58.75, 0, 2000, {.maxSpeed = max_speed});
    moveLift(240);
    chassis.moveToPoint(-48, 0, 1000, {.forwards = false, .maxSpeed = max_speed});
    
    pros::delay(5);
} // end auton_60s_skills_1()
// Auton skills number 2 fully done
void auton_60s_skills_2()	
{	
	
}





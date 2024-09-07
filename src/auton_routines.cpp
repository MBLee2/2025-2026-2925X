#include "auton_basics.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "pros/distance.hpp"
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
auton_routine blue_goal_rush { 1.234, -1.234,  90, "15S Auton - Near Driver # 1", &goal_rush};
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
void goal_rush() //BLUE
{
    /*chassis.setPose(-48, -51.5, 180);
    chassis.moveToPoint(-48, -7.5, 3000, {.forwards = false, .maxSpeed = max_speed * 1.5f});
    chassis.waitUntil(38.5);
    mogo_clamp.set_value(true);
    pros::delay(200);*/
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48, -7.5, 180);

    chassis.moveToPoint(-38.5, -18, 2000, {.maxSpeed = max_speed});
    chassis.moveToPoint(-38.5, -12, 2000, {.forwards = false, .maxSpeed = max_speed});
    chassis.waitUntilDone();
    intake.move(-127);
    pros::delay(750);

    intake.move(127);
    lift.move(127);
    chassis.moveToPose(-60, -59, 195, 3000, {.maxSpeed = max_speed});
    chassis.waitUntil(10);
    lift.brake();
    mogo_rush.set_value(true);
    chassis.turnToHeading(140, 3000);
    chassis.turnToHeading(180, 1000, {.maxSpeed = max_speed1});
    mogo_rush.set_value(false);

    chassis.moveToPoint(-62, -60, 1000, {.maxSpeed = max_speed});
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.moveToPoint(-60, -40, 1000, {.forwards = false, .maxSpeed = max_speed});
    intake.move(0);
    chassis.waitUntilDone();
    mogo_clamp.set_value(false);
    // return;

    chassis.moveToPoint(-60, -55, 2000, {.maxSpeed = max_speed});
    chassis.moveToPoint(-29, -25, 2000, {.forwards = false, .maxSpeed = max_speed});
    chassis.waitUntilDone();
    mogo_clamp.set_value(true);

    intake.move(127);
    chassis.moveToPoint(-24, -12, 1000, {.maxSpeed = max_speed});
    chassis.waitUntilDone();
    intake.move(0);
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
    chassis.waitUntil(6);
    intake.move(127);
    chassis.turnToHeading(270, 1000,{.maxSpeed=max_speed1});
    pros::Task([=] {
        while(intake_dist.get() > 20){
            pros::delay(10);
        }
        pros::delay(300);
        intake.move(0);
    });
    chassis.turnToHeading(0, 1000,{.maxSpeed=max_speed1}); 
    mogo_rush.extend();

    chassis.turnToPoint(24, -25, 1000,{.forwards=false,.maxSpeed=max_speed1});
    chassis.moveToPoint(20, -25, 1000,{.forwards=false,.maxSpeed=max_speed});
    mogo_rush.retract();
    chassis.waitUntil(20);
    mogo_clamp.set_value(true);
    chassis.turnToPoint(72, -72, 1000,{.maxSpeed = max_speed1});
    chassis.moveToPoint(59, -59, 4000,{.maxSpeed=max_speed});
    intake.move(127);
    intake_lift.extend();
    lift.move(127);
    pros::delay(500);
    lift.move(0);
    /*mogo_rush.extend();
    chassis.turnToHeading(45, 2000,{.maxSpeed=max_speed1});
    chassis.moveToPose(48, -24, 0, 2000,{.maxSpeed = max_speed});
    /*chassis.moveToPoint(36, -24, 1000,{.maxSpeed=max_speed});
    mogo_clamp.extend();
    chassis.turnToHeading(270, 1000,{.maxSpeed=max_speed1});
    chassis.moveToPoint(2, -24, 1000,{.forwards=false, .maxSpeed=max_speed});
    chassis.waitUntilDone();
    mogo_clamp.retract();//*/


}
void auton_15s_far_driver_qual() //Blue
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

    /*chassis.setPose(-24, -60.75, 180);
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

    chassis.setPose(-62, -62, 45);
    chassis.moveToPoint(-48, -48, 2000, {.maxSpeed = max_speed});
    chassis.waitUntilDone();
    intake.move(0);
    chassis.turnToHeading(0, 1000, {.maxSpeed = max_speed1});
    setBasket(false);
    chassis.waitUntilDone();

    chassis.setPose(-72 + findDistToWall(1), -72 + findDistToWall(2), findHeading(1, 0));

    chassis.moveToPoint(-48, 0, 2000, {.maxSpeed = max_speed});
    moveLift(540);
    chassis.waitUntilDone();
    intake.move(127);
    chassis.turnToHeading(270, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = max_speed1});

    chassis.moveToPoint(-64, 0, 2000, {.maxSpeed = max_speed});
    pros::Task([=] {
        while(intake_dist.get() > 20){
            pros::delay(10);
        }
        intake.move(0);
    });
    chassis.moveToPoint(-58, 0, 1000, {.forwards = false, .maxSpeed = 20});
    moveLift(240);
    chassis.moveToPoint(-48, 0, 1000, {.forwards = false, .maxSpeed = max_speed});
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);*/

    //setBasket(true);
    //basketRings();
    /*chassis.turnToPoint(-48, 24, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = max_speed1});
    intake.move(127);
    chassis.moveToPoint(-48, 24, 3000, {.maxSpeed = max_speed});
    chassis.waitUntilDone();
    //basketRings();
    chassis.moveTPoint(-24, -36, 2000, {.maxSpeed = max_speed});
    intake.move(0);
    chassis.turnToHeading(180, 2000, {.maxSpeed = max_speed1});
    chassis.moveToPoint(-25, 56, 3000, {.forwards = false, .maxSpeed = max_speed});
    chassis.waitUntil(26);
    mogo_clamp.set_value(true);*/

    /*chassis.moveToPoint(-36, 58, 2000, {.maxSpeed = max_speed});
    mogo_rush.extend();
    chassis.turnToHeading(90, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-57, 56, 2000, {.forwards = false, .maxSpeed = max_speed});
    mogo_rush.retract();
    mogo_clamp.set_value(false);
    */

    chassis.setPose(-57, 56, 90);

    moveLift(300);
    chassis.moveToPoint(-48, 56, 2000, {.maxSpeed = max_speed});
    chassis.waitUntilDone();
    printf("Left Distance: %f", findDistToWall(1));
    printf("Back Distance: %f", findDistToWall(2));
    chassis.setPose(-72 + findDistToWall(2), 72 - findDistToWall(1), findHeading(1, 90));
    pros::delay(1000);
    chassis.moveToPoint(-12, 56, 2000, {.maxSpeed = max_speed});
    chassis.turnToHeading(315, 2000, {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = max_speed1});
    chassis.moveToPoint(0, 48, 2000, {.forwards = false, .maxSpeed = max_speed});
    chassis.waitUntilDone();
    mogo_clamp.set_value(true);
    chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
    chassis.moveToPoint(0, 54, 2000, {.maxSpeed = max_speed});
    
    pros::delay(5);
} // end auton_60s_skills_1()
// Auton skills number 2 fully done
void auton_60s_skills_2()	
{	
	
}





#include "auton_routines.h"
#include "auton_basics.h"
#include "auton_menu.h"
#include "controls.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "liblvgl/draw/lv_draw_label.h"
#include "liblvgl/draw/lv_img_buf.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include <cstdio>
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#include <type_traits>
#include <variant>
#include "hal.h"

ASSET(matchloadturn4ball_txt);
ASSET(touchbar_txt);

// Auton routine start positions
auton_routine null_routine{0, 0, 0, "None - Invalid Routine", nullptr};
auton_routine blue_goal_rush{1.234, -1.234, 90, "15S Auton - Near Driver # 1",
                             &goal_rush};
auton_routine blue_ring_rush{1.000, -1.300, 190, "15S Auton - Near Driver # 2",
                             &ring_rush};
auton_routine blue_neg_awp{1.000, -1.300, 190, "15S Auton - Near Driver # 3", &blue_negative_four};
auton_routine near_driver_elim{1.000, -1.300, 190,
                               "15S Auton - Near Driver # 2",
                               &DescoreRushElim}; // to be updated
auton_routine near_driver_elim2{1.000, -1.300, 190,
                                "15S Auton - Near Driver # 2",
                                &rushelim}; // to be updated

auton_routine rush_wp_a{1.000, -1.300, 190,
                                "15S Auton - Near Driver # 2",
                                &rushWP}; // to be updated

auton_routine safe_positive{1.234, -1.234, 90,
                                   "15S Auton - Near Driver # 1",
                                   &safePos};
auton_routine negative_four{1.234, -1.234, 90,
                                   "15S Auton - Near Driver # 1",
                                   &negativeFour};
auton_routine far_from_driver_elim2{-0.600, 0.600, 180, "extra_1",
                                    &safe_6_ball};

auton_routine skills_1{-0000, 0.000, 00, "60S Auton - Skills # 1",
                       &auton_60s_skills_1};
auton_routine skills_2{-0.600, 0.600, 180, "60S Auton - Skills # 2",
                       &auton_60s_skills_2};

float max_speed = 60;            // forward back max speed
float mid_speed = 60;           //forward medium speed
int max_speed1 = (int)max_speed; // turn max speed

void printPosition(char *msg, bool withDistanceSensors = false,
                   bool detailedDist = false) {
  lemlib::Pose currentPose = chassis.getPose();
  printf("%s\tX: %3.2f\tY: %3.2f\tTheta: %3.2f\n", msg, currentPose.x,
         currentPose.y, currentPose.theta);
  if (withDistanceSensors) {
    if (detailedDist) {
      printf("\nRF: %d\tRB: %d\tLF: %d\tLB: %d\tBL: %d\tBR: "
             "%d\tFront: %d\n",
             distance_rf.get(), distance_rb.get(), distance_lf.get(),
             distance_lb.get(), distance_bl.get(), distance_br.get(),
             distance_front.get());
    }
    printf("\nRight: %3.2f\tLeft: %3.2f\tBack:%3.2f\tFront:%3.2f\n",
           findDistToWall(0), findDistToWall(1), findDistToWall(2),
           findDistToWall(3));
  }
}

// FULLY DONE
void auton_15s_near_driver_qual() // DONE
{}
void goal_rush() // BLUE
{

  chassis.setPose(-48, -51.5, 180);
  liftIntake();
  chassis.moveToPoint(-48, -24, 3000, {.forwards = false});
  chassis.waitUntil(22);
  closeClamp();
  dropIntake();
  pros::delay(250);
  openClamp();
  chassis.waitUntilDone();
  chassis.moveToPoint(-48, -7.5, 2000, {.forwards = false,});
  chassis.waitUntil(14);
  closeClamp();
  pros::delay(200);

  
}

void rushBlue() {

  spinIntake(-127);
  pros::delay(250);
  spinIntake(0);

  setBasket(false);

  pros::delay(200);

  basketRings();
  pros::delay(100);
  spinIntake(127);

  double newHeading = findHeading(2, 180);
  chassis.setPose(72 - fabs(findDistToWall(1) * cos(lemlib::degToRad(newHeading))), 72 - fabs(findDistToWall(2) * cos(lemlib::degToRad(newHeading))), newHeading);
  printPosition((char *)"Starting position", true, true);
  pros::delay(300);
  lemlib::Pose currentPose = chassis.getPose();
  chassis.moveToPose(0, 49, 270, 2000, {.lead = 0.2, .maxSpeed = 70 + 20});
  spinIntake(-127);
  chassis.waitUntilDone();
  pros::delay(200);
  spinIntake(0);
  printPosition((char *)"First Point", true, true);
  pros::delay(100);
  setBasket(true);
  moveLiftToPos(350);
  newHeading = findHeading(0, 270);
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, 72 - fabs(findDistToWall(0) * cos(lemlib::degToRad(270 - newHeading))), newHeading);
  printPosition((char *)"Reset position", true, true);
  pros::delay(300);
  //chassis.moveToPoint(24, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  printPosition((char *)"TEST", true, true);
//return;

  //return;
  pros::delay(500);
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  printPosition((char *)"TEST2", true, true);
  dropIntake();
  //chassis.waitUntilDone();
  pros::delay(250);
  chassis.moveToPoint(0, 56.5, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  printPosition((char *)"TEST3", true, true);
  moveLiftToPos(100);


  //chassis.setPose(0, -58, 180);
  setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
  chassis.moveToPoint(0, 48, 2000, {.forwards = false, .maxSpeed = max_speed});
  //chassis.movetoPoint(0, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  liftIntake();
  chassis.turnToHeading(-45, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.moveToPoint(30, 21, 2000, {.forwards = false, .maxSpeed = max_speed + 15});
  chassis.waitUntilDone();
  closeClamp();
  pros::delay(200);
  spinIntake(127);
  pros::delay(250);
  chassis.turnToHeading(90, 2000);
  chassis.waitUntilDone();
  chassis.moveToPoint(56, 28, 2000);
  chassis.waitUntilDone();
  //chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});

  chassis.moveToPoint(14, 14, 2000, {.forwards = false});
  chassis.waitUntil(36);
  spinIntake(0);

}


void rushRed() { // RED

  spinIntake(-127);
  pros::delay(250);
  spinIntake(0);

  setBasket(false);

  pros::delay(300);

  basketRings();
  pros::delay(100);
  spinIntake(127);
  

  float myDist1 = fabs(distance_rb.get() - distance_rf.get()) > 400.0
                     ? fabs(distance_rb.get() - 1300.0) < 250.0
                           ? distance_rb.get()
                           : distance_rf.get()
                     : (distance_rb.get() + distance_rf.get()) / 2.0;
  myDist1 = myDist1 / 25.4 + 6.75;

  float myX = 72 - myDist1;
  printf("\n\n\n MYy: %3.2f\n\n\n", myX);
  
  //pros::Task basketRingsTask(basketRingsAsTask);
  float failsafe_x = (fabs(24.0 - (71 - (distance_rb.get() / 25.4 + 6.75))) > fabs(24.0 - (71 - ((distance_rf.get() - RIGHT_DIFFERENCE) / 25.4 + 6.75)))) ? 
      (71 - ((distance_rf.get() - RIGHT_DIFFERENCE) / 25.4 + 6.75)) : 
      (71 - (distance_rb.get() / 25.4 + 6.75));
  chassis.setPose(failsafe_x, findDistToWall(2) - 72, findHeading(2, 0));

  printPosition((char *)"Starting position", true, true);
  pros::delay(300);
  lemlib::Pose currentPose = chassis.getPose();
  chassis.moveToPose(4, -49, 270, 2000, {.maxSpeed = 70});
  spinIntake(-127);
  chassis.waitUntilDone();
  pros::delay(200);
  printPosition((char *)"Back Position", true, true);
  pros::delay(100);
  setBasket(true);
  moveLiftToPos(310);
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, findDistToWall(1) - 72, findHeading(1, 270));
  chassis.waitUntilDone();
  printPosition((char *)"Reset position", true, true);
  pros::delay(400);
  chassis.moveToPoint(2.5, -48, 2000, {.maxSpeed = max_speed});
  //chassis.moveToPoint(24, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  printPosition((char *)"TEST", true, true);
//return;

  //return;
  pros::delay(700);
  chassis.turnToHeading(180, 2000, {.maxSpeed = max_speed1});
  spinIntake(0);
  chassis.waitUntilDone();
  printPosition((char *)"Towards stake", true, true);
  dropIntake();
  pros::delay(250);
  chassis.moveToPoint(2.5, -59, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  printPosition((char *)"At stake", true, true);
  moveLiftToPos(100);


  //chassis.setPose(0, -58, 180);
  setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
  chassis.moveToPoint(2.5, -48, 2000, {.forwards = false, .maxSpeed = max_speed});
  //chassis.movetoPoint(0, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  liftIntake();
  chassis.turnToHeading(225, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.moveToPoint(24, -24, 2000, {.forwards = false, .maxSpeed = max_speed + 15});
  chassis.waitUntilDone();
  closeClamp();
  pros::delay(200);
  spinIntake(127);
  pros::delay(250);
  chassis.turnToHeading(90, 2000);
  chassis.waitUntilDone();
  chassis.moveToPoint(52, -24, 2000);
  chassis.waitUntilDone();
  pros::delay(900);
  spinIntake(0);
  //chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});

  chassis.moveToPoint(14, -14, 2000, {.forwards = false});

}


void  rushWP(){
  bool isRed = false;  //TODO SWITCH

  if(isRed){
    rushRed();
  }else{
    rushBlue();
  }

}

void safeBlue() {
    spinIntake(-127);
    pros::delay(250);
    spinIntake(0);

    chassis.setPose(48, 48, findHeading(2, 180));
    chassis.turnToHeading(180, 1000, {.maxSpeed = max_speed1});
    chassis.waitUntilDone();

    float failsafe_x = (fabs(24.0 - (distance_lb.get() / 25.4 + 6.75))) > fabs(24.0 - ((distance_lf.get() - LEFT_DIFFERENCE) / 25.4 + 6.75)) ? 
      (71 - ((distance_lf.get() - LEFT_DIFFERENCE) / 25.4 + 6.75)) : 
      (71 - (distance_lb.get() / 25.4 + 6.75));
    chassis.setPose(failsafe_x, 72 - findDistToWall(2), findHeading(2, 180));

    printPosition((char *)"Starting position", true, true);
    pros::delay(500);

    chassis.moveToPoint(48, 48, 1500, {.maxSpeed = max_speed});
    pros::delay(500);

    chassis.turnToPoint(24, 24, 1500, {.forwards = false, .maxSpeed = max_speed1});
    chassis.moveToPoint(20, 20, 2000, {.forwards = false, .maxSpeed = max_speed});
    chassis.waitUntilDone();
    closeClamp();
    pros::delay(250);
    spinIntake(127);
    pros::delay(250);
    chassis.moveToPoint(48, 24, 2000);
    pros::delay(1000);
    chassis.moveToPoint(36, 24, 1500, {.forwards = false, .maxSpeed = max_speed});
    chassis.waitUntilDone();
    pros::delay(500);
    //moveLiftToPos(400);

    chassis.turnToPoint(0, 0, 1500, {.maxSpeed = max_speed1});
    spinIntake(0);
    chassis.moveToPoint(14, 14, 2000, {.maxSpeed = max_speed});

    pros::delay(500);
}

void safePos() {
  bool isRed = false;

  if(isRed){

  } else {
    safeBlue();
  }
}


void rushelim() { printf("%s(): Exiting\n", __func__); }

void ring_rush() // BLUE
{

  chassis.setPose(48, -52.25, 0);
  chassis.moveToPose(50, -10, 15, 3000, {.maxSpeed = max_speed});
  spinIntake(-127);
  pros::delay(300);
  spinIntake(0);
  moveLift(127);
  pros::delay(400);
  moveLift(0);
  spinIntake(127);
  closeClamp();
  chassis.waitUntilDone();
  chassis.moveToPoint(48, -17, 1000, {.forwards = false, .maxSpeed = max_speed});
  spinIntake(0);
  chassis.turnToPoint(46, 0, 1000, {.maxSpeed = max_speed1});
  chassis.moveToPoint(46, -8, 2000, {.maxSpeed = max_speed});
  spinIntake(127);
  chassis.turnToHeading(270, 1000, {.maxSpeed = max_speed1});
  chassis.waitUntil(3);
  spinIntake(0);
  chassis.waitUntil(6);
  extendSweep();
  chassis.waitUntilDone();
  spinIntake(0);
  chassis.turnToHeading(0, 1000, {.maxSpeed = max_speed1});
  chassis.turnToPoint(24, -24, 1000,
                      {.forwards = false, .maxSpeed = max_speed1});
  chassis.moveToPoint(20, -2, 1000, {.forwards = false, .maxSpeed = max_speed});
  retractSweep();
  chassis.waitUntil(18);
  closeClamp();
  /*chassis.moveToPoint(36, -24, 1000,{.maxSpeed=max_speed});
  openClamp();
  chassis.turnToHeading(270, 1000,{.maxSpeed=max_speed1});
  chassis.moveToPoint(2, -24, 1000,{.forwards=false, .maxSpeed=max_speed});
  chassis.waitUntilDone();
  closeClamp();//*/
}

void blue_negative_four()
{

  
  chassis.setPose(-24, 51.5, 0);
  lemlib::Pose currentPose = chassis.getPose();
  pros::delay(200);
  double newHeading = findHeading(1, 0);
  chassis.setPose(fabs(findDistToWall(1) * cos(lemlib::degToRad(newHeading))) - 72, currentPose.y, newHeading);
  printPosition((char *)"Starting position", true, true);
  pros::delay(200);
  chassis.moveToPoint(-26.5, 18, 2000, {.forwards = false, .maxSpeed = max_speed + 20});
  spinIntake(-127);
  pros::delay(300);
  spinIntake(0);
  chassis.waitUntil(26);
  closeClamp();
  chassis.waitUntilDone();
  printPosition((char *)"Grabbing Goal", true, true);

  pros::delay(700);
  spinIntake(127);
  chassis.turnToHeading(-90, 1000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.moveToPoint(-43, 24, 2000, {.maxSpeed = max_speed + 20});
  chassis.waitUntilDone();
  printPosition((char *)"First Ring", true, true);


  chassis.turnToHeading(180, 1000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  newHeading = findHeading(0, 180);
  chassis.setPose(fabs(findDistToWall(0) * cos(lemlib::degToRad(newHeading))) - 72, 72 - fabs(findDistToWall(2) * cos(lemlib::degToRad(newHeading))), newHeading);
  printPosition((char *)"After reset", true, true);

  pros::delay(200);
  chassis.turnToHeading(165, 2000, {.maxSpeed = max_speed1});
  chassis.moveToPose(-42, 8, 180, 2000, {.maxSpeed = max_speed + 20});
  chassis.waitUntilDone();
  printPosition((char *)"Second Ring", true, true);
  pros::delay(200);
  chassis.moveToPoint(-48, 24, 2000, {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.turnToHeading(185, 2000, {.maxSpeed = max_speed1});
  chassis.moveToPoint(-50, 8, 2000, {.maxSpeed = max_speed + 20});
  chassis.waitUntilDone();
  printPosition((char *)"Third Ring", true, true);
  pros::delay(200);
  chassis.moveToPoint(-48, 18, 2000, {.forwards = false, .maxSpeed = max_speed});
  chassis.turnToHeading(90, 1000, {.maxSpeed = max_speed1});
  chassis.moveToPoint(-14, 14, 1500, {.maxSpeed = max_speed});
  
}


void negativeFour()
{
  bool isRed = false;

  if(isRed){

  } else {
    blue_negative_four();
  }
}

void safe_6_ball() // FAR ELIM
{}

void auton_15s_far_driver_elim() { printf("%s(): Exiting\n", __func__); }
void DescoreRushElim() { printf("%s(): Exiting\n", __func__); }
// STILL WORKING ON IT


ASSET(skillsPathPart1_txt);
void auton_60s_skills_1() {

  printf("\n\n\nskills\n\n\n");

  //pros::delay(1500);
  //printf("\nBattery: %3.2f\n", pros::c::battery_get_capacity());







  chassis.setPose(0, 0, 0);
  outakeFor(127, 200);
  pros::delay(600);
  //chassis.moveToPoint(0, -8, 4000, {.forwards = false, .maxSpeed = mid_speed, .minSpeed = 50, .earlyExitRange = 1});
  chassis.moveToPoint(0, -18, 4000, {.forwards = false, .maxSpeed = mid_speed - 15});
  chassis.waitUntilDone();
  pros::delay(500);
  closeClamp(); //clamp on mobile goal
  pros::delay(500);
  chassis.turnToHeading(-135, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  lemlib::Pose currentPose = chassis.getPose(); 
  chassis.setPose(currentPose.x, currentPose.y, findHeading(1, 0));
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.setPose(findDistToWall(1) - 72, findDistToWall(2) - 72, findHeading(1, 0.0));
  currentPose = chassis.getPose();
  if(fabs(currentPose.theta) > 25.0){
    pros::c::delay(300);
      chassis.setPose(findDistToWall(1) - 72, findDistToWall(2) - 72, findHeading(1, 0.0));
      printf("\nBad 1st reset \n");
  }
  printPosition((char *)"Starting position", true, true); // after grabbing goal
  spinIntake(0);
  pros::delay(300);
  //chassis.moveToPoint(-24, -48, 1000, {.forwards = false, .maxSpeed = max_speed}); //Move towards first ring 
  //chassis.waitUntil(10);
  //pros::delay(350);
  spinIntake(127);
  //intake.move(0); //stop intake to avoid outaking preload 
  pros::Task lift_up1([=] { //raise lift (later lift may get in way near the wall) 
  moveLiftToPos(300);
  });

  pros::delay(250);

  //intake other rings in bottom left corner
  chassis.moveToPoint(-24, -26, 2000, {.maxSpeed = mid_speed}); //grabs first ring
  chassis.waitUntilDone();
  pros::delay(100);
  chassis.turnToHeading(-100, 2000, {.maxSpeed = 40});
  chassis.moveToPoint(-52, -26, 2000, {.maxSpeed = mid_speed}); //second ring
  chassis.waitUntilDone();
  chassis.moveToPoint(-48, -28, 2000, {.forwards = false, .maxSpeed = mid_speed}); //backs up from second ring
  pros::delay(250);
  chassis.turnToHeading(180, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.moveToPoint(-48, -48, 2000, {.maxSpeed = max_speed-15}); //3rd ring
  chassis.waitUntilDone();
  pros::delay(1250);
  chassis.moveToPoint(-48, -58, 2000, {.maxSpeed = max_speed - 10}); //4th ring
  chassis.waitUntilDone();
  chassis.moveToPoint(-38,-50,2000, {.forwards = false, .maxSpeed = max_speed}); //backs up before last (basket) ring
  chassis.waitUntilDone();
  pros::delay(50);
  currentPose = chassis.getPose();
  chassis.turnToHeading(270, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  pros::delay(150);
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, currentPose.y, findHeading(1, 270));
  pros::delay(250);
  /*pros::delay(150);

  currentPose = chassis.getPose();

  float myDist1 = fabs(distance_lb.get() - distance_lf.get()) > 400.0
                     ? fabs(distance_lb.get() - 1700.0) < 250.0
                           ? distance_lb.get()
                           : distance_lf.get()
                     : (distance_lb.get() + distance_lf.get()) / 2.0;
  myDist1 = myDist1 / 25.4 + 6.75;
  float myY = myDist1 - 72.0;

  float myX_skills = fabs(findDistToWall(3) - 24.0) > 10.0 ? currentPose.y : findDistToWall(3) - 72;

  chassis.setPose(myX_skills, myY, findHeading(1, 270));*/
  printPosition((char *)"Before last ring", true, true);
  pros::delay(200);
  chassis.moveToPoint(-62, -50, 2000, {.maxSpeed = mid_speed});
  chassis.waitUntilDone();
  spinIntake(0);//save last intaked ring for basket
  pros::delay(1000);
  spinIntake(0);
  // chassis.moveToPoint(-40, -48, 1000, {.forwards = false, .maxSpeed = 40}); 
  // chassis.waitUntilDone();
  // chassis.moveToPoint(-60, -52, 2000, {.maxSpeed = max_speed});

  //move to place goal in corner
  chassis.turnToHeading(0, 1000, {.maxSpeed = max_speed1});
  pros::delay(80);
  chassis.moveToPoint(-63, -65, 1000, {.forwards = false, .maxSpeed = max_speed}); 
  pros::Task lift_down1([=] { moveLiftToPos(60); setBasket(true);});
  chassis.waitUntilDone();
  printPosition((char *)"at goal drop");
  openClamp();
  
  setBasket(false);
  pros::delay(300);

  //reset & move to wall stake
  chassis.moveToPoint(-45, -45, 2000);
  pros::Task basketRingstask(basketRingsAsTask);
  pros::delay(1500);
  chassis.turnToHeading(0, 1000, {.maxSpeed = 40});
  chassis.waitUntilDone();
  pros::delay(1000);
  currentPose = chassis.getPose();
  printPosition((char *)"(-48,-48) reset point before sensors", true, true);
  chassis.setPose(currentPose.x, currentPose.y, findHeading(1, 0));
  chassis.waitUntilDone();
  pros::delay(50);
  printPosition((char *)"(-48,-48) angle through sensors", true, true);
  pros::delay(50);
  chassis.turnToHeading(0, 2000);
  chassis.waitUntilDone();
  pros::delay(50);
  printPosition((char *)"(-48,-48) final angle", true, true);
  pros::delay(50);
  currentPose = chassis.getPose();
  chassis.setPose(findDistToWall(1) - 72, findDistToWall(2) - 72, currentPose.theta);
  pros::delay(50);
  // chassis.moveToPoint(-48, -48, 2000, {.forwards = false, .maxSpeed = max_speed});
  // chassis.waitUntilDone();
  // chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  // chassis.waitUntilDone();
  //pros::delay(100);
  printPosition((char *)"turn coords", true, true);
  pros::delay(100);
  printPosition((char *)"(-48,-48) final distance", true, true);
  chassis.moveToPoint(-48, -4, 2000, {.maxSpeed = max_speed-5});
  setBasket(true);
  moveLiftToPos(540);
  chassis.waitUntilDone();
    printPosition((char *)"(-48, 0 distance)", true, true);



// return;
  //chassis.setPose(-72 + findDistToWall(1), -72 + findDistToWall(2), findHeading(1, 0));

  printPosition((char *)"basket lineup", true, true);
  spinIntake(0);
  pros::Task saveringsdsfds(saveRingsAsTask);
  chassis.turnToHeading(270, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = max_speed1});

  chassis.moveToPoint(-68, -1, 2000, {.maxSpeed = max_speed});
  printPosition((char *)"basket scoring", true, true);
  chassis.waitUntilDone();
  spinIntake(0);
  moveLiftToPos(240);
  pros::delay(200);
  chassis.moveToPoint(-61, 0, 1000, {.forwards = false, .maxSpeed = 20});
  printPosition((char *)"basket backup", true, true);
  setBasket(false);
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.moveToPoint(-48, 0, 1000, {.forwards = false, .maxSpeed = max_speed});
  setLiftBrake(pros::E_MOTOR_BRAKE_COAST);

  // setBasket(false);
  // basketRings();

/* skipping for time - 2nd alliance basket

  //2nd corner
  chassis.turnToPoint(-48, 24, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = max_speed1 - 10});

  pros::Task basketRingstask2(basketRingsAsTask);
  chassis.waitUntilDone();


  currentPose = chassis.getPose();
  chassis.setPose(findDistToWall(1) - 72, currentPose.y, findHeading(1, 0));
  printPosition((char *)"2nd corner transition", true, true);
  pros::delay(250);
  chassis.moveToPoint(-48, 24, 3000, {.maxSpeed = mid_speed});
  pros::Task basketRingstask3(basketRingsAsTask);
  chassis.waitUntilDone();
  printPosition((char *)"2nd corner 1st ring", true, true);
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
//end of basketing left side rings

*/

  chassis.waitUntilDone();
  pros::delay(1000);
  chassis.moveToPoint(-48, -45, 2000, {.forwards = false, .maxSpeed = max_speed+10});
  pros::Task basketRingstask2(basketRingsAsTask15);
  chassis.waitUntilDone();

  


  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, currentPose.y, findHeading(2, 0));
  pros::delay(100);
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1, .earlyExitRange = 0.5});
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  pros::delay(100);
  chassis.setPose(findDistToWall(1) - 74, findDistToWall(2) - 72, findHeading(1, 0.0)); //reset after ring after first pole
  printPosition((char *)"post -48 -48 coming back reset", true, true);
  pros::delay(100);
  chassis.moveToPoint(-48, -45, 2000, {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntilDone();
  chassis.turnToHeading(90, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.setPose(findDistToWall(2) - 73, findDistToWall(0) - 72, findHeading(2, 90.0)); //reset after coming back turn CHECK AT MAINE
  pros::delay(100);
  chassis.moveToPoint(-1.5, -48, 2000, {.maxSpeed = mid_speed});
  chassis.waitUntilDone();
  spinIntake(0);
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, currentPose.y, findHeading(0, 90)); //reset angle at red stake
  printPosition((char *)"Alliance Angle reset", true, true);
  chassis.waitUntilDone();
 //chassis.turnToHeading(90, 2000, {.maxSpeed = max_speed1, .earlyExitRange = 0.5});
  //chassis.waitUntilDone();
  currentPose = chassis.getPose();
  //chassis.setPose(currentPose.x, findDistToWall(0) - 72, currentPose.theta);
  printPosition((char *)"Alliance reset", true, true);
  chassis.waitUntilDone();
  setBasket(true); //close hood
   moveLiftToPos(350);
  // chassis.waitUntilDone();
  chassis.turnToHeading(180, 2000, {.maxSpeed = max_speed1});
  spinIntake(127);
  chassis.waitUntilDone();
  //currentPose = chassis.getPose();
  chassis.moveToPoint(-1.5, -52, 2000, {.maxSpeed = mid_speed});
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, findDistToWall(3) - 72, currentPose.theta);
  chassis.moveToPoint(-1.5, -57, 2000, {.maxSpeed = mid_speed});
  chassis.waitUntilDone();
  moveLiftToPos(100);
  printPosition((char *)"Alliance Score", true, true);
  spinIntake(127);
  pros::delay(250);
  chassis.moveToPoint(0, -48, 2000, {.forwards = false, .maxSpeed = max_speed}); //back out of stake
  spinIntake(0);
  chassis.turnToHeading(270, 2000);
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, findDistToWall(1) - 72, findHeading(1, 270));
  chassis.moveToPoint(28, -48, 2000, {.forwards = false, .maxSpeed = max_speed}); //grrab goal
  chassis.waitUntil(21);
  closeClamp();
  pros::delay(250);
  printPosition((char *)"Goal Grab", true, true);



   //2nd corner

  pros::delay(750);
  spinIntake(127);
  chassis.setPose(72 - findDistToWall(2) , findDistToWall(1) - 72, findHeading(1, 270));
  printPosition((char *)"Post Goal Grab", true, true);
  pros::delay(100);
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.moveToPoint(24, -24, 2000, {.maxSpeed = max_speed}); //first ring
  chassis.waitUntilDone();
  printPosition((char *)"1", true, true);
  chassis.turnToHeading(90, 2000, {.maxSpeed = max_speed1});
  chassis.moveToPoint(48, -24, 2000, {.maxSpeed = max_speed});//second ring
  chassis.waitUntilDone();
  printPosition((char *)"2", true, true);
  chassis.turnToHeading(180, 2000, {.maxSpeed = max_speed1}); 
  chassis.waitUntilDone();
  chassis.moveToPoint(48, -45, 2000, {.maxSpeed = max_speed}); //third ring
  moveLiftToPos(310);
  chassis.waitUntilDone();
  pros::delay(350);
  printPosition((char *)"3", true, true);
  pros::delay(150);
  chassis.moveToPoint(48, -58, 2000, {.maxSpeed = max_speed}); 
  printPosition((char *)"4", true, true);
  chassis.waitUntilDone();
  chassis.moveToPoint(38, -48, 2000, {.forwards = false, .maxSpeed = 65});
  chassis.waitUntilDone();
  pros::delay(250);
  chassis.turnToHeading(90, 1500, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.moveToPoint(55, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  pros::delay(350);
  printPosition((char *)"5", true, true);
  chassis.moveToPoint(48, -48, 2000, {.forwards = false, .maxSpeed = 65});
  chassis.turnToHeading(270, 2000);
  chassis.waitUntilDone();
  spinIntake(0);
  openClamp();
  chassis.moveToPoint(63, -63, 500, {.forwards = false, .maxSpeed = 127});
  chassis.moveToPoint(48, -48, 2000, {.maxSpeed = max_speed});

  /*
  chassis.turnToHeading(135, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.moveToPoint(40, -48, 2000, {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntilDone();
  chassis.turnToHeading(90, 2000, {.maxSpeed = 65});
  chassis.waitUntilDone();
  chassis.moveToPoint(62, -48, 2000, {.maxSpeed = max_speed});
  printPosition((char *)"5 (saved)", true, true);
  chassis.waitUntilDone();
  intake.move(0);
  pros::delay(350);
  chassis.moveToPoint(48, -48, 2000, {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntilDone();
  printPosition((char *)"pre drop backout", true, true);
  chassis.turnToHeading(-45, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  intake.move(0); //save ring
  chassis.moveToPoint(65, -65, 2000, {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntilDone();
  printPosition((char *)"drop", true, true);
  openClamp();
  chassis.moveToPoint(42, -42, 2000, {.maxSpeed = max_speed});
  printPosition((char *)"post drop backout", true, true);
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();

  
*/
/*


  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, currentPose.y, findHeading(0, 0));
  pros::delay(100);
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1, .earlyExitRange = 0.5});
  chassis.setPose(72 - findDistToWall(0), findDistToWall(2) - 72, currentPose.theta);
  pros::delay(100);
  printPosition((char *)"final reset", true, true);
  setBasket(false); //open hood
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  pros::c::delay(500);
  basketRings(true);
  pros::c::delay(1000);
  //intake.move(127);
  chassis.moveToPoint(42, 0, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  chassis.turnToHeading(90, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();



  setBasket(false); //basket at wall stake
  intake.move(127);
/* took out 2nd basket at wall stake for time
  chassis.moveToPoint(53.5, 0, 2000, {.maxSpeed = max_speed});
  pros::Task basketRingstaskabggc(basketRingsAsTask);
  chassis.waitUntilDone();
  pros::delay(750);
  chassis.moveToPoint(44.0, 0, 2000, {.forwards = false, .maxSpeed = max_speed});
  */
 // pros::delay(1000);

 /*
  setBasket(true);
  pros::delay(500);
  moveLiftToPos(520);
  currentPose = chassis.getPose();
  chassis.setPose(72 - findDistToWall(3), currentPose.y, currentPose.theta);
  chassis.moveToPoint(60, 0, 2000, {.maxSpeed = max_speed});
  pros::delay(250);
  chassis.waitUntilDone();
  moveLiftToPos(210);
  chassis.moveToPoint(48, 0, 2000);
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  saveRings();
  

*/

  return;



  chassis.moveToPoint(-20, 36, 3000, {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntilDone();
  setBasket(true);
  chassis.moveToPoint(-20, 62, 3000, {.forwards = false, .maxSpeed = max_speed});
  chassis.turnToHeading(90, 2000, {.maxSpeed = max_speed1});
  //chassis.moveToPoint(-24, 65, 1000, {.forwards = false, .maxSpeed = max_speed});
  //chassis.turnToHeading(90, 2000, {.maxSpeed = max_speed1});
  chassis.moveToPoint(-70, 60, 2000, {.forwards = false});
  intake.move(0);






  // Less efficient goal scoring
  // setBasket(true);
  // chassis.waitUntil(41);
  // closeClamp();
  // spinIntake(-90);
  // pros::delay(500);

  // chassis.turnToPoint(-65, 62, 2000, {.maxSpeed = max_speed1});
  // chassis.moveToPoint(-65, 62, 2000, {.maxSpeed = max_speed});
  // extendSweep();
  // chassis.waitUntilDone();
  // printPosition((char *)"Before sweep", true, true);
  // spinIntake(0);
  // chassis.turnToHeading(135, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
  // chassis.waitUntil(90);
  // retractSweep();
  // chassis.waitUntilDone();
  // printPosition((char *)"After sweep", true, true);
  // chassis.moveToPoint(-65, 65, 2000, {.forwards = false, .maxSpeed = max_speed}); 
  // openClamp();

    moveLiftToPos(350);
    chassis.moveToPoint(-48, 48, 2000, {.maxSpeed = max_speed});
    chassis.turnToHeading(270, 1000, {.maxSpeed = max_speed1});
    chassis.waitUntilDone();


    moveLiftToPos(350);
    pros::delay(2000);

    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, findHeading(0, 270));
    chassis.turnToHeading(270, 1000, {.maxSpeed = max_speed1});
    chassis.waitUntilDone();
    pros::delay(100);
    // chassis.setPose(-72 + findDistToWall(3), 71 - findDistToWall(0), findHeading(0, 270));
    printPosition((char *)"After 2nd corner reset", true, true);
    //chassis.setPose(0, 0, findHeading(0, 0));
    //pros::delay(500);
    // chassis.moveToPoint(-12, 56, 2000, {.maxSpeed = max_speed});
    // chassis.turnToHeading(-45, 2000, {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = max_speed1});

    //chassis.moveToPose(findDistToWall(0) - 21, findDistToWall(3) - 72, 0, 3000, {.forwards = false, .maxSpeed = max_speed});
    chassis.moveToPoint(2, 52, 2000, {.forwards = false, .maxSpeed = max_speed});
    
    currentPose = chassis.getPose(); 
    moveLiftToPos(350);
    setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
    //chassis.waitUntilDone();
    closeClamp();
    pros::delay(500);
    printPosition((char *)"After getting goal", true, true);
    chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
    chassis.waitUntilDone();
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, findHeading(0, 0));
    printPosition((char *)"goal reset", true, true);
    chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1, .earlyExitRange = 1});
    chassis.waitUntilDone();
    pros::delay(250);
    currentPose = chassis.getPose();
    chassis.setPose(findDistToWall(1) - 71, 72-findDistToWall(3), currentPose.theta);
    chassis.waitUntilDone();

    //pros::delay(2000);
    //chassis.setPose(0, 72-findDistToWall(3), currentPose.theta);
    //chassis.waitUntilDone();
    
    printPosition((char *)"Face wall stake", true, true);
    pros::delay(100);
    //chassis.waitUntilDone();
    chassis.moveToPoint(0, 56.5, 2000, {.maxSpeed = max_speed});
    setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntilDone();
    pros::delay(500);
    if(currentPose.theta > 180.0){
      chassis.turnToHeading(360.0, 500, {.earlyExitRange = 1});
    }else{
      chassis.turnToHeading(0.0, 500, {.earlyExitRange = 1});
    }
    chassis.waitUntilDone();
    moveLiftToPos(170);
    pros::delay(100);
    pros::Task lift_down2([=] {
      moveLift(-10);
      pros::delay(250);
      moveLift(0);
      setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
    });
    currentPose = chassis.getPose();
    chassis.moveToPoint(0, 50, 2000, {.forwards = false, .maxSpeed = max_speed}); chassis.getPose();


  
  float myDist2 = fabs(distance_rb.get() - distance_rf.get()) > 400.0
                     ? fabs(distance_rb.get() - 1700.0) < 250.0
                           ? distance_rb.get()
                           : distance_rf.get()
                     : (distance_rb.get() + distance_rf.get()) / 2.0;
  myDist2 = myDist2 / 25.4 + 6.75;

  float myX = myDist2 - 72.0;
  printf("\n\n\n MYy: %3.2f\n\n\n", myX);


    printPosition((char*)"Before pickup", true, true);


    closeClamp();

    chassis.setPose(myX, 74.0-findDistToWall(3), currentPose.theta);

    currentPose = chassis.getPose();

    printPosition((char*)"after pickup");


    pros::delay(250);
    chassis.turnToHeading(135, 2000, {.maxSpeed = max_speed1});
    chassis.waitUntilDone();

  printPosition((char*)"after turn", true, true);

  spinIntake(127);
  printPosition((char*)"intake started");
  // chassis.moveToPose(24, 24, 135, 2000);
  // chassis.moveToPoint(24, 24, 2000, {.maxSpeed = max_speed});

    //pros::c::delay(250);
    chassis.setPose(0.0, 0.0, 0.0);
    printPosition((char*)"magic");
    pros::c::delay(250);

    // chassis.moveToPoint(24, 24, 2000, {.maxSpeed = max_speed});

    setBasket(false);
    pros::Task basketRingstaskabc(basketRingsAsTask);

    chassis.moveToPoint(0, 36,2000, {.maxSpeed = max_speed});


    chassis.waitUntilDone();
    printPosition((char*)"first ring");

    pros::delay(50);

    // basket first ring
    chassis.turnToHeading(-45, 2000, {.maxSpeed = max_speed1, .earlyExitRange = 1});
    chassis.waitUntilDone();
    pros::delay(1500);
    printPosition((char*)"after turn");


  //     float myDist2 = fabs(distance_rb.get() - distance_rf.get()) > 200.0
  //                    ? fabs(distance_rb.get() - 1200.0) < 100.0
  //                          ? distance_rb.get()
  //                          : distance_rf.get()
  //                    : (distance_rb.get() + distance_rf.get()) / 2.0;
  // myDist1 = myDist1 / 25.4 + 6.75;


  // float myX2 = 72 - myDist1;

  // currentPose = chassis.getPose();

  // chassis.setPose(74.0 - findDistToWall(3), myX2, findHeading(1, 90), currentPose.theta);


  // pros::c::delay(100);
  // printPosition((char*)"after reset 1", true, true);
  currentPose = chassis.getPose();


  chassis.moveToPoint(-24, 57, 2000, {.maxSpeed = max_speed});
  chassis.waitUntil(10);
  setBasket(true);
  chassis.waitUntilDone();
  chassis.turnToHeading(-45, 2000);
  chassis.waitUntilDone();
  pros::delay(250);
  chassis.setPose(74.0 - findDistToWall(3), 74.0-findDistToWall(1), findHeading(1, 90)); 
  pros::c::delay(250);
  printPosition((char*)"after reset 2", true, true);
  chassis.turnToHeading(0, 2000);
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, currentPose.y, findHeading(0, 0));
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  printPosition((char*)"after heading reset", true, true);
  chassis.waitUntilDone();
  chassis.setPose(74.0 - findDistToWall(0), 74 - findDistToWall(3), currentPose.theta);
  printPosition((char*)"after full reset", true, true);


/* hard corner rings
  chassis.moveToPoint(60, 37, 2000, {.maxSpeed = max_speed});
  moveLiftToPos(300);
  chassis.waitUntilDone();
  printPosition((char*)"second ring");
  chassis.turnToHeading(-10, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  pros::delay(50);
  printPosition((char*)"after turn");

  pros::delay(50);
  chassis.moveToPoint(48, 50, 2000);
  chassis.waitUntilDone();
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  pros::c::delay(250);
  chassis.moveToPoint(48, 62, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(48, 24, 2000, {.forwards = false, .maxSpeed = max_speed}); 
    chassis.waitUntilDone(); chassis.turnToHeading(15, 2000, {.maxSpeed = max_speed1}); 
    chassis.waitUntilDone(); pros::delay(50);
    printPosition((char*)"last ring", true, true);
*/

  chassis.moveToPoint(60, 52, 2000, {.maxSpeed = max_speed});
  moveLiftToPos(300);
  //pros::Task save_basket(saveRings);
  //chassis.waitUntil(30);
  chassis.waitUntilDone();
  chassis.moveToPoint(60, 56, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  chassis.moveToPoint(58, 50, 2000, {.forwards = false});
  saveSecondRing();
  chassis.waitUntilDone();
  pros::delay(250);
  //extendSweep();
    chassis.waitUntilDone();
    currentPose = chassis.getPose();
  chassis.turnToHeading(200, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = max_speed1});
  chassis.waitUntilDone();
  //chassis.moveToPoint(63, 61, 2000);
  //chassis.turnToHeading(currentPose.theta-160, 2000, {.maxSpeed = 50});
  //chassis.waitUntilDone();
  //retractSweep();
  pros::delay(200);
  openClamp();
  pros::delay(300);
  currentPose = chassis.getPose();
  chassis.moveToPoint(53, 40, 2000);
  chassis.waitUntilDone();
  chassis.moveToPoint(75, 75, 2000, {.forwards = false});
  chassis.waitUntilDone();
  //spinIntake(-50);
  pros::delay(1000);
  chassis.moveToPoint(48, 48, 2000, {.maxSpeed = max_speed});
  spinIntake(127);
  moveLiftToPos(10);
  //setBasket(false);
  chassis.turnToHeading(180, 1000, {.maxSpeed = max_speed1, .earlyExitRange = 0.5});
  chassis.waitUntilDone();
  pros::delay(100);

  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, currentPose.y ,findHeading(1, 180.0)); 
  pros::delay(250); 

  //chassis.turnToHeading(180, 2000);

  if(distance_lf.get() - LEFT_DIFFERENCE > distance_lb.get()){
    left_side_motors.move(-50);
    right_side_motors.move(50);
  } else {
    left_side_motors.move(50);
    right_side_motors.move(-50);
  }

  while(abs(distance_lf.get() - distance_lb.get()) - LEFT_DIFFERENCE > 7){
    pros::delay(10);
  }
  left_side_motors.move(0);
  right_side_motors.move(0);

  currentPose = chassis.getPose();
  chassis.setPose(72.0 - findDistToWall(1), 72.0 - ((distance_bl.get() > distance_br.get()) ? (distance_bl.get() / 25.4 + 7.25) : (distance_br.get() / 25.4 + 7.25)), findHeading(1, 180)); 
  pros::delay(250); 
    printPosition((char*)"after reset", true, true); 
    //setBasket(false); //remove later 
    //pros::delay(750); //remove later2 
    chassis.moveToPoint(46, 0, 2000, {.maxSpeed = max_speed});
    //basketRings();
    chassis.turnToHeading(90, 1000, {.maxSpeed = max_speed1});
    pros::delay(1000);
    //setBasket(true);
    moveLiftToPos(510);
    chassis.moveToPose(67.25, 1, 86, 2000, {.maxSpeed = max_speed});
    //pros::Task save_basket2();
    saveRings();
    chassis.waitUntilDone();
    pros::Task lower_lift([=] {
    moveLiftToPos(240);
      // lift.move(-127);
      // pros::delay(500);
      // lift.move(0);
    setLiftBrake(pros::E_MOTOR_BRAKE_COAST);});
    pros::delay(1000);
    // chassis.moveToPoint(56, 0, 1000, {.forwards = false, .maxSpeed =max_speed});
    chassis.moveToPoint(48, 0, 1000, {.forwards = false, .maxSpeed = max_speed}); 
    chassis.turnToHeading(90, 1000, {.maxSpeed = max_speed1});
    chassis.waitUntilDone();

    

  pros::delay(1000);

  float myDist = fabs(distance_rb.get() - distance_rf.get()) > 400.0
                     ? fabs(distance_rb.get() - 1700.0) < 250.0
                           ? distance_rb.get()
                           : distance_rf.get()
                     : (distance_rb.get() + distance_rf.get()) / 2.0;
  myDist = myDist / 25.4 + 6.75;

  float myY2 = myDist - 72.0;
  printf("\n\n\n MYy: %3.2f\n\n\n", myY2);

  chassis.setPose(74.0 - findDistToWall(3), myY2, 90);
  pros::delay(500);
  printPosition((char *)"position at basket", true, true);
  pros::delay(100);
  // chassis.swingToPoint(24, -48, DriveSide::RIGHT, 1000, {.forwards = false,
  // .maxSpeed = max_speed});
  chassis.moveToPoint(38, -24, 2000,
                      {.forwards = false, .maxSpeed = max_speed});
  chassis.turnToPoint(26, -49, 1000,
                      {.forwards = false, .maxSpeed = max_speed1});
  chassis.moveToPoint(26, -49, 2000,
                      {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntil(26);
  closeClamp();
  pros::delay(500);
  spinIntake(127);
  chassis.moveToPoint(24, -24, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  chassis.turnToHeading(90.0, 2000);
  chassis.waitUntilDone();
  chassis.moveToPoint(48, -24, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  chassis.turnToHeading(180, 1000,
                        {.maxSpeed = max_speed1, .earlyExitRange = 1.0});
  chassis.waitUntilDone();
  pros::delay(100);
  currentPose = chassis.getPose();
  chassis.setPose(74.0 - findDistToWall(3), -72.0 + findDistToWall(0),
                  currentPose.theta);
  pros::delay(100);
  printPosition((char *)"position at outer ring", true);
  chassis.waitUntilDone();
  moveLiftToPos(300);
  chassis.waitUntilDone();
  pros::delay(100);
  chassis.setPose(74.0 - findDistToWall(1), -72.0 + findDistToWall(3),
                  findHeading(1, 180));
  printPosition((char *)"position at 4th corner outer ring", true);
  pros::c::delay(250);
  chassis.moveToPoint(52, -50, 2000, {.maxSpeed = max_speed}); 
  chassis.waitUntilDone();
  pros::c::delay(300);
  chassis.moveToPoint(52, -60, 2000, {.maxSpeed = max_speed}); 
  chassis.waitUntilDone();

  currentPose = chassis.getPose(); // HERE

  chassis.setPose(74.0 - findDistToWall(1), -72.0 + findDistToWall(3),
                  currentPose.theta);
  printPosition((char *)"position at 4th corner inner ring", true);
  pros::delay(100);
  chassis.moveToPoint(48, -48, 2000,
                      {.forwards = false, .maxSpeed = max_speed});
  chassis.turnToHeading(90, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  //pros::Task save_basket86678();
  saveRings();
  setBasket(false);
  pros::delay(250);
  chassis.moveToPoint(60, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  pros::c::delay(100);
  chassis.moveToPoint(48, -48, 2000, {.forwards = false});
  chassis.waitUntilDone();
  setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
  chassis.turnToHeading(-45, 2000, {.maxSpeed = max_speed1});
  pros::delay(500);
  // chassis.setPose(74.0 - findDistToWall(1), -72.0 + findDistToWall(3), 180);
  pros::delay(1000);
  printPosition((char *)"position at 4th corner last ring", true);
  chassis.moveToPose(68, -68, -45, 2000, {.forwards = false});
  openClamp();
  chassis.waitUntilDone();
  pros::c::delay(250);


  setBasket(false);

  spinIntake(127);
  pros::c::delay(50);
  spinIntake(0);

  pros::c::delay(500);

  spinIntake(-127);
  while (intake_dist.get() < 20) {
    pros::c::delay(10);
  }
  spinIntake(0);

  basketRings(false);

  pros::c::delay(250);


  chassis.moveToPose(48, -48, -45, 2000);

  /*
  chassis.moveToPoint(0, -36, 2000);
  chassis.turnToHeading(180, 2000);
  moveLiftToPos(300);
  return;
  */

  // pros::delay(2000); THIS IS TEST CODE
  // setBasket(false);
  // pros::delay(2000);
  // basketRings(false); END TEST CODE

  // chassis.moveToPoint(65, -65, 2000, {.forwards = false, .maxSpeed =
  // max_speed}); chassis.moveToPoint(60, -48, 2000, {.maxSpeed = max_speed});
  // chassis.turnToHeading(-15, 2000);
  /*
      pros::delay(250);


  */

  pros::delay(5);
} // end auton_60s_skills_1()
// Auton skills number 2 fully done
void auton_60s_skills_2() {}

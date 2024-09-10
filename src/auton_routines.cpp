#include "auton_routines.h"
#include "auton_basics.h"
#include "auton_menu.h"
#include "controls.h"
#include "lemlib/api.hpp"
#include "liblvgl/draw/lv_draw_label.h"
#include "liblvgl/draw/lv_img_buf.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#include <type_traits>
#include <variant>

ASSET(matchloadturn4ball_txt);
ASSET(touchbar_txt);

// Auton routine start positions
auton_routine null_routine{0, 0, 0, "None - Invalid Routine", nullptr};
auton_routine blue_goal_rush{1.234, -1.234, 90, "15S Auton - Near Driver # 1",
                             &goal_rush};
auton_routine blue_ring_rush{1.000, -1.300, 190, "15S Auton - Near Driver # 2",
                             &ring_rush};
auton_routine near_driver_elim{1.000, -1.300, 190,
                               "15S Auton - Near Driver # 2",
                               &DescoreRushElim}; // to be updated
auton_routine near_driver_elim2{1.000, -1.300, 190,
                                "15S Auton - Near Driver # 2",
                                &rushelim}; // to be updated

auton_routine far_from_driver_qual{1.234, -1.234, 90,
                                   "15S Auton - Near Driver # 1",
                                   &auton_15s_far_driver_qual};
auton_routine far_from_driver_elim{1.234, -1.234, 90,
                                   "15S Auton - Near Driver # 1",
                                   &auton_15s_far_driver_elim};
auton_routine far_from_driver_elim2{-0.600, 0.600, 180, "extra_1",
                                    &safe_6_ball};

auton_routine skills_1{-0.600, 0.600, 180, "60S Auton - Skills # 1",
                       &auton_60s_skills_1};
auton_routine skills_2{-0.600, 0.600, 180, "60S Auton - Skills # 2",
                       &auton_60s_skills_2};

float max_speed = 60;            // forward back max speed
int max_speed1 = (int)max_speed; // turn max speed

// FULLY DONE
void auton_15s_near_driver_qual() // DONE
{}
void goal_rush() // BLUE
{
  /*chassis.setPose(-48, -51.5, 180);
  chassis.moveToPoint(-48, -7.5, 3000, {.forwards = false, .maxSpeed = max_speed
  * 1.5f}); chassis.waitUntil(38.5); mogo_clamp.set_value(true);
  pros::delay(200);*/
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.setPose(-48, -7.5, 180);

  chassis.moveToPoint(-38.5, -18, 2000, {.maxSpeed = max_speed});
  chassis.moveToPoint(-38.5, -12, 2000,
                      {.forwards = false, .maxSpeed = max_speed});
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
  chassis.moveToPoint(-60, -40, 1000,
                      {.forwards = false, .maxSpeed = max_speed});
  intake.move(0);
  chassis.waitUntilDone();
  mogo_clamp.set_value(false);
  // return;

  chassis.moveToPoint(-60, -55, 2000, {.maxSpeed = max_speed});
  chassis.moveToPoint(-29, -25, 2000,
                      {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntilDone();
  mogo_clamp.set_value(true);

  intake.move(127);
  chassis.moveToPoint(-24, -12, 1000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  intake.move(0);
}
void rushWP() { //
}
void rushelim() { printf("%s(): Exiting\n", __func__); }

void ring_rush() // BLUE
{

  chassis.setPose(48, -52.25, 0);
  chassis.moveToPose(50, -10, 15, 3000, {.maxSpeed = max_speed});
  intake.move(-127);
  pros::delay(300);
  intake.move(0);
  lift.move(127);
  pros::delay(400);
  lift.move(0);
  intake.move(127);
  mogo_clamp.retract();
  chassis.waitUntilDone();
  chassis.moveToPoint(48, -17, 1000,
                      {.forwards = false, .maxSpeed = max_speed});
  intake.move(0);
  chassis.turnToPoint(46, 0, 1000, {.maxSpeed = max_speed1});
  chassis.moveToPoint(46, -8, 2000, {.maxSpeed = max_speed});
  intake.move(127);
  chassis.turnToHeading(270, 1000, {.maxSpeed = max_speed1});
  chassis.waitUntil(3);
  intake.move(0);
  chassis.waitUntil(6);
  mogo_rush.extend();
  chassis.waitUntilDone();
  intake.move(0);
  chassis.turnToHeading(0, 1000, {.maxSpeed = max_speed1});
  chassis.turnToPoint(24, -24, 1000,
                      {.forwards = false, .maxSpeed = max_speed1});
  chassis.moveToPoint(20, -2, 1000, {.forwards = false, .maxSpeed = max_speed});
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
void auton_15s_far_driver_qual() // Blue
{}

void safe_6_ball() // FAR ELIM
{}

void auton_15s_far_driver_elim() { printf("%s(): Exiting\n", __func__); }
void DescoreRushElim() { printf("%s(): Exiting\n", __func__); }
// STILL WORKING ON IT

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

ASSET(skillsPathPart1_txt);
void auton_60s_skills_1() {

  pros::delay(1500);
  //printf("\nBattery: %3.2f\n", pros::c::battery_get_capacity());

  chassis.setPose(0, 0, 0);
  intake.move_relative(-200, 127);
  pros::delay(600);
  chassis.moveToPoint(0, -17, 2000, {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntil(14);
  mogo_clamp.set_value(true); //clamp on mobile goal
  chassis.waitUntilDone();
  pros::delay(250);
  chassis.turnToHeading(-135, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  lemlib::Pose currentPose = chassis.getPose(); 
  chassis.setPose(currentPose.x, currentPose.y, findHeading(1, 0));
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.setPose(findDistToWall(1) - 72, findDistToWall(2) - 72, findHeading(1, 0));
  currentPose = chassis.getPose();
  printPosition((char *)"Starting position", true, true);
  intake.move(0); 
  pros::delay(300);
  //chassis.moveToPoint(-24, -48, 1000, {.forwards = false, .maxSpeed = max_speed}); //Move towards first ring 
  //chassis.waitUntil(10);
  //pros::delay(350);
  intake.move(127);
  //intake.move(0); //stop intake to avoid outaking preload 
  pros::Task lift_up1([=] { //raise lift (later lift may get in way near the wall) 
  moveLift(300);
  });

  pros::delay(250);

  //intake other rings in bottom left corner
  chassis.moveToPoint(-24, -26, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  pros::delay(100);
  chassis.turnToHeading(-90, 2000);
  chassis.moveToPoint(-49, -26, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  pros::delay(250);
  chassis.turnToHeading(180, 2000);
  chassis.waitUntilDone();
  chassis.moveToPoint(-48, -50, 2000);
  chassis.waitUntilDone();
  pros::delay(400);
  chassis.moveToPoint(-48, -67, 3000, {.maxSpeed = max_speed - 10});
  chassis.waitUntilDone();
  chassis.moveToPoint(-48,-48,2000, {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntilDone();
  pros::delay(50);
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, currentPose.y, findHeading(0, 180));
  chassis.turnToHeading(270, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  pros::delay(50);
  chassis.setPose(findDistToWall(3) - 72, findDistToWall(1) - 71.5, findHeading(1, 270));
  printPosition((char *)"Before last ring", true, true);
  chassis.moveToPoint(-62, -50, 2000, {.maxSpeed = max_speed});
  // chassis.moveToPoint(-40, -48, 1000, {.forwards = false, .maxSpeed = 40}); 
  // chassis.waitUntilDone();
  // chassis.moveToPoint(-60, -52, 2000, {.maxSpeed = max_speed});

  //move to place goal in corner
  chassis.turnToHeading(0, 1000, {.maxSpeed = max_speed1});
  pros::delay(80);
  intake.move(0);//save last intaked ring for basket
  chassis.moveToPoint(-63, -65, 1000, {.forwards = false, .maxSpeed = max_speed}); 
  pros::Task lift_down1([=] { moveLift(60); setBasket(true);});
  chassis.waitUntilDone();
  printPosition((char *)"at goal drop");
  mogo_clamp.set_value(false);
  intake.move(0);
  
  setBasket(false);
  pros::delay(500);

  //reset & move to wall stake
  chassis.moveToPoint(-48, -48, 2000);
  pros::Task basketRingstask(basketRingsAsTask);
  chassis.turnToHeading(0, 1000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  pros::delay(100);
  currentPose = chassis.getPose();
  printPosition((char *)"(-48,-48) reset point before sensors", true, true);
  chassis.setPose(currentPose.x, currentPose.y, findHeading(1, 0));
  pros::delay(300);
  printPosition((char *)"(-48,-48) angle through sensors", true, true);
  pros::delay(100);
  chassis.turnToHeading(0, 2000);
  chassis.waitUntilDone();
  pros::delay(100);
  printPosition((char *)"(-48,-48) final angle", true, true);
  pros::delay(250);
  currentPose = chassis.getPose();
  chassis.setPose(findDistToWall(1) - 72, findDistToWall(2) - 72, currentPose.theta);
  pros::delay(100);
  // chassis.moveToPoint(-48, -48, 2000, {.forwards = false, .maxSpeed = max_speed});
  // chassis.waitUntilDone();
  // chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  // chassis.waitUntilDone();
  pros::delay(100);
  printPosition((char *)"turn coords", true, true);
  pros::delay(100);
  printPosition((char *)"(-48,-48) final distance", true, true);
  chassis.moveToPoint(-48, 0, 2000, {.maxSpeed = 60});
  setBasket(true);
  moveLift(540);
  chassis.waitUntilDone();
    printPosition((char *)"(-48, 0 distance)", true, true);



// return;
  //chassis.setPose(-72 + findDistToWall(1), -72 + findDistToWall(2), findHeading(1, 0));

  printPosition((char *)"basket lineup", true, true);
  intake.move(127);
  chassis.turnToHeading(270, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = max_speed1});

  chassis.moveToPoint(-68, -2, 2000, {.maxSpeed = max_speed});
  printPosition((char *)"basket scoring", true, true);
  chassis.waitUntilDone();
  intake.move(0);
  moveLift(240);
  pros::delay(200);
  chassis.moveToPoint(-61, 0, 1000, {.forwards = false, .maxSpeed = 20});
  printPosition((char *)"basket backup", true, true);
  setBasket(false);
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.moveToPoint(-48, 0, 1000, {.forwards = false, .maxSpeed = max_speed});
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  // setBasket(false);
  // basketRings();

  //2nd corner
  chassis.turnToPoint(-48, 24, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = max_speed1 - 10});

  pros::Task basketRingstask2(basketRingsAsTask);
  chassis.waitUntilDone();


  currentPose = chassis.getPose();
  chassis.setPose(findDistToWall(1) - 72, currentPose.y, findHeading(1, 0));
  printPosition((char *)"2nd corner transition", true, true);
  pros::delay(250);
  chassis.moveToPoint(-48, 24, 3000, {.maxSpeed = max_speed - 10});
  pros::Task basketRingstask3(basketRingsAsTask);
  chassis.waitUntilDone();
  printPosition((char *)"2nd corner 1st ring", true, true);
  //basketRings();
  //chassis.moveToPoint(-24, 40, 2000, {.maxSpeed = max_speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = max_speed1});
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
  // mogo_clamp.set_value(true);
  // intake.move(-90);
  // pros::delay(500);

  // chassis.turnToPoint(-65, 62, 2000, {.maxSpeed = max_speed1});
  // chassis.moveToPoint(-65, 62, 2000, {.maxSpeed = max_speed});
  // mogo_rush.extend();
  // chassis.waitUntilDone();
  // printPosition((char *)"Before sweep", true, true);
  // intake.move(0);
  // chassis.turnToHeading(135, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
  // chassis.waitUntil(90);
  // mogo_rush.retract();
  // chassis.waitUntilDone();
  // printPosition((char *)"After sweep", true, true);
  // chassis.moveToPoint(-65, 65, 2000, {.forwards = false, .maxSpeed = max_speed}); 
  // mogo_clamp.set_value(false);

    moveLift(350);
    chassis.moveToPoint(-48, 48, 2000, {.maxSpeed = max_speed});
    chassis.turnToHeading(270, 1000, {.maxSpeed = max_speed1});
    chassis.waitUntilDone();


    moveLift(350);
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
    moveLift(350);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //chassis.waitUntilDone();
    mogo_clamp.set_value(true);
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
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntilDone();
    pros::delay(500);
    if(currentPose.theta > 180.0){
      chassis.turnToHeading(360.0, 500, {.earlyExitRange = 1});
    }else{
      chassis.turnToHeading(0.0, 500, {.earlyExitRange = 1});
    }
    chassis.waitUntilDone();
    moveLift(170);
    pros::delay(100);
    pros::Task lift_down2([=] {
      lift.move(-10);
      pros::delay(250);
      lift.move(0);
      lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    });
    currentPose = chassis.getPose();
    chassis.moveToPoint(0, 50, 2000, {.forwards = false, .maxSpeed = max_speed}); chassis.getPose();


  
  float myDist1 = fabs(distance_rb.get() - distance_rf.get()) > 400.0
                     ? fabs(distance_rb.get() - 1700.0) < 250.0
                           ? distance_rb.get()
                           : distance_rf.get()
                     : (distance_rb.get() + distance_rf.get()) / 2.0;
  myDist1 = myDist1 / 25.4 + 6.75;

  float myX = myDist1 - 72.0;
  printf("\n\n\n MYy: %3.2f\n\n\n", myX);


    printPosition((char*)"Before pickup", true, true);


    mogo_clamp.set_value(true);

    chassis.setPose(myX, 74.0-findDistToWall(3), currentPose.theta);

    currentPose = chassis.getPose();

    printPosition((char*)"after pickup");


    pros::delay(250);
    chassis.turnToHeading(135, 2000, {.maxSpeed = max_speed1});
    chassis.waitUntilDone();

  printPosition((char*)"after turn", true, true);

  intake.move(127);
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
  moveLift(300);
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
  moveLift(300);
  //pros::Task save_basket(saveRings);
  //chassis.waitUntil(30);
  chassis.waitUntilDone();
  chassis.moveToPoint(60, 56, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  chassis.moveToPoint(58, 50, 2000, {.forwards = false});
  saveSecondRing();
  chassis.waitUntilDone();
  pros::delay(250);
  //mogo_rush.extend();
    chassis.waitUntilDone();
    currentPose = chassis.getPose();
  chassis.turnToHeading(200, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = max_speed1});
  chassis.waitUntilDone();
  //chassis.moveToPoint(63, 61, 2000);
  //chassis.turnToHeading(currentPose.theta-160, 2000, {.maxSpeed = 50});
  //chassis.waitUntilDone();
  //mogo_rush.retract();
  pros::delay(200);
  mogo_clamp.set_value(false);
  pros::delay(300);
  currentPose = chassis.getPose();
  chassis.moveToPoint(53, 40, 2000);
  chassis.waitUntilDone();
  chassis.moveToPoint(75, 75, 2000, {.forwards = false});
  chassis.waitUntilDone();
  //intake.move(-50);
  pros::delay(1000);
  chassis.moveToPoint(48, 48, 2000, {.maxSpeed = max_speed});
  intake.move(127);
  moveLift(10);
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
    moveLift(510);
    chassis.moveToPose(67.25, 1, 86, 2000, {.maxSpeed = max_speed});
    //pros::Task save_basket2();
    saveRings();
    chassis.waitUntilDone();
    pros::Task lower_lift([=] {
    moveLift(240);
      // lift.move(-127);
      // pros::delay(500);
      // lift.move(0);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);});
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

  float myY = myDist - 72.0;
  printf("\n\n\n MYy: %3.2f\n\n\n", myY);

  chassis.setPose(74.0 - findDistToWall(3), myY, 90);
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
  mogo_clamp.set_value(true);
  pros::delay(500);
  intake.move(127);
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
  moveLift(300);
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
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  chassis.turnToHeading(-45, 2000, {.maxSpeed = max_speed1});
  pros::delay(500);
  // chassis.setPose(74.0 - findDistToWall(1), -72.0 + findDistToWall(3), 180);
  pros::delay(1000);
  printPosition((char *)"position at 4th corner last ring", true);
  chassis.moveToPose(68, -68, -45, 2000, {.forwards = false});
  mogo_clamp.set_value(false);
  chassis.waitUntilDone();
  pros::c::delay(250);


  setBasket(false);

  intake.move(127);
  pros::c::delay(50);
  intake.move(0);

  pros::c::delay(500);

  intake.move(-127);
  while (intake_dist.get() < 20) {
    pros::c::delay(10);
  }
  intake.move(0);

  basketRings(false);

  pros::c::delay(250);


  chassis.moveToPose(48, -48, -45, 2000);

  /*
  chassis.moveToPoint(0, -36, 2000);
  chassis.turnToHeading(180, 2000);
  moveLift(300);
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

#include "auton_routines.h"
#include "auton_basics.h"
#include "auton_menu.h"
#include "controls.h"
#include "hal.h"
#include "lemlib/api.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "liblvgl/core/lv_obj_class.h"
#include "liblvgl/draw/lv_draw_label.h"
#include "liblvgl/draw/lv_img_buf.h"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include <cmath>
#include <cstdio>
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#include <type_traits>
#include <variant>

ASSET(matchloadturn4ball_txt);
ASSET(touchbar_txt);

// Auton routine start positions
auton_routine null_routine{0, 0, 0, "None - Invalid Routine", nullptr};

auton_routine safe_positive{0, 0, 0, "None - Invalid Routine", &positiveHalfWp};

auton_routine positive_WP{0, 0, 0, "None - Invalid Routine", &possitiveFullWP};

auton_routine safe_positive_stake{0, 0, 0, "None - Invalid Routine", &positiveWpStake};

auton_routine goal_rush_and_stake{0, 0, 0, "None - Invalid Routine", &goalRushWallStake};

auton_routine safe_negative{0, 0, 0, "None - Invalid Routine", &negativeHalfWP}; //EVERYTHING DONE

auton_routine negetive_6_ring{0, 0, 0, "None - Invalid Routine", &negSixRing};

auton_routine negetive_safe_wall_stake{0, 0, 0, "None - Invalid Routine", 
                                         &negHalfWPWallStake};

auton_routine safe_six_ring{0, 0, 0, "None - Invalid Routine", 
                                         &safeSixRing};

auton_routine skills_1{-0000, 0.000, 00, "60S Auton - Skills # 1", &auton_60s_skills_1};

auton_routine solo_WP{-0.600, 0.600, 180, "extra_1", &soloWP};

auton_routine skills_2{-0.600, 0.600, 180, "60S Auton - Skills # 2",
                       &auton_60s_skills_2};

float max_speed = 60;            // forward back max speed
float mid_speed = 60;            // forward medium speed
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

void printPositionV2(char *msg, bool withDistanceSensors = false,
                      bool detailedDist = false, int timer = -1) {
  lemlib::Pose currentPose = chassis.getPose();
  printf("%s\tX: %3.2f     \tY: %3.2f     \tTheta: %3.2f     ", msg, currentPose.x,
         currentPose.y, currentPose.theta);
  if(timer > 0){
    printf("\tTime: %d", pros::millis() - timer);
  }
  printf("\n");
  if (withDistanceSensors) {
    if (detailedDist) {
      printf("F: %d     \tL: %d\n",
             distance_front.get(), distance_left.get());
    }
    printf("Front: %3.2f     \tLeft: %3.2f\n",
           distToWallF(), distToWallL());
    printf("Front position: %3.2f     \tLeft position: %3.2f\n",
            72 - distToWallF() * cos(deg2rad(fmodf(currentPose.theta + 360, 90))), 72 - distToWallL() * cos(deg2rad(fmodf(currentPose.theta + 360, 90))));
  }
}

/*********************** THESE ARE ALL THE AUTONS FOR WORLDS ***********************/

/*********************** POSSITIVE HALF WP ***********************/


/** positiveHalfWp
 * Starts at wall stake, facing towards negative corner
 * Preload in alliance stake device
 * 
 * Alliance stake
 * 3/4 rings on one goal
 * Touch bar
 */
void redPositiveHalfWP(){
  int time = pros::millis();
  int speed = 80;
  float speed1 = float(speed);
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  pros::delay(2000);
  
  chassis.setPose(14.5,-57,230);
  pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);  
  chassis.moveToPoint(12, -59.5, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  pros::Task lift1 ([=] {moveLiftToPos(930);});    
  pros::delay(300); 
  chassis.moveToPoint(19.5, -53, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(24, -24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  pros::Task lift2 ([=] {moveLiftToPos(-10);});    
  chassis.moveToPoint(24, -33,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.moveToPoint(24, -21,600,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.waitUntil(4);
  closeClamp();
  pros::delay(280);
  chassis.turnToPoint(48, -24, 1000,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  spinIntake(127);
  chassis.moveToPoint(54, -24, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(46, -24, 1000,{.forwards = false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  // chassis.turnToPoint(48, -48, 1000,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  // chassis.moveToPoint(48, -48, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(54, -47, 1000,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1}, false);
  startSorting();
  chassis.moveToPoint(54, -47, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  pros::Task lift3 ([=] {moveLiftToPos(350);});      
  chassis.moveToPoint(58, -51, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToHeading(90, 1000,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1},false);
  pros::delay(500);
  chassis.turnToHeading(135, 1400,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1},false);
  stopSorting();
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x+12,  temp_pos.y-12 , 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(temp_pos.x-2,  temp_pos.y+2, 800,{.forwards=false,.maxSpeed=127,.minSpeed=5,.earlyExitRange=1},false);
  chassis.moveToPoint(temp_pos.x+5.5,  temp_pos.y-5.5, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(temp_pos.x-2,  temp_pos.y+2, 1500,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(14,-14,2000,{.forwards = false});
  chassis.waitUntil(15);
  stopIntake();
  chassis.waitUntil(1000);
  moveLiftToPos(-10,50,200);

  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }

}

void bluePositiveHalfWP(){//PATH DONE
  int time = pros::millis();
  int speed = 100;
  float speed1 = float(speed);
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  // chassis.setPose(15.5,58,315);
  spinIntake(127);
  turnToHeadingWithVis(160);
  /*pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);  
  /***************A-WALL STAKE**************/
  chassis.moveToPoint(11.5, 60, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  //moveLiftToPos(800, 127, 750);
  moveLiftToPos(550, 127, 750);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x+(-7 * sin(deg2rad(temp_pos.theta))),  temp_pos.y+(-7 * cos(deg2rad(temp_pos.theta))) , 700,{.forwards = false, .maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  /***************CORNER RING**************/
  chassis.turnToPoint(54, 55, 1000, {.forwards = false, .maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(54,55,1500,{.forwards = false, .maxSpeed = speed1});
  pros::Task lift3 ([=] {moveLiftToPos(310);});    
  chassis.turnToHeading(45,1000,{});
  temp_pos = chassis.getPose();
  spinIntake(127);
  //stopSorting();
  chassis.moveToPoint(temp_pos.x+14,  temp_pos.y+14 , 700,{.maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  //saveRing1(300);
  pros::delay(300);
  chassis.moveToPoint(temp_pos.x+2,  temp_pos.y+2, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=0.5}, false);
  stopIntake();
  /***************GOAL GRAB**************/
  chassis.turnToPoint(36, 48, 1000, {.forwards = false, .maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(36, 48, 1000,{.forwards = false, .maxSpeed=speed1});
  chassis.turnToPoint(24, 24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  pros::Task lift2 ([=] {moveLiftToPos(-10);});    
  chassis.moveToPoint(24, 24,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.waitUntil(27);
  closeClamp();
  pros::delay(280);
  /***************MIDDLE RING**************/
  chassis.turnToPoint(0, 0, 1000, {.maxSpeed = speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(8.4, 8.4, 1000, {.maxSpeed = speed1 - 30,.minSpeed=25,.earlyExitRange=2}, false);
  pros::delay(100);
  extendLeftSweeper();
  pros::delay(300);
  chassis.moveToPoint(24, 24, 1500, {.forwards = false, .maxSpeed = speed1 - 30,.minSpeed=25,.earlyExitRange=2});
  chassis.turnToHeading(100, 1000, {.maxSpeed = speed}, false);
  pros::delay(100);
  retractLeftSweeper();
  pros::delay(200);
  /***************OPEN RING**************/
  spinIntake(127);
  startSorting();
  chassis.turnToPoint(48, 24, 1000, {.maxSpeed = speed, .minSpeed = 5, .earlyExitRange = 1});
  moveToPointWithVis(51, 24, 1500, {.maxSpeed = speed1, .xLimit = 54, .driveThrough = true, .keepDriving = true});
  /***************AWS RING**************/
  chassis.moveToPoint(24, 48, 1500, {.forwards = false, .maxSpeed = speed1, .minSpeed=25, .earlyExitRange=2});
  chassis.turnToPoint(0, 48, 1000, {.maxSpeed = speed, .minSpeed = 5, .earlyExitRange = 1}, false);
  chassis.moveToPoint(0, 48, 1000, {.maxSpeed = speed1, .minSpeed=25, .earlyExitRange=2});
  chassis.turnToHeading(180, 1000, {.maxSpeed = speed, .minSpeed = 5, .earlyExitRange = 1});
  chassis.moveToPoint(0, 28, 1000, {.maxSpeed = speed1, .minSpeed=25, .earlyExitRange=2}, false);
  moveLiftToPos(400, 127, 750);

  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void positiveHalfWp(){
  if(COLOR){
    redPositiveHalfWP();
  } else {
    bluePositiveHalfWP();
  }
}

/*********************** POSSITIVE WP WITH STAKE ***********************/
void bluePositiveWPStake(){//PATH DONE  
  int time = pros::millis();
  int speed = 60;
  float speed1 = float(speed);
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  chassis.setPose(15.5,58,315);
  /***************A-WALL STAKE**************/
  chassis.moveToPoint(11.5, 60, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);

  moveLiftToPos(230, 127, 750);
  stopLiftHold();

  //moveLiftToPos(550, 127, 750);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x+(-7 * sin(deg2rad(temp_pos.theta))),  temp_pos.y+(-7 * cos(deg2rad(temp_pos.theta))) , 700,{.forwards = false, .maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  chassis.moveToPoint(55, 48, 1500,{.forwards = false, .maxSpeed = 127,.minSpeed=25,.earlyExitRange=2});
  chassis.waitUntil(7);
  pros::Task lift_pickup([=] {
    //liftPickup();
  });
  /***************THIRD RING**************/
  chassis.turnToPoint(48,24,1500,{.maxSpeed = speed,.minSpeed = 5, .earlyExitRange = 1});
  spinIntake(127);
  startSorting();
  chassis.moveToPoint(48,24,1500,{.maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  chassis.waitUntil(100);
  pros::delay(400);
  /***************GOAL GRAB**************/
  chassis.turnToPoint(24, 24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1}, false);
  chassis.moveToPoint(24, 24,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.waitUntil(23.5);
  closeClamp();
  //pros::Task lift3 ([=] {moveLiftToPos(150, 80, 500);});
  pros::delay(70);
  stopIntake();
  pros::delay(280);
  /***************WALL STAKE**************/
  //Add Distance reset
  chassis.moveToPoint(45, 24, 1500, {.maxSpeed = speed1});
  chassis.turnToPoint(71.5, -0.5, 1500, {.maxSpeed = speed}, false);
  temp_pos = chassis.getPose();
  //pros::Task lift_stake ([=] {liftUpWallStake();});
  chassis.moveToPoint(71.5 + (-11.5 * sin(deg2rad(temp_pos.theta))), -0.5 + (-11.5 * cos(deg2rad(temp_pos.theta))), 1500, {.maxSpeed = 60}, false);
  moveLiftToPos(400, 127, 500);
  pros::delay(300);
  /***************CORNER**************/
  chassis.moveToPoint(71.25 + (-20 * sin(deg2rad(temp_pos.theta))), 0 + (-20 * cos(deg2rad(temp_pos.theta))), 1500, {.forwards = false, .maxSpeed = 50, .minSpeed=25,.earlyExitRange=2});
  return;
  chassis.moveToPoint(48, 24, 1500, {.forwards = false, .maxSpeed = speed1, .minSpeed=25,.earlyExitRange=2});
  chassis.turnToPoint(52, 48, 1000, {.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(52,48,1500,{.maxSpeed = speed1}); 
  turnToHeadingWithVis(45,1000, 45, {});
  temp_pos = chassis.getPose();
  spinIntake(127);
  //stopSorting();
  chassis.moveToPoint(temp_pos.x+(20 * sin(deg2rad(temp_pos.theta))),  temp_pos.y+(20 * cos(deg2rad(temp_pos.theta))) , 700,{.maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  //saveRing1(300);
  pros::delay(300);
  chassis.moveToPoint(temp_pos.x+2,  temp_pos.y+2, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=0.5}, false);
  /***************AWS STAKE RING**************/
  chassis.turnToPoint(0, 48, 1500, {.maxSpeed = speed,.minSpeed=5,.earlyExitRange=1});
  moveToPointWithVis(0, 48, 2000, {.maxSpeed = speed1, .xLimit = 5}, 750);
  /***************TOUCH LADDER**************/
  chassis.turnToHeading(180, 1000, {.maxSpeed = speed, .minSpeed=5, .earlyExitRange=1});
  chassis.moveToPoint(0, 30, 1000, {.maxSpeed = speed1, .minSpeed=25, .earlyExitRange=2});

  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void positiveWpStake(){
  if(COLOR){
    return;
  } else {
    bluePositiveWPStake();
  }
}

/*********************** POSSITIVE WP WITH GOAL ***********************/
void bluePositiveWPGoal(){
  int time = pros::millis();
  int speed = 80;
  float speed1 = float(speed);
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  chassis.setPose(15.5,58,315);
  /***************A-WALL STAKE**************/
  chassis.moveToPoint(11.5, 60, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  //moveLiftToPos(800, 127, 750);
  moveLiftToPos(550, 127, 750);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x+(-7 * sin(deg2rad(temp_pos.theta))),  temp_pos.y+(-7 * cos(deg2rad(temp_pos.theta))) , 700,{.forwards = false, .maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  chassis.moveToPoint(55, 48, 1500,{.forwards = false, .maxSpeed = 127,.minSpeed=25,.earlyExitRange=2});
  chassis.waitUntil(7);
  pros::Task lift_pickup([=] {
    liftPickup();
  });
  /***************THIRD RING**************/
  chassis.turnToPoint(48,24,1500,{.maxSpeed = speed,.minSpeed = 5, .earlyExitRange = 1});
  spinIntake(127);
  startSorting();
  chassis.moveToPoint(48,24,1500,{.maxSpeed = 127,.minSpeed = 5, .earlyExitRange = 1}, false);
  pros::delay(800);
  /***************MIDDLE GOAL**************/
  turnToHeadingWithVisGoal(180, 1500, 180, 127);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x+((distToObject() - 10.36) * sin(deg2rad(temp_pos.theta))),  temp_pos.y+((distToObject() - 10.36) * cos(deg2rad(temp_pos.theta))) , 700,{.maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2});
  moveLiftToPos(520, 127, 400);
  chassis.turnToHeading(135, 1500, {.maxSpeed = speed, .minSpeed=5, .earlyExitRange=1});
  chassis.moveToPoint(39.5, 24, 1500, {.forwards = false, .maxSpeed = speed1, .minSpeed=25,.earlyExitRange=2});
  /***************GOAL GRAB**************/
  chassis.turnToPoint(24, 24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1}, false);
  chassis.moveToPoint(24, 24,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.waitUntil(23.5);
  closeClamp();
  pros::delay(70);
  stopIntake();
  pros::delay(280);
  /***************CORNER**************/
  chassis.moveToPoint(48, 24, 1500, {.maxSpeed = speed1, .minSpeed=25,.earlyExitRange=2});
  moveLiftToPos(310, 127, 500);
  chassis.turnToPoint(52, 48, 1000, {.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(52,48,1500,{.maxSpeed = speed1}); 
  turnToHeadingWithVis(45,1000, 45, {});
  temp_pos = chassis.getPose();
  spinIntake(127);
  //stopSorting();
  chassis.moveToPoint(temp_pos.x+(20 * sin(deg2rad(temp_pos.theta))),  temp_pos.y+(20 * cos(deg2rad(temp_pos.theta))) , 700,{.maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  //saveRing1(300);
  pros::delay(300);
  chassis.moveToPoint(temp_pos.x+2,  temp_pos.y+2, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=0.5}, false);
  /***************AWS STAKE RING**************/
  chassis.turnToPoint(0, 48, 1500, {.maxSpeed = speed,.minSpeed=5,.earlyExitRange=1});
  moveToPointWithVis(0, 48, 2000, {.maxSpeed = speed1, .xLimit = 5}, 600);
  /***************TOUCH LADDER**************/
  chassis.turnToHeading(180, 1000, {.maxSpeed = speed, .minSpeed=5, .earlyExitRange=1});
  chassis.moveToPoint(0, 30, 1000, {.maxSpeed = speed1, .minSpeed=25, .earlyExitRange=2});

  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void positiveWpGoal(){
  if(COLOR){
    return;
  } else {
    bluePositiveWPGoal();
  }
}

/*********************** POSSITIVE FULL WP ***********************/

void bluePossitiveFullWP(){
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  
  chassis.setPose(14.5,57,-50);
  pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);  
  chassis.moveToPoint(12, 59.5, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  pros::Task lift1 ([=] {moveLiftToPos(930);});    
  pros::delay(450); 
  chassis.moveToPoint(19.5, 53, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(48, 24,1500,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  intake_anti_jam.suspend();
  pros::Task lift2 ([=] {liftPickup();});  
  chassis.moveToPoint(48, 24,1000,{.maxSpeed=speed1-50,.minSpeed=5,.earlyExitRange=0.5});
  spinIntake(127);
  intake_anti_jam.suspend();
  chassis.turnToHeading(180, 700,{},false);
  temp_pos = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallR() * cos(deg2rad(temp_pos.theta))), 72 - fabs(distToWallB() * cos(deg2rad(temp_pos.theta))), temp_pos.theta);
  chassis.moveToPoint(48, 21.5,1000,{.maxSpeed=speed1-50,.minSpeed=5,.earlyExitRange=0.5});
  printPosition((char *) "Before WS", false, false);
  pros::delay(50);
  chassis.turnToPoint(71, 0,1000,{.maxSpeed=speed-40});
  chassis.moveToPoint(61, 7,2000,{.maxSpeed=speed1-60});
  chassis.waitUntil(100);
  stopIntake();
  moveLiftToPos(850,127,1000);
  pros::delay(100);
  return;
  spinIntake(127);
  moveLiftToPos(1100,25,500);    
  pros::delay(100);
  chassis.arcade(-15, 0);
  pros::delay(200);
  chassis.arcade(0, 0);
  chassis.moveToPoint(51, -23,1000,{.forwards=false,.minSpeed=5,.earlyExitRange=0.5});
  chassis.turnToHeading(180, 800,{.minSpeed=5,.earlyExitRange=0.5},false);
  temp_pos = chassis.getPose();
  printf("Back Dist: %f", distToWallB());
  chassis.setPose(72 - fabs(distToWallR() * cos(deg2rad(temp_pos.theta))), - 72 + fabs(distToWallB() * cos(deg2rad(temp_pos.theta))), temp_pos.theta);
  pros::delay(100);
  return;
  if(distToObject() < 20.66)
  {
    chassis.turnToPoint(51, 0, 1000,{.forwards =false,.minSpeed=5,.earlyExitRange=0.5});
    chassis.moveToPoint(50, -11.5, 1000,{.forwards =false,.maxSpeed = 50});
    chassis.waitUntil(1000);
    closeClamp();
    /*chassis.waitUntilDone();
    closeClamp();
    pros::delay(200);
    startSorting();
    chassis.moveToPoint(48, -44, 2000);
    chassis.turnToHeading(135, 1000);
    chassis.moveToPoint(72, -72, 1000);
    chassis.moveToPoint(56, -56, 1000,{.forwards = false});
    //*/
    printf("Lift Pos: %f",getLiftPosition());
  } else  {
    chassis.turnToPoint(24, -24, 1000,{.forwards =false});
  }
  
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }

}

void redPossitiveFullWP(){
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = true;
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  
  chassis.setPose(14.5,-57,230);
  pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);  
  chassis.moveToPoint(12, -59.5, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  pros::Task lift1 ([=] {moveLiftToPos(930);});    
  pros::delay(450); 
  chassis.moveToPoint(19.5, -53, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(48, -24,1500,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  intake_anti_jam.suspend();
  pros::Task lift2 ([=] {liftPickup();});  
  chassis.moveToPoint(48, -24,1000,{.maxSpeed=speed1-50,.minSpeed=5,.earlyExitRange=0.5});
  spinIntake(127);
  intake_anti_jam.suspend();
  chassis.turnToHeading(0, 700,{},false);
  temp_pos = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallR() * cos(deg2rad(temp_pos.theta))), - 72 + fabs(distToWallB() * cos(deg2rad(temp_pos.theta))), temp_pos.theta);
  chassis.moveToPoint(48, -21.5,1000,{.maxSpeed=speed1-50,.minSpeed=5,.earlyExitRange=0.5});
  printPosition((char *) "Before WS", false, false);
  pros::delay(50);
  chassis.turnToPoint(71, 0,1000,{.maxSpeed=speed-40});
  chassis.moveToPoint(61, -7,2000,{.maxSpeed=speed1-60});
  chassis.waitUntil(100);
  stopIntake();
  moveLiftToPos(850,127,1000);
  pros::delay(100);
  return;
  spinIntake(127);
  moveLiftToPos(1100,25,500);    
  pros::delay(100);
  chassis.arcade(-15, 0);
  pros::delay(200);
  chassis.arcade(0, 0);
  chassis.moveToPoint(51, -23,1000,{.forwards=false,.minSpeed=5,.earlyExitRange=0.5});
  chassis.turnToHeading(0, 800,{.minSpeed=5,.earlyExitRange=0.5},false);
  temp_pos = chassis.getPose();
  printf("Back Dist: %f", distToWallB());
  chassis.setPose(72 - fabs(distToWallR() * cos(deg2rad(temp_pos.theta))), - 72 + fabs(distToWallB() * cos(deg2rad(temp_pos.theta))), temp_pos.theta);
  pros::delay(100);
  return;
  if(distToObject() < 20.66)
  {
    chassis.turnToPoint(51, 0, 1000,{.forwards =false,.minSpeed=5,.earlyExitRange=0.5});
    chassis.moveToPoint(50, -11.5, 1000,{.forwards =false,.maxSpeed = 50});
    chassis.waitUntil(1000);
    closeClamp();
    /*chassis.waitUntilDone();
    closeClamp();
    pros::delay(200);
    startSorting();
    chassis.moveToPoint(48, -44, 2000);
    chassis.turnToHeading(135, 1000);
    chassis.moveToPoint(72, -72, 1000);
    chassis.moveToPoint(56, -56, 1000,{.forwards = false});
    //*/
    printf("Lift Pos: %f",getLiftPosition());
  } else  {
    chassis.turnToPoint(24, -24, 1000,{.forwards =false});
  }




  
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
} 

void possitiveFullWP(){
  if(COLOR){
    redPossitiveFullWP();
  } else {
    bluePossitiveFullWP();
  }
}

/*********************** GOAL RUSH ***********************/

void RedLateStake(){
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = true;
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  
  chassis.setPose(14.5,-57,230);
  pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);  
  chassis.moveToPoint(12, -59.5, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  pros::Task lift1 ([=] {moveLiftToPos(930);});    
  pros::delay(300); 
  chassis.moveToPoint(19.5, -53, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(24, -24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  pros::Task lift2 ([=] {moveLiftToPos(-10);});    
  chassis.moveToPoint(24, -33,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.moveToPoint(24, -21,600,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.waitUntil(4);
  closeClamp();
  pros::delay(280);
  chassis.turnToPoint(48, -24, 1000,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  spinIntake(127);
  startSorting();
  chassis.moveToPoint(54, -24, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(46, -24, 1000,{.forwards = false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  // chassis.turnToPoint(48, -48, 1000,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  // chassis.moveToPoint(48, -48, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(54, -47, 1000,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(54, -47, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(58, -51, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToHeading(135, 1400,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1},false);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x+10,  temp_pos.y-10 , 1000,{.maxSpeed=speed1-50,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(temp_pos.x-2,  temp_pos.y+2, 800,{.forwards=false,.maxSpeed=127,.minSpeed=5,.earlyExitRange=1},false);
  pros::delay(500);
  liftPickup();
  chassis.moveToPoint(temp_pos.x+5.5,  temp_pos.y-5.5, 1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  intake_anti_jam.suspend();
  chassis.moveToPoint(temp_pos.x-2,  temp_pos.y+2, 1500,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(52, -16, 1000,{});
  chassis.moveToPoint(52, -16, 1000, {.maxSpeed = speed1});
  chassis.turnToHeading(0, 300,{});
  liftIntake();
  chassis.waitUntil(8);
  openClamp();
  spinIntake(5);
  chassis.waitUntil(1000);
  temp_pos = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallR() * cos(deg2rad(temp_pos.theta))), temp_pos.y, temp_pos.theta);
  pros::delay(50);
  if(distToObject() < 20.66)
  {
    temp_pos = chassis.getPose();
    chassis.moveToPoint(temp_pos.x, temp_pos.y + (distToObject() - 10.5), 800, {.maxSpeed = 100});
    chassis.turnToHeading(0, 300,{},false);
    moveLiftToPos(1500,127,1000); 
    intake_anti_jam.resume();
    chassis.moveToPoint(51,-18, 1500,{.forwards=false,.maxSpeed=speed1,.minSpeed=25,.earlyExitRange=3});
    pros::Task lift2 ([=] {moveLiftToPos(0);});    
    chassis.moveToPoint(51,-26, 600,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
    chassis.waitUntil(1000);
    closeClamp();    
    dropIntake();
    chassis.moveToPoint(38, -7,1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});    
    chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 1000);
    printf("Lift Pos: %f",getLiftPosition());
  } else  {
    chassis.turnToPoint(72, 0, 1000, {.maxSpeed = speed, .minSpeed = 5, .earlyExitRange = 1});
  }

  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void BlueLateStake(){
  int time = pros::millis();
  int speed = 115;
  float speed1 = float(speed);
  lemlib::Pose temp_pos = chassis.getPose();

  startSorting();
  chassis.setPose(62.5,52,185);
  chassis.moveToPoint(57, 17, 10000,{.minSpeed = 127, .earlyExitRange = 2});
  chassis.turnToHeading(194, 1000, {.minSpeed = 25, .earlyExitRange = 4});
  chassis.waitUntil(3);
 
  chassis.moveToPoint(62.5, 30, 1000,{.forwards = false,.maxSpeed = speed1,.minSpeed = 50, .earlyExitRange = 2}, false);

  chassis.moveToPoint(40, 48, 1000,{.forwards = false,.maxSpeed = speed1,.minSpeed = 25, .earlyExitRange = 2}, false);

  chassis.turnToPoint(24, 24, 1000,{.forwards =false,.maxSpeed = speed,.minSpeed = 25, .earlyExitRange = 2});
  chassis.moveToPoint(30, 30, 1000,{.forwards = false,.maxSpeed = speed1,.minSpeed = 32, .earlyExitRange = 4});
  
  chassis.moveToPoint(24, 24.5, 1000,{.forwards = false,.maxSpeed = 80,.minSpeed = 25, .earlyExitRange = 2});
  chassis.waitUntil(11);
  closeClamp();
  spinIntake(127);
  chassis.turnToHeading(70, 1000, {.maxSpeed = speed, .minSpeed = 25, .earlyExitRange = 4});
  chassis.moveToPoint(44, 27, 1000,{.maxSpeed = speed1 - 20,.minSpeed = 25, .earlyExitRange = 2}, false);
  pros::delay(300);
  chassis.moveToPoint(48, 24, 1000, {.maxSpeed = speed1 - 20, .minSpeed = 25, .earlyExitRange = 2});
  chassis.moveToPoint(20, 36, 1000,{.forwards = false,.maxSpeed = speed1,.minSpeed = 25, .earlyExitRange = 2});
  spinIntake(-127);
  pros::delay(90);
  spinIntake(127);
  chassis.waitUntilDone();
  pros::delay(500);
  openClamp();

  chassis.moveToPoint(30, 28, 1000, {.maxSpeed = speed1, .minSpeed = 25, .earlyExitRange = 2});
  chassis.turnToPoint(60, 22, 1000, {.forwards = false, .maxSpeed = speed, .minSpeed = 25, .earlyExitRange = 4});
  chassis.moveToPoint(56, 17, 1000, {.forwards = false, .maxSpeed = speed1, .minSpeed = 40, .earlyExitRange = 2});
  chassis.moveToPoint(64, 20, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 25, .earlyExitRange = 2});
  chassis.waitUntil(6.5);
  closeClamp();
  pros::delay(100);
  chassis.turnToHeading(0, 1000, {.maxSpeed = speed, .minSpeed = 25, .earlyExitRange = 4});
  chassis.moveToPoint(54, 47, 2000,{.minSpeed = 25, .earlyExitRange = 2});
  chassis.turnToHeading(45, 1000,{},false);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x + 15, temp_pos.y + 15, 1000, {.maxSpeed = speed1, .minSpeed = 25, .earlyExitRange = 2});
  chassis.moveToPoint(temp_pos.x, temp_pos.y, 1000, {.forwards = false, .maxSpeed = speed1, .minSpeed = 25, .earlyExitRange = 2});
  chassis.turnToHeading(180,1000);
  chassis.moveToPoint(temp_pos.x, 25, 3000);



  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }

  
}

void lateStake(){
  if(COLOR){
    RedLateStake();
  } else {
    BlueLateStake();
  }
}

/*********************** GOAL RUSH AND WALL STAKE ***********************/

void goalRushWallStake(){
  if(COLOR){
    return;
  } else {
    return;
  }
}

/*********************** NEGETIVE HALF WP ***********************/

/** negetiveHalfWp
 * Start at alliance stake, facing positive corner
 * Score AS
 * Get pos goal, score 3 rings by line
 * Scores 1 ring in the corrner
 * Scores 1 ring from middle
 * Get positive goal, score one ring
 * Touch ladder
 */
void redNegativeHalfWP(){//PATH DONE
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = true;
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  chassis.setPose(-15.5,-58,135);
  pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);  
  /***************A-WALL STAKE**************/
  chassis.moveToPoint(-11.5,- 60, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  moveLiftToPos(550, 127, 750);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x+(-7 * sin(deg2rad(temp_pos.theta))),  temp_pos.y+(-7 * cos(deg2rad(temp_pos.theta))) , 700,{.forwards = false, .maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  /***************GOAL GRAB**************/
  chassis.moveToPoint(-24, -53, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(-24, -24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  pros::Task lift2 ([=] {moveLiftToPos(-10);});    
  chassis.moveToPoint(-24, -36,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.moveToPoint(-23.5, -19,800,{.forwards=false,.maxSpeed=speed1 - 80,.minSpeed=5,.earlyExitRange=1});
  chassis.waitUntil(10);
  closeClamp();
  pros::delay(280);
  spinIntake(127);
  startSorting();
  /***************MIDLINE RINGS**************/
  chassis.turnToHeading(310, 1000,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(-38,-7,1000,{.maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  chassis.turnToHeading(270,700,{.maxSpeed = speed,.minSpeed = 15, .earlyExitRange = 5});
  chassis.moveToPoint(-51,-7,1200,{.maxSpeed = speed1 - 40 ,.minSpeed = 10, .earlyExitRange = 1});
  /***************THIRD RING**************/
  chassis.moveToPoint(-28,-24,1500,{.forwards=false,.maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  chassis.turnToPoint(-48,-24,1500,{.maxSpeed = speed,.minSpeed = 5, .earlyExitRange = 1});
  chassis.moveToPoint(-50,-24,1500,{.maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  /***************CORNER**************/
  chassis.turnToHeading(180,1000,{.maxSpeed = speed,.minSpeed = 15, .earlyExitRange = 5});
  chassis.moveToPoint(-52,-55,1500,{.maxSpeed = speed1});
  chassis.waitUntil(10);
  pros::Task lift3 ([=] {moveLiftToPos(310);});    
  chassis.turnToHeading(225,1000,{},false);
  temp_pos = chassis.getPose();
  spinIntake(127);
  //stopSorting();
  chassis.moveToPoint(temp_pos.x-14,  temp_pos.y-14 , 700,{.maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  pros::delay(300);
  chassis.moveToPoint(temp_pos.x-2,  temp_pos.y-2, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=0.5});
  // chassis.turnToHeading(90, 600,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=2},false);
  // temp_pos = chassis.getPose();
  // chassis.moveToPoint(-6,temp_pos.y,1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  // chassis.waitUntil(10);
  // liftIntake();
  // startSorting();
  // chassis.waitUntil(1000);
  // dropIntake();
  // chassis.moveToPoint(-20.5,temp_pos.y,1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});

  /***************TOUCH LADDER**************/
  chassis.turnToPoint(0,0,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(-18,-26,2000,{.forwards=false,.maxSpeed=speed1 - 20,.minSpeed=5,.earlyExitRange=1});
  chassis.waitUntil(12);
  stopIntake();
  chassis.waitUntil(1000);
  moveLiftToPos(-10,127,200);


  master.clear_line(0);
  //*/
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }
  
}

void blueNegativeHalfWP(){
   int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = false;

  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  pros::delay(2000);
  chassis.setPose(-14.5,57,50);
  startSorting();
  pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);  
  chassis.moveToPoint(-12, 59.5, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  pros::Task lift1 ([=] {moveLiftToPos(930);});
  pros::delay(300); 
  chassis.moveToPoint(-19.5, 53, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.turnToPoint(-24, 24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=15,.earlyExitRange=5});
  pros::Task lift2 ([=] {moveLiftToPos(-10);});    
  chassis.moveToPoint(-24, 32,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.moveToPoint(-24.25, 22,500,{.forwards=false,.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.waitUntil(1000);
  closeClamp();
  pros::delay(300);
  chassis.turnToPoint(-46, 5,1000,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=3});
  chassis.moveToPoint(-45.5, 7,1000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  spinIntake(127);
  spinIntake(127);
  chassis.turnToHeading(270, 700,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=2},false);
  // chassis.swingToHeading(270, ::DriveSide::RIGHT, 2000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2},false);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(-48, temp_pos.y+0.5,1000,{.maxSpeed=speed1-40,.minSpeed=15,.earlyExitRange=2}, false);
  pros::delay(200);
  chassis.moveToPoint(-55, temp_pos.y+1,1000,{.maxSpeed=speed1-40,.minSpeed=15,.earlyExitRange=2});
  chassis.moveToPoint(-40, 16,1000,{.forwards=false,.minSpeed=15,.earlyExitRange=2});
  chassis.turnToPoint(-48, 16,1000,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=5});
  chassis.moveToPoint(-48,16,1000,{.maxSpeed=speed1,.minSpeed=14,.earlyExitRange=2});
  chassis.turnToPoint(-62,46,1000,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=5});
  pros::Task lift3 ([=] {moveLiftToPos(330);});    
  chassis.moveToPoint(-60.5,47.5,1000,{.maxSpeed=speed1-30,.minSpeed=15,.earlyExitRange=2});
  chassis.turnToHeading(315, 500,{.minSpeed=4,.earlyExitRange=0.5},false);
  temp_pos = chassis.getPose();
  spinIntake(127);
  chassis.moveToPoint(temp_pos.x-30,  temp_pos.y+30 , 700,{.maxSpeed=speed1-40,.minSpeed=45,.earlyExitRange=2},false);
  pros::delay(300);
  chassis.moveToPoint(temp_pos.x+4,  temp_pos.y-4, 1500,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=0.5});
  chassis.moveToPoint(temp_pos.x-5,  temp_pos.y+5, 1000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.moveToPoint(temp_pos.x+5,  temp_pos.y-5, 600,{.forwards=false,.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2},false);
    /*chassis.turnToHeading(90, 600,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=2},false);
  temp_pos = chassis.getPose();
  // chassis.moveToPoint(-32, temp_pos.y,1000,{.maxSpeed=speed1,.minSpeed=35,.earlyExitRange=5});
  chassis.moveToPoint(-16, temp_pos.y,1000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.waitUntil(5);
  liftIntake();
  chassis.waitUntil(1000);
  dropIntake();
  chassis.moveToPoint(-28, temp_pos.y,700,{.forwards=false,.maxSpeed=speed1,.minSpeed=4,.earlyExitRange=2});
  pros::delay(300);*/
  chassis.turnToPoint(-27, 9,1000,{.forwards=false,.minSpeed=15,.earlyExitRange=2});
  chassis.moveToPoint(-21, 9,3000,{.forwards=false,.minSpeed=5,.earlyExitRange=0.5});
  chassis.waitUntil(8);
  stopIntake();
  chassis.turnToHeading(315, 500);
  moveLiftToPos(-20,127,600);
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }
}

void negativeHalfWP(){
  if(COLOR){
    redNegativeHalfWP();
  } else {
    blueNegativeHalfWP();
  }
}

/*********************** NEGATIVE HALF WP AND WALL STAKE ***********************/

void redNegHalfWPWallStake() {//PATH DONE
  int speed = 100;
  float speed1 = float(speed);
  COLOR = true;
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  chassis.setPose(-15.5,-58,135);
  pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);
  /***************A-WALL STAKE**************/
  chassis.moveToPoint(-11.5, -60, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  //moveLiftToPos(800, 127, 750);
  moveLiftToPos(550, 127, 750);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x+(-7 * sin(deg2rad(temp_pos.theta))),  temp_pos.y+(-7 * cos(deg2rad(temp_pos.theta))) , 700,{.forwards = false, .maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  /***************GOAL GRAB**************/
  chassis.moveToPoint(-38, -48, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(-24, -24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  pros::Task lift2 ([=] {moveLiftToPos(-10);});    
  chassis.moveToPoint(-24, -24,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.waitUntil(27);
  closeClamp();
  pros::delay(280);
  spinIntake(127);
  startSorting();
  /***************MIDLINE RINGS**************/
  chassis.turnToHeading(310, 1000,{.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(-38,-7,1000,{.maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  chassis.turnToHeading(270,700,{.maxSpeed = speed,.minSpeed = 15, .earlyExitRange = 5});
  chassis.moveToPoint(-54,-5,1200,{.maxSpeed = speed1 - 40 ,.minSpeed = 10, .earlyExitRange = 1});
  /***************THIRD RING**************/
  chassis.moveToPoint(-28,-24,1500,{.forwards=false,.maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  chassis.turnToPoint(-48,-24,1500,{.maxSpeed = speed,.minSpeed = 5, .earlyExitRange = 1});
  chassis.moveToPoint(-50,-24,1500,{.maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  liftPickup();
  chassis.waitUntil(100);
  pros::delay(200);
  //Add Distance reset
  chassis.moveToPoint(-41.2, -24, 1500, {.forwards = false, .maxSpeed = speed1});
  chassis.turnToPoint(-71.25, 0, 1500, {.maxSpeed = speed}, false);
  temp_pos = chassis.getPose();
  //liftUpWallStake();
  chassis.moveToPoint(71.25 + (-11.75 * sin(deg2rad(temp_pos.theta))), 0 + (-11.75 * cos(deg2rad(temp_pos.theta))), 1500, {.maxSpeed = 60}, false);
  moveLiftToPos(310, 80, 500);
  pros::delay(300);
  /***************CORNER**************/
  chassis.moveToPoint(71.25 + (-20 * sin(deg2rad(temp_pos.theta))), 0 + (-20 * cos(deg2rad(temp_pos.theta))), 1500, {.forwards = false, .maxSpeed = speed1, .minSpeed=25,.earlyExitRange=2});
  chassis.moveToPoint(-48,-24,1500,{.forwards = false, .maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  chassis.turnToHeading(180,1000,{.maxSpeed = speed,.minSpeed = 15, .earlyExitRange = 5});
  chassis.moveToPoint(-52,-55,1500,{.maxSpeed = speed1});
  chassis.waitUntil(10);
  pros::Task lift3 ([=] {moveLiftToPos(310);});    
  chassis.turnToHeading(225,1000,{},false);
  temp_pos = chassis.getPose();
  spinIntake(127);
  //stopSorting();
  chassis.moveToPoint(temp_pos.x-14,  temp_pos.y-14 , 700,{.maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  pros::delay(300);
  chassis.moveToPoint(temp_pos.x-2,  temp_pos.y-2, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=0.5});
  // chassis.turnToHeading(90, 600,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=2},false);
  // temp_pos = chassis.getPose();
  // chassis.moveToPoint(-6,temp_pos.y,1000,{.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  // chassis.waitUntil(10);
  // liftIntake();
  // startSorting();
  // chassis.waitUntil(1000);
  // dropIntake();
  // chassis.moveToPoint(-20.5,temp_pos.y,1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  
  /***************TOUCH LADDER**************/
  chassis.turnToPoint(0,0,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  chassis.moveToPoint(-18,-26,2000,{.forwards=false,.maxSpeed=speed1 - 20,.minSpeed=5,.earlyExitRange=1});
  chassis.waitUntil(12);
  stopIntake();
  chassis.waitUntil(1000);
  moveLiftToPos(-10,127,200);


}

void negHalfWPWallStake(){
  int time = pros::millis();

  if(COLOR){
    redNegHalfWPWallStake();
  } else {
    return;
  }

  master.clear_line(0);
  //*/
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }
}

/*********************** NEGTIVE SIX RING ***********************/

/** negSixRing
 * Start at alliance stake, facing positive corner
 * Score AS
 * Get pos goal, score 3 rings by line
 * Scores 2 ring in the corrner
 * Scores 1 ring from middle
 */
void redNegSixRing(){ 
    int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = true;
  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  chassis.setPose(-15.5,-58,135);
  pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);  
  /***************A-WALL STAKE**************/
  chassis.moveToPoint(-11.5,- 60, 500,{.maxSpeed=speed1});
  chassis.waitUntil(5);
  moveLiftToPos(550, 127, 750);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(temp_pos.x+(-7 * sin(deg2rad(temp_pos.theta))),  temp_pos.y+(-7 * cos(deg2rad(temp_pos.theta))) , 700,{.forwards = false, .maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  /***************GOAL GRAB**************/
  chassis.moveToPoint(-24, -53, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});
  chassis.turnToPoint(-24, -24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=5,.earlyExitRange=1});
  pros::Task lift2 ([=] {moveLiftToPos(-30,127,1000);});    
  chassis.moveToPoint(-24, -36,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.moveToPoint(-23.5, -19,800,{.forwards=false,.maxSpeed=speed1 - 80,.minSpeed=5,.earlyExitRange=1});
  chassis.waitUntil(10);
  closeClamp();
  pros::delay(280);
  startSorting();
  chassis.waitUntil(100);
  temp_pos = chassis.getPose();
  /***********Center of field rings***********/
  chassis.turnToHeading(45, 1000,{.maxSpeed=speed,.minSpeed=2,.earlyExitRange=0.25});
  chassis.moveToPoint(-8 ,-3, 1000);
  //USE SWEEPER TO GRAB RINGS
  chassis.moveToPoint(temp_pos.x, temp_pos.y, 1000,  {.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=1});


  /***************MIDLINE RINGS**************/
  chassis.turnToHeading(325, 1000,{.maxSpeed=speed,.minSpeed=2,.earlyExitRange=0.5});
  chassis.moveToPoint(-42,-3,1000,{.maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  spinIntake(127);
  chassis.turnToHeading(270,700,{.maxSpeed = speed,.minSpeed = 2, .earlyExitRange = 0.5});
  chassis.moveToPoint(-51,-2,1200,{.maxSpeed = speed1 - 40 ,.minSpeed = 10, .earlyExitRange = 1});
  /***************THIRD RING**************/
  chassis.moveToPoint(-28,-24,1500,{.forwards=false,.maxSpeed = speed1,.minSpeed = 5, .earlyExitRange = 1});
  turnToHeadingWithVis(270,2000);
  moveToPointWithVis(-50, -24,2000);
  /***************CORNER**************/
  chassis.turnToHeading(180,1000,{.maxSpeed = speed,.minSpeed = 15, .earlyExitRange = 5});
  chassis.moveToPoint(-52,-55,1500,{.maxSpeed = speed1});
  chassis.waitUntil(10);
  pros::Task lift3 ([=] {moveLiftToPos(310);});    
  chassis.turnToHeading(225,1000,{},false);
  temp_pos = chassis.getPose();
  spinIntake(127);
  //stopSorting();
  chassis.moveToPoint(temp_pos.x-14,  temp_pos.y-14 , 700,{.maxSpeed=speed1,.minSpeed=25,.earlyExitRange=2},false);
  pros::delay(300);
  chassis.moveToPoint(temp_pos.x-2,  temp_pos.y-2, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=5,.earlyExitRange=0.5});
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }
  
}

void blueNegSixRing(){
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = false;

  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();
  chassis.setPose(-14.5,57,50);
  pros::Task intake_anti_jam(intakeAntiJamTaskFunc);
  pros::Task reset_lift_pos(resetLiftWithDistTaskFunc);  
  chassis.moveToPoint(-12.5, 59, 500,{.maxSpeed=speed1});
  chassis.waitUntil(4);
  moveLiftToPos(930);    
  pros::delay(300); 
  chassis.moveToPoint(-19.5, 53, 1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.turnToPoint(-24, 24,1000,{.forwards=false,.maxSpeed=speed,.minSpeed=15,.earlyExitRange=5});
  pros::Task lift2 ([=] {moveLiftToPos(-10);});    
  chassis.moveToPoint(-24, 32,2000,{.forwards=false,.maxSpeed =speed1,.minSpeed=25,.earlyExitRange=2});
  chassis.moveToPoint(-24, 20,600,{.forwards=false,.maxSpeed=127,.minSpeed=15,.earlyExitRange=2});
  chassis.waitUntil(2);
  closeClamp();
  pros::delay(300);
  chassis.turnToPoint(-46, 5,1000,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=3});
  chassis.moveToPoint(-46, 5,1000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  spinIntake(127);
  chassis.swingToHeading(270, ::DriveSide::RIGHT, 2000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2},false);
  temp_pos = chassis.getPose();
  chassis.moveToPoint(-55, temp_pos.y-1,1000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.moveToPoint(-34, 16,1000,{.forwards=false,.minSpeed=15,.earlyExitRange=2});
  chassis.turnToPoint(-48, 16,1000,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=5});
  chassis.moveToPoint(-48,16,1000,{.maxSpeed=speed1,.minSpeed=14,.earlyExitRange=2});
  chassis.turnToPoint(-62,46,1000,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=5});
  pros::Task lift3 ([=] {moveLiftToPos(330);});    
  chassis.moveToPoint(-62,46,1000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.turnToHeading(315, 500,{.minSpeed=4,.earlyExitRange=0.5},false);
  temp_pos = chassis.getPose();
  spinIntake(127);
  chassis.moveToPoint(temp_pos.x-30,  temp_pos.y+30 , 1000,{.maxSpeed=speed1,.minSpeed=45,.earlyExitRange=2},false);
  pros::delay(200);
  chassis.moveToPoint(temp_pos.x+3,  temp_pos.y-3, 1500,{.forwards=false,.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.moveToPoint(temp_pos.x-5,  temp_pos.y+5, 1000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.moveToPoint(temp_pos.x+5,  temp_pos.y-3.5, 1500,{.forwards=false,.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  chassis.turnToHeading(90,600,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=2},false);
  temp_pos = chassis.getPose();
  chassis.turnToPoint(-24, temp_pos.y,500,{.maxSpeed=speed,.minSpeed=15,.earlyExitRange=5});
  chassis.moveToPoint(-32, temp_pos.y,1000,{.maxSpeed=speed1,.minSpeed=15,.earlyExitRange=2});
  liftIntake();
  chassis.moveToPoint(-20, temp_pos.y,1000,{.maxSpeed=speed1-50,.minSpeed=15,.earlyExitRange=2});
  chassis.waitUntil(1000);
  pros::delay(200);
  dropIntake();
  chassis.moveToPoint(-32, temp_pos.y,1000,{.forwards=false,.maxSpeed=speed1,.minSpeed=4,.earlyExitRange=2},false);
  pros::delay(200);
  chassis.turnToPoint(-30, 15,1000,{.forwards=false,.minSpeed=15,.earlyExitRange=2});
  chassis.moveToPoint(-30, 15,3000,{.forwards=false,.minSpeed=15,.earlyExitRange=2});
  moveLiftToPos(160);
  chassis.waitUntil(8);
  stopIntake();
  chassis.turnToHeading(315, 1000);
  moveLiftToPos(0,127,200);

  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }

}

void negSixRing(){
  if(COLOR){
    redNegSixRing();
  } else {
    blueNegSixRing();
  }
}


/*********************** NEGATIVE SIX RING AND BOTH STAKE ***********************/
void redSafeSixRing(){
   int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = true;
  stopSorting();

  chassis.setPose(-24,-53,180);
  openClamp();
  chassis.moveToPoint(-24,-24,2000,{.forwards=false,.maxSpeed = 80,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(10000);
  closeClamp();
  spinIntake(127);

  chassis.turnToPoint(-43,-24,1000,{.maxSpeed = 127,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-43,-24,1000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  startSorting();

  //FIRST RING
  chassis.turnToPoint(-43,-10,1000,{.maxSpeed = 127,.minSpeed = 20, .earlyExitRange = 1});
  chassis.moveToPoint(-43,-10.25,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 6});

  chassis.moveToPoint(-48,-24,2000,{.forwards=false,.maxSpeed = 127,.minSpeed = 30, .earlyExitRange = 2});
  spinIntake(-127);
  pros::delay(40);
  spinIntake(127);

  chassis.turnToPoint(-48,-10,1000,{.maxSpeed = 127,.minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(-48,-11.75,1000,{.maxSpeed = speed1, .minSpeed = 12, .earlyExitRange = 2},false);
  pros::delay(300);

  chassis.moveToPoint(-48,-52,2000,{.forwards=false,.maxSpeed = 127-30,.minSpeed = 30, .earlyExitRange = 6});
  spinIntake(-127);
  pros::delay(40);
  spinIntake(127);
  chassis.waitUntil(18);
  chassis.turnToPoint(-72, -74, 1000,{.maxSpeed = 127, .minSpeed = 20, .earlyExitRange = 5});
  chassis.moveToPoint(-64,-65,600,{.maxSpeed = speed1, .minSpeed = 20, .earlyExitRange = 5});
  chassis.moveToPoint(-72,-74,600,{.maxSpeed = speed1-30});
  chassis.moveToPoint(-54,-50,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 20, .earlyExitRange = 8},false);
  chassis.turnToPoint(0,-48,2000,{.maxSpeed = 127,.minSpeed = 20, .earlyExitRange = 8});
  chassis.waitUntil(24);
   
  chassis.moveToPoint(-8,-51,1400,{.maxSpeed = 80});
  chassis.waitUntil(10000);
   
  pros::delay(100);
  chassis.moveToPoint(-24,-48,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.turnToPoint(-15,-23,2000,{.maxSpeed = 127,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-13,-18,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.turnToHeading(45, 1000,{},false);
  master.clear_line(0);
  //*/
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }
  
}
void blueSafeSixRing(){
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = false;

  stopSorting();
  lemlib::Pose temp_pos = chassis.getPose();

  chassis.setPose(-24,53,0);
  openClamp();
  chassis.moveToPoint(-24,26,2000,{.forwards=false,.maxSpeed = 80,.minSpeed = 20, .earlyExitRange = 2},false);
  pros::delay(100);
  closeClamp();
  startSorting();
  spinIntake(127);
  
  chassis.turnToPoint(-43,24,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-43,21,1000,{.maxSpeed = speed1,.minSpeed = 20, .earlyExitRange = 4});
  chassis.moveToPoint(-47,21,1000,{.maxSpeed = speed1 - 20,.minSpeed = 10, .earlyExitRange = 2});
  chassis.turnToPoint(-42,11,1300,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-43,7.25,1300,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  pros::delay(250);
  chassis.moveToPoint(-48,24,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(-48,8.5,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  pros::delay(250);
  chassis.moveToPoint(-50,43,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  speed = 127;
  speed1 = float(speed);
  temp_pos = chassis.getPose();

  chassis.turnToHeading(315,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  //startSorting();
  chassis.moveToPoint(temp_pos.x - 22 ,temp_pos.y + 22,1000,{.maxSpeed = 127,.minSpeed = 30, .earlyExitRange = 2});
  //stopSorting();
  chassis.moveToPoint(-50,48,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  chassis.turnToPoint(0,48,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  //startSorting();
  chassis.moveToPoint(-15,44,2000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
   
  chassis.moveToPoint(-9,43,2000,{.maxSpeed = speed1-40,.minSpeed = 10, .earlyExitRange = 0.5});
  chassis.waitUntil(10000);
  pros::delay(300);
  
  chassis.moveToPoint(-24,48,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.turnToPoint(-16,16,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-14,14,2000,{.maxSpeed = speed1,.minSpeed = 20, .earlyExitRange = 8});

  chassis.turnToHeading(145, 800,{},false);
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }

}

void safeSixRing(){
  if(COLOR){
    redSafeSixRing();
  } else {
    blueSafeSixRing();
  }
}

/*********************** SOLO WP ***********************/

/** solo_wp
 * Start at alliance stake, facing positive corner
 * Score AS
 * Get Neg goal, score 2 rings by line
 * Scores 1 ring corrner
 * Drop goal by middle, picks up middle ring
 * Get positive goal, score 2 ring
 * Touch ladder
 */
void red_solo_wp(){ 
  int time = pros::millis();
  int speed = 100;
  float speed1 = float(speed);
  int temp_pos = 0;
  
  stopSorting();
  chassis.setPose(0.5,-59,90);
  openClamp();
  chassis.moveToPoint(-9,-60,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(1.25);
  
  chassis.moveToPoint(-19,-36,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 6},false);
 
  chassis.moveToPoint(-25,-24,2000,{.forwards=false,.maxSpeed = 80,.minSpeed = 30, .earlyExitRange = 6},false);
  pros::delay(200);
  closeClamp();
  pros::delay(200);
  spinIntake(127);

  chassis.turnToPoint(-42,-25,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-42,-25,1000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});

  chassis.turnToPoint(-41,-11,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-41,-8,1000,{.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  chassis.moveToPoint(-50,-52,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  /*
  chassis.moveToPoint(-50,-8,1000,{.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  chassis.moveToPoint(-50,-52,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  //*/
  chassis.turnToPoint(-66, -70, 2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-66,-70,1000,{.maxSpeed = 127,.minSpeed = 60, .earlyExitRange = 5});

  chassis.moveToPoint(-32, -54, 2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  chassis.turnToHeading(90, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8},false);
  temp_pos = chassis.getPose().y;
  stopSorting();
  //chassis.moveToPoint(15,-43,2000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  chassis.moveToPoint(-7,temp_pos,1000,{.maxSpeed = 80});
   
  saveOurRing(2000);
  chassis.moveToPoint(15,temp_pos,2000,{.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  toggleClamp();
  chassis.turnToPoint(24,-27,1000,{.forwards=false,.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(24,-28,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6},false);
  toggleClamp();
  pros::delay(200);
  spinIntake(127);
  
  chassis.turnToHeading(90, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.waitUntilDone();
  speed1 = 127;
  temp_pos = chassis.getPose().y;
  chassis.moveToPoint(50,temp_pos-4,1000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 2});
  chassis.turnToPoint(18, temp_pos-4,1500,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 3});
  chassis.moveToPoint(22, 14, 2000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 3});
  chassis.turnToHeading(315, 1000);

  /*
  chassis.turnToHeading(90,1000);
  chassis.moveToPoint(48,-24,2000);
  chassis.waitUntilDone();
  chassis.moveToPoint(11,-11,3000);
  chassis.turnToHeading(135,1000);

//*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void blue_solo_wp(){
  int time = pros::millis();
  int speed = 100;
  float speed1 = float(speed);
  int temp_pos = 0;
  chassis.setPose(0,60,90);

  startSorting();
  openClamp();
  chassis.moveToPoint(-6,60,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(1.25);
  chassis.waitUntil(6);
  chassis.moveToPoint(-24,30,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 6});
  chassis.moveToPoint(-24,26,2000,{.forwards=false,.maxSpeed = speed1-60,.minSpeed = 30, .earlyExitRange = 2},false);
  closeClamp();
  pros::delay(300);
  spinIntake(127);

  chassis.turnToPoint(-43,24,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-43,24,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});

  chassis.turnToPoint(-41,11,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-42,3,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  pros::delay(300);
  chassis.moveToPoint(-45,45,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 5});
  /*
  chassis.moveToPoint(-50,-8,1000,{.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  chassis.moveToPoint(-50,-52,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  //*/
  chassis.turnToPoint(-72, 70, 2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-74,72,1000,{.maxSpeed = 127,.minSpeed = 30, .earlyExitRange = 5});

  chassis.moveToPoint(-44, 44, 2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  chassis.turnToHeading(90, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8},false);
  temp_pos = chassis.getPose().y;
  //chassis.moveToPoint(15,-43,2000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  chassis.moveToPoint(-9,temp_pos-2,1500,{.maxSpeed = 80});
  stopSorting();
  chassis.waitUntil(12);
  pros::Task savering([=] {saveOurRing(2000);});    
  chassis.waitUntil(10000);
  
  chassis.moveToPoint(15,temp_pos,2000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 5});
  toggleClamp();
  chassis.turnToPoint(21,27,1000,{.forwards=false,.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(16,29,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 2});
  chassis.moveToPoint(16,24,500,{.forwards=false,.maxSpeed = speed1-60,.minSpeed = 10, .earlyExitRange = 2});
  chassis.waitUntil(10000);
  toggleClamp();
  startSorting();
  pros::delay(200);
  chassis.turnToHeading(90, 1400,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 4});
  chassis.waitUntilDone();
  pros::delay(200);
  spinIntake(127);
  temp_pos = chassis.getPose().y;
  chassis.moveToPoint(43,temp_pos,1000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 2});
  chassis.turnToPoint(14, temp_pos-8,1500,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 3});
  chassis.moveToPoint(13, temp_pos-9, 2000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 3});
  chassis.turnToHeading(225, 1000);

  /*
  chassis.turnToHeading(90,1000);
  chassis.moveToPoint(48,-24,2000);
  chassis.waitUntilDone();
  chassis.moveToPoint(11,-11,3000);
  chassis.turnToHeading(135,1000);

//*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void soloWP(){
  if(COLOR){
    red_solo_wp();
  } else {
    blue_solo_wp();
  }
}

/*********************** END OF ALL WISCO SIG ***********************/

void auton_60s_skills_1() {
  COLOR = true;
  stopSorting();
  int time = pros::millis();
  float speed = 130;
  int speed1 = (int) speed;
  int temp = 0;
  chassis.setPose(0,-61.3,0);
  lemlib::Pose currentPose = chassis.getPose();
  stopSorting();
  spinIntake(127);
  pros::delay(500);

  chassis.moveToPoint(0, -48, 800, {.maxSpeed = speed},false);
  currentPose = chassis.getPose();
  chassis.turnToPoint(-24, currentPose.y, 1500, {.forwards = false, .maxSpeed = speed1});
  stopIntake();
  chassis.waitUntil(1000);
  currentPose = chassis.getPose();
  chassis.moveToPoint(-24, currentPose.y, 1500, {.forwards = false, .maxSpeed = speed - 40});
  chassis.waitUntil(19.5);
  closeClamp();

  printPositionV2((char *) "Goal clamp"); 
  chassis.turnToPoint(-24, -24, 1000, {.maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  spinIntake(127);
  chassis.moveToPoint(-24, -24, 1500, {.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  printPositionV2((char *) "1st ring");
  chassis.turnToPoint(-48, 24, 1000,{.maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(-48, 24, 1500, {.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25});
  liftPickup();
  chassis.moveToPoint(-36, 0, 1500, {.forwards=false,.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25});
  turnToHeadingWithVis(90,1000);
  liftUpWallStake();
  chassis.moveToPoint(-65, 0, 1000);
  //hit wall stake
  chassis.moveToPoint(-48, 0, 1500, {.forwards=false,.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  moveLiftToPos(0);
  chassis.turnToPoint(-48, -24, 1000, {.maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(-50, -22, 1500, {.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  printPositionV2((char *) "2nd ring");
  chassis.turnToPoint(-50, -48, 1500, {.maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(-50, -48, 2000, {.maxSpeed = 60,}, false);
  pros::delay(200);
  chassis.moveToPoint(-50, -57, 2000, {.maxSpeed = 60,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  printPositionV2((char *) "4th and 5th ring");
  chassis.turnToPoint(-42, -48, 1000, {.forwards = false, .maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(-42, -48, 1000, {.forwards = false, .maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.turnToPoint(-60, -48, 1000, {.maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(-57, -48, 1000, {.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  printPositionV2((char *) "6th ring");
  pros::delay(350);
  chassis.turnToPoint(-68, -72, 1500, {.forwards = false, .maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(-65, -62, 1000, {.forwards = false, .maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  openClamp();
  spinIntake(-127);
  printPositionV2((char *) "Goal drop");
  pros::delay(300);

  chassis.moveToPoint(-57, -48, 1000, {.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.turnToHeading(0, 1000, {.maxSpeed = speed1}, false);
  pros::delay(50);
  currentPose = chassis.getPose();
  chassis.setPose(-68.5 + fabs(distToWallL() * cos(deg2rad(currentPose.theta))), -70 + fabs(distToWallB() * cos(deg2rad(currentPose.theta))), currentPose.theta);
  pros::delay(50);
  printPositionV2((char *) "Reset");

  chassis.turnToPoint(24, -24, 1000, {.forwards = false, .maxSpeed = speed1});
  chassis.moveToPoint(24, -24, 3000, {.forwards = false, .maxSpeed = speed - 10}); 
  chassis.turnToPoint(24, -48, 2000, {.forwards = false, .maxSpeed = speed1});
  chassis.moveToPoint(24, -48.4, 1000, {.forwards = false, .maxSpeed = speed - 40});
  chassis.waitUntil(22);
  closeClamp();
  printPositionV2((char *) "Goal clamp");

  printPositionV2((char *) "Goal clamp"); 
  chassis.turnToPoint(24, -24, 1000, {.maxSpeed = speed1}, false);
  spinIntake(127);
  chassis.moveToPoint(24, -24, 1500, {.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  printPositionV2((char *) "1st ring");
  chassis.turnToPoint(48, 24, 1000,{.maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(48, 24, 1500, {.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25});
  liftPickup();
  chassis.moveToPoint(36, 0, 1500, {.forwards=false,.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25});
  turnToHeadingWithVis(90,1000);
  liftUpWallStake();
  chassis.moveToPoint(65, 0, 1000);
  //hit wall stake
  chassis.moveToPoint(48, 0, 1500, {.forwards=false,.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  moveLiftToPos(0);

  chassis.turnToPoint(48, -24, 1000, {.maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(50, -22, 1500, {.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  printPositionV2((char *) "2nd ring");
  chassis.turnToPoint(50, -48, 1500, {.maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(50, -44, 2000, {.maxSpeed = 60}, false);
  pros::delay(200);
  chassis.moveToPoint(50, -56, 2000, {.maxSpeed = 60,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  printPositionV2((char *) "4th and 5th ring");
  chassis.turnToPoint(42, -48, 1000, {.forwards = false, .maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(42, -48, 1000, {.forwards = false, .maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.turnToPoint(60, -48, 1000, {.maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(57, -48, 1000, {.maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  printPositionV2((char *) "6th ring");
  pros::delay(350);
  chassis.turnToPoint(68, -72, 1500, {.forwards = false, .maxSpeed = speed1,.minSpeed = 3,.earlyExitRange = 0.25});
  chassis.moveToPoint(65, -62, 1000, {.forwards = false, .maxSpeed = speed,.minSpeed = 3,.earlyExitRange = 0.25}, false);
  openClamp();
  spinIntake(-127);
  printPositionV2((char *) "Goal drop");
  pros::delay(300);

  chassis.moveToPoint(54, -48, 1000, {.maxSpeed = speed});
  chassis.turnToHeading(0, 1000, {.maxSpeed = speed1}, false);
  pros::delay(50);
  currentPose = chassis.getPose();
  chassis.setPose(68.5 - fabs(distToWallR() * cos(deg2rad(currentPose.theta))), -70 + fabs(distToWallB() * cos(deg2rad(currentPose.theta))), currentPose.theta);
  pros::delay(50);
  printPositionV2((char *) "Reset");
  chassis.moveToPoint(34, 34, 3000,{},false);
  chassis.turnToHeading(180, 1300,{},false);//*/
  currentPose = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallL() * cos(deg2rad(currentPose.theta))), 72 - fabs(distToWallB() * cos(deg2rad(currentPose.theta))), currentPose.theta);
  pros::delay(200);
  chassis.turnToPoint(26, 60, 1000,{.forwards = false});
  chassis.moveToPoint(26, 60, 1500,{.forwards = false,.maxSpeed = speed - 20});
  chassis.waitUntil(1000);
  closeClamp();
  chassis.turnToPoint(68, 68, 1000,{.forwards = false},false);
  chassis.arcade(-40, 0);
  pros::delay(600);
  openClamp();
  chassis.arcade(40, 0);
  pros::delay(600);
  chassis.turnToPoint(72, 65, 1000,{});
  chassis.moveToPoint(72, 65, 1000,{});
  chassis.moveToPoint(40, 59, 2000,{.forwards = false});
  chassis.turnToHeading(270, 1000);
  chassis.setPose(72 - fabs(distToWallB() * sin(deg2rad(currentPose.theta))), 72 - fabs(distToWallR() * sin(deg2rad(currentPose.theta))), currentPose.theta);
  chassis.moveToPoint(28, -60, 1000,{});
  chassis.turnToPoint(72, -65, 1000);
  chassis.moveToPoint(72, -65, 1800);
  chassis.moveToPoint(30, -59, 1800,{.forwards = false});


  master.clear_line(0);
  temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
} // end auton_60s_skills_1()
// Auton skills number 2 fully done
void auton_60s_skills_2() {
    COLOR = true;
  stopSorting();
  int time = pros::millis();
  float speed = 127;
  int temp = 0;
  chassis.setPose(1,-60.5,90);
  stopSorting();


  /*
  DONT 
  CHANGE 
  FIRST 
  HALF 
  OF 
  SKILLS  
  */

  lemlib::Pose currentPose = chassis.getPose();

  
  chassis.moveToPoint(-5, -60.5,1000,{.forwards=false, .minSpeed = 40, .earlyExitRange = 3});
  chassis.waitUntil(0.9);
 
  chassis.moveToPoint(-24, -48, 2000,{.forwards=false, .maxSpeed=speed-30, .minSpeed = 40, .earlyExitRange = 3});
  chassis.waitUntil(17);
 
  spinIntake(-127);
  closeClamp();
  printPositionV2((char *) "Goal #1 pickup",false,false,time);
  chassis.turnToPoint(-24, -24,1000, {.maxSpeed = (int) speed, .minSpeed = 40, .earlyExitRange = 8});
  spinIntake(127);
  chassis.moveToPoint(-23, -24, 2000,{.maxSpeed=speed, .minSpeed = 40, .earlyExitRange = 8});
  chassis.waitUntil(12);
  spinIntake(127);
  chassis.turnToHeading(-90, 1000, {.maxSpeed = (int) speed, .minSpeed = 40, .earlyExitRange = 8});
  chassis.moveToPoint(-48, -24, 2000,{.maxSpeed=speed, .minSpeed =40, .earlyExitRange = 7});
  chassis.turnToPoint(-56, 0, 1000, {.maxSpeed = (int) speed, .minSpeed = 40, .earlyExitRange = 8});
  chassis.moveToPoint(-56, 3, 1000,{.maxSpeed=speed, .minSpeed = 40, .earlyExitRange = 7});
  chassis.waitUntil(26);
  //printPosition((char *)"Ring by stake", false);
  chassis.turnToPoint(-48, 24, 1000, {.maxSpeed = (int) speed, .minSpeed = 40, .earlyExitRange = 8});
   
  chassis.moveToPoint(-46, 28, 1000,{.maxSpeed=speed-10, .minSpeed = 20, .earlyExitRange = 0.5});
   
  //printPosition((char *)"Basket ring", false);
  chassis.waitUntil(1000);
  pros::delay(300);
  chassis.moveToPoint(-48, 3.8, 1400,{.forwards = false, .maxSpeed=speed - 15, .minSpeed = 5, .earlyExitRange = 0.5});
  chassis.turnToHeading(270,1000, {.maxSpeed = (int) speed});
   
  liftUpWallStake();
  chassis.waitUntil(100);
   
  currentPose = chassis.getPose();
  chassis.setPose(-72 + fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  pros::delay(10);
  //printPosition((char *)"Facing wall stake", false);
  chassis.moveToPoint(-61, currentPose.y+0.3, 1000, {.maxSpeed = speed, .minSpeed = 7, .earlyExitRange = 0.5}, false);
  //printPosition((char *)"At stake", false);//*/
  int lift_angle_down = 78;
  int x = 0;
  int angle = 7;
  lemlib::Pose placehold = chassis.getPose();
  pros::Task lift_wall_stake([=] {moveLiftToPos(lift_angle_down);}); 

    while(x<2 || getLiftPosition() > lift_angle_down)
    {
      chassis.turnToHeading(placehold.theta + angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 3});
      chassis.turnToHeading(placehold.theta - angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 3});
      x++;
    }
    x=0;
  printPositionV2((char *) "Wall Stake #1",false,false,time);
   
  //printf("\n Time at 1st stake: %d", pros::millis() - time);
  chassis.moveToPoint(-52,0,1000,{.forwards=false, .maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5},false);
  currentPose = chassis.getPose();
  chassis.setPose(-72 + fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  pros::delay(50);
  //printPosition((char *)"After reset", false);
  chassis.turnToHeading(180,800, {.maxSpeed = (int) speed, .minSpeed = 30, .earlyExitRange = 5});
  pros::Task lift_down(liftDown);
   
  pros::delay(200);

  chassis.moveToPoint(-48, -48, 2000,{.maxSpeed=speed, .minSpeed = 50, .earlyExitRange = 6});
  chassis.waitUntil(5);
  spinIntake(127);
  spinIntake(127);
  spinIntake(127);
  spinIntake(127);
  spinIntake(127);
  chassis.moveToPoint(-48, -56, 2000,{.maxSpeed=speed-40, .minSpeed = 50, .earlyExitRange = 4});
   spinIntake(127);

  chassis.moveToPoint(-40, -48, 2000,{.forwards=false, .maxSpeed = speed, .minSpeed = 50, .earlyExitRange = 4});
  //printPosition((char *)"4th and 5th rings", false);
  chassis.turnToHeading(270, 1000, {.maxSpeed = (int) speed, .minSpeed = 40, .earlyExitRange = 8});
  chassis.moveToPoint(-58, -46, 2000,{.maxSpeed=speed, .minSpeed = 40, .earlyExitRange = 5});
  //printPosition((char *)"6th ring", false);

  chassis.turnToPoint(-72, -72, 1000,{.forwards=false, .maxSpeed = (int) speed - 10, .minSpeed = 10, .earlyExitRange = 3}, false);

  currentPose = chassis.getPose();
  chassis.moveToPoint(currentPose.x-2,currentPose.y-10 ,1000,{.forwards=false,.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3},false);
  pros::delay(600);
  openClamp();

  printPositionV2((char *) "Goal #1 Drop",false,false,time);
  //printf("\n Time at 1st goal: %d", pros::millis() - time);
  chassis.moveToPoint(-52, -52, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  
  chassis.turnToHeading(270,1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 1},false);
  currentPose = chassis.getPose();
  //grabbing second goal
  chassis.moveToPoint(10, -48, 4000,{.forwards = false, .maxSpeed = speed, .minSpeed = 60, .earlyExitRange = 4});
  chassis.moveToPoint(21, -48, 2000,{.forwards=false, .maxSpeed = speed - 50, .minSpeed = 10, .earlyExitRange = 0.5});
  chassis.waitUntil(22);
  closeClamp();
  printPositionV2((char *) "Goal #2 Pickup",false,false,time);
  chassis.turnToPoint(22,-24,1000, {.maxSpeed = (int) speed, .minSpeed = 50, .earlyExitRange = 5});
  chassis.moveToPoint(22, -24, 2000,{.maxSpeed=speed, .minSpeed = 50, .earlyExitRange = 8});
  chassis.turnToHeading(90,1000, {.maxSpeed = (int) speed, .minSpeed = 50, .earlyExitRange = 5});
  chassis.moveToPoint(48, -24, 2000,{.maxSpeed=speed, .minSpeed = 50, .earlyExitRange = 8});
  chassis.turnToPoint(56, 0,1000, {.maxSpeed = (int) speed, .minSpeed = 50, .earlyExitRange = 5});
  chassis.moveToPoint(56, 4, 2000,{.maxSpeed=speed-10, .minSpeed = 50, .earlyExitRange = 7});

  chassis.turnToPoint(48,24,2000, {.maxSpeed = (int) speed, .minSpeed = 50, .earlyExitRange = 3});
   
  chassis.moveToPoint(48, 24, 2000,{.maxSpeed=speed-10, .minSpeed = 20, .earlyExitRange = 0.5});
   
  //printPosition((char *)"Basket ring", false);
  chassis.waitUntil(1000);
  pros::delay(300);

  chassis.moveToPoint(46, 1.7, 1400,{.forwards=false, .maxSpeed=speed,.minSpeed = 12, .earlyExitRange = 0.5});
  chassis.turnToHeading(90,600, {.maxSpeed = (int)speed});
  liftUpWallStake();
   
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  pros::delay(10);
  //printPosition((char *)"Facing 2nd Stake", false);
  chassis.moveToPoint(62, currentPose.y+0.5, 800,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 1}, false);
  //printPosition((char *)"At 2nd Wall Stake", false);
  pros::Task lift_wall_stake1([=] {moveLiftToPos(lift_angle_down);});    
    placehold = chassis.getPose();
    while(x<2 || getLiftPosition() > lift_angle_down)
    {
      chassis.turnToHeading(placehold.theta + 10, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      chassis.turnToHeading(placehold.theta - 10, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      x++;
    }
    x=0;
  printPositionV2((char *) "Wall Stake #2",false,false,time);
  // printf("\n Time at 2nd wallstake: %d", pros::millis() - time);
  chassis.moveToPoint(48,currentPose.y,1000,{.forwards=false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5},false);
  currentPose = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  pros::delay(10);
  // printPosition((char *)"After 2nd reset", false);
  chassis.turnToHeading(180,1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 6});
  pros::Task lift_down1(liftDown);
   
  pros::delay(200);
  chassis.moveToPoint(48, -59, 2000,{.maxSpeed = speed - 10, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.waitUntil(5);
  spinIntake(127);
  spinIntake(127);
  spinIntake(127);
  spinIntake(127);
  spinIntake(127);
  chassis.moveToPoint(40, -45, 2000,{.forwards = false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  spinIntake(127);
  // printPosition((char *)"4th & 5th rings", false);
  chassis.moveToPoint(60, -48, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  // printPosition((char *)"6th ring", false);
  chassis.turnToPoint(72, -72, 1000,{.forwards=false, .maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3},false);
  currentPose = chassis.getPose();
  chassis.moveToPoint(currentPose.x+4,currentPose.y-9 ,1000,{.forwards=false,.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3},false);
  openClamp();
  pros::delay(500);
  printPositionV2((char *) "Goal #2 Drop",false,false,time);
  //chassis.moveToPoint(48, -48, 2000,{.maxSpeed = speed, .minSpeed = speed - 30, .earlyExitRange = 0.5});
  //chassis.turnToPoint(48,0, 1000,{.maxSpeed = (int) speed, .minSpeed = 30, .earlyExitRange = 0.5});
  chassis.moveToPoint(48, 0, 3000, {.maxSpeed = speed, .minSpeed  = 40, .earlyExitRange = 6});
  chassis.turnToPoint(24, 24, 1000,{.maxSpeed = (int)speed, .minSpeed = 40, .earlyExitRange = 3});
  chassis.waitUntil(10000);
  chassis.moveToPoint(24, 24, 1900, {.maxSpeed = speed,.minSpeed=40,.earlyExitRange=3});
   
   
  chassis.waitUntil(100);
  pros::delay(600);
  chassis.turnToPoint(0, 48, 1000, {.forwards = false, .maxSpeed = (int)speed - 50, .minSpeed = 30, .earlyExitRange = 3});

  // printPosition((char *)"3rd Basket", false);
  chassis.moveToPoint(8, 40, 4000,{.forwards = false, .maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3});

  chassis.moveToPoint(2, 47, 500, {.forwards = false, .maxSpeed = speed-30},false);
  //*/


  closeClamp();
   
  pros::delay(150);

  pros::Task lift_wall_stake2([=] {moveLiftToPos(55);}); 
  printPositionV2((char *) "Goal #3 Pickup",false,false,time);
  chassis.turnToHeading(0, 900, {.maxSpeed = (int)speed, .earlyExitRange = 1},false);
   
  currentPose = chassis.getPose();
  pros::delay(50);
  chassis.setPose(currentPose.x, 72 - fabs(distToWallF() * cos(deg2rad(currentPose.theta))) , currentPose.theta);
  pros::delay(10);
  //printPositionV2((char *)"Facing Blue WS");
  pros::delay(50);
   
  chassis.moveToPoint(currentPose.x+0.4, 57.5, 1000, {.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 0.5}, false);
  //printPosition((char *)"At Blue WS", false);
  lift_angle_down = 30;
  angle = 6;
  pros::Task lift_wall_stake3([=] {moveLiftToPos(lift_angle_down);});    
    placehold = chassis.getPose();
    while(x<1)
    {
      chassis.turnToHeading(placehold.theta + angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      chassis.turnToHeading(placehold.theta - angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      x++;
    } 
    x=0;
  printPositionV2((char *) "Wall Stake #3",false,false,time);
  chassis.moveToPoint(currentPose.x, 48, 1000, {.forwards = false,.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5},false);
  liftDown();
    
  pros::delay(50);
  chassis.waitUntilDone();
  spinIntake(-127);

  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, 72 - fabs(distToWallF() * cos(deg2rad(currentPose.theta))) , currentPose.theta);
  pros::delay(10);
  //printPositionV2((char *)"After Blue WS", false, false, time);
  chassis.turnToPoint(-22, 28, 500, {.maxSpeed = (int) speed});
  master.rumble(" - ");
  chassis.waitUntil(1000);
}

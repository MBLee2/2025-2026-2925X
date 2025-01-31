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
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
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

auton_routine goal_rush{0, 0, 0, "None - Invalid Routine", &goalRush};

auton_routine goal_rush_and_stake{0, 0, 0, "None - Invalid Routine", &goalRushWallStake};

auton_routine safe_negative{0, 0, 0, "None - Invalid Routine", &negativeHalfWP}; //EVERYTHING DONE

auton_routine negetive_6_ring{0, 0, 0, "None - Invalid Routine", &negSixRing};

auton_routine negetive_6_ring_wall_stake{0, 0, 0, "None - Invalid Routine", 
                                         &negSixRingWallStake};

auton_routine negetive_6_ring_both_stake{0, 0, 0, "None - Invalid Routine", 
                                         &negSixRingBothStake};

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
                      bool detailedDist = false) {
  lemlib::Pose currentPose = chassis.getPose();
  printf("%s\tX: %3.2f\tY: %3.2f\tTheta: %3.2f\n", msg, currentPose.x,
         currentPose.y, currentPose.theta);
  if (withDistanceSensors) {
    if (detailedDist) {
      printf("\nF: %d\tL: %d\n",
             distance_front.get(), distance_left.get());
    }
    printf("\nFront: %3.2f\tLeft: %3.2f\n",
           distToWallF(), distToWallL());
    printf("\nFront position: %3.2f\tLeft position: %3.2f\n",
            72 - distToWallF() * cos(deg2rad(fmodf(currentPose.theta + 360, 90))), 72 - distToWallL() * cos(deg2rad(fmodf(currentPose.theta + 360, 90))));
  }
}

/*********************** THESE ARE ALL THE AUTONS FOR WISCO SIG ***********************/

/*********************** POSSITIVE HALF WP ***********************/


/** positiveHalfWp
 * Starts at wall stake, facing towards negative corner
 * Preload in alliance stake device
 * 
 * Alliance stake
 * 3/4 rings on one goal
 * Touch bar
 */
void redPositiveHalfWP(){ //EVERYTHING DONE
  int time = pros::millis();
  int speed = 123;
  float speed1 = float(speed);
  int temp_pos1 = 0;
  int temp_pos2 = 0;

  stopSorting();
  
  chassis.setPose(-1, -58.5, 270);
  temp_pos2=chassis.getPose().y;

  liftPneumaticUp();
  pros::delay(200);
  chassis.moveToPoint(20, temp_pos2, 900, {.forwards = false, .minSpeed = 12, .earlyExitRange = 0.5}, false);
  liftPneumaticDown();
  
  chassis.turnToPoint(24, -24, 2000, {.forwards = false, .maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(24, -33.5, 4000,{.forwards = false, .maxSpeed = speed1, .minSpeed = 10, .earlyExitRange = 3});
  chassis.moveToPoint(24, -24, 2000,{.forwards=false, .maxSpeed = speed1 - 30, .minSpeed = 6, .earlyExitRange = 2});
  spinIntake(127);
  chassis.waitUntil(14);
  closeClamp();
  pros::delay(10);
  printPositionV2((char *) "Goal pickup");

  chassis.turnToPoint(43, -24, 2000, {.maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(45, -24, 1500,{.minSpeed = 12, .earlyExitRange = 2}, false);
  openRedirectAfterOurRing(2000);
  printPositionV2((char *) "1st Ring");
  chassis.turnToPoint(48, -48, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 8}); 
  chassis.moveToPoint(48, -46, 1000,{.maxSpeed = speed1});
  chassis.turnToHeading(220, 1000,{});
  pros::delay(1000);
  chassis.turnToHeading(135, 2000, {.maxSpeed = speed, .minSpeed = 8, .earlyExitRange = 2},false);
  closeRedirect();
  startSorting();
  spinIntake(127);
  temp_pos1 = chassis.getPose().x;
  temp_pos2 = chassis.getPose().y;
  chassis.moveToPoint(temp_pos1 + 20, temp_pos2 - 20, 2000,{.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(48, -50, 2000,{.forwards= false,.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  liftIntake();
  chassis.turnToHeading(270, 1000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(27, -50, 3000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(17, -51, 1000,{.maxSpeed=50,.minSpeed = 30, .earlyExitRange = 6});
  chassis.waitUntil(10000);
  dropIntake();

  chassis.moveToPoint(30, -50, 3000,{.forwards=false,.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  speed = 80;
  speed1 = int(speed);
  chassis.turnToPoint(25, -22, 1000);
  chassis.moveToPoint(25, -22, 1000,{.maxSpeed = speed1});
  chassis.turnToHeading(315, 1000,{.maxSpeed=speed},false);

  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }

}

void bluePositiveHalfWP(){ //EVERYTHING DONE
  int time = pros::millis();
  int speed = 123;
  float speed1 = float(speed);
  int temp_pos1 = 0;
  int temp_pos2 = 0;
  COLOR = false;
  
  startSorting();
  chassis.setPose(-1, 58.5, 270);
  temp_pos2=chassis.getPose().y;

  liftPneumaticUp();
  pros::delay(200);
  chassis.moveToPoint(20, temp_pos2, 900, {.forwards = false, .minSpeed = 30, .earlyExitRange = 8}, false);
  liftPneumaticDown();
  chassis.moveToPoint(48, 48, 900, {.forwards = false, .minSpeed = 12, .earlyExitRange = 0.5}, false);
  spinIntake(127);
  chassis.turnToPoint(48, 24, 2000, {.maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(48, 24, 1500,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 2});
  saveOurRing(3000);

  printPositionV2((char *) "1st Ring");


  chassis.turnToPoint(24, 24, 2000, {.forwards = false, .maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(32, 23, 4000,{.forwards = false, .maxSpeed = speed1, .minSpeed = 30, .earlyExitRange = 5});
  chassis.moveToPoint(26, 23, 800,{.forwards=false, .maxSpeed = speed1 - 60});
  chassis.waitUntil(1000);
  closeClamp();
  pros::delay(200);
  printPositionV2((char *) "Goal pickup");

  chassis.turnToPoint(54, 54, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 8});
  spinIntake(127);
  chassis.moveToPoint(54, 54, 1000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 4});
  chassis.turnToHeading(45, 1000, {.maxSpeed = speed},false);
  temp_pos1 = chassis.getPose().x;
  temp_pos2 = chassis.getPose().y;
  chassis.moveToPoint(temp_pos1 +18, temp_pos2 + 14, 1000,{.maxSpeed = 80, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(48, 50, 2000,{.forwards= false,.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  chassis.turnToHeading(270, 1000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(12, 48, 3000,{.minSpeed = 30, .earlyExitRange = 8});
  liftIntake();

  chassis.moveToPoint(8, 48, 1000,{.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});  
  chassis.moveToPoint(24, 48, 3000,{.forwards=false,.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  dropIntake();
  speed = 80;
  speed1 = int(speed);
  chassis.turnToPoint(15, 17, 1000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(15, 17, 1000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 3});
  chassis.turnToHeading(225, 1000,{.maxSpeed=speed},false);
  dropIntake();
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

/*********************** POSSITIVE FULL WP ***********************/

void bluePossitiveFullWP(){ //EVERYTHING DONE
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  int temp_pos1 = 0;
  int temp_pos2 = 0;
  COLOR = false;
  
  startSorting();
  chassis.setPose(-1, 58.5, 270);
  temp_pos2=chassis.getPose().y;

  liftPneumaticUp();
  pros::delay(200);
  chassis.moveToPoint(20, temp_pos2, 900, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3}, false);
  liftPneumaticDown();
  chassis.turnToPoint(48, 24, 3000, {.maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(48, 24, 1500,{.minSpeed = 12, .earlyExitRange = 2});
  chassis.waitUntil(12);
  spinIntake(127);
  pros::Task save_ring([=] {saveOurRing(2000);});
  chassis.turnToPoint(24, 24, 2000, {.forwards = false, .maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(24, 24, 1000,{.forwards = false, .maxSpeed = speed1-40});
  chassis.waitUntil(100);
  closeClamp();
  spinIntake(127);
  pros::delay(400);
  chassis.moveToPoint(12, 32, 2000,{ .forwards=false,.maxSpeed = speed1, .minSpeed = 6, .earlyExitRange = 2});
  chassis.turnToHeading(90, 500,{.maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.waitUntil(10000);
  openClamp();
  chassis.moveToPoint(36, 24, 2000,{ .maxSpeed = speed1 - 30, .minSpeed = 6, .earlyExitRange = 2});
  chassis.turnToPoint(48, 0, 2000, {.forwards = false, .maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(42, 6.5, 2000,{.forwards = false, .maxSpeed = speed1-30, .minSpeed = 10, .earlyExitRange = 3});
  chassis.waitUntil(100);
  closeClamp();
  chassis.turnToPoint(48, 48, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 8}); 
  spinIntake(-127);
  chassis.moveToPoint(50, 48, 1000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 8});
  chassis.turnToHeading(45, 2000, {.maxSpeed = speed, .minSpeed = 8, .earlyExitRange = 2},false);
  spinIntake(127);
  temp_pos1 = chassis.getPose().x;
  temp_pos2 = chassis.getPose().y;
  chassis.moveToPoint(temp_pos1 +24, temp_pos2 + 26, 800,{.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(48, 50, 2000,{.forwards= false,.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  chassis.turnToHeading(270, 1000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(12, 48, 3000,{.minSpeed = 30, .earlyExitRange = 6});
  liftIntake();

  chassis.moveToPoint(6, 48, 1000,{.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  chassis.waitUntil(100000);
  dropIntake();
  chassis.moveToPoint(24, 48, 3000,{.forwards=false,.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  speed = 80;
  speed1 = int(speed);
  chassis.turnToPoint(12, 20, 1000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(12, 20, 1000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 3});
  chassis.turnToHeading(225, 1000,{.maxSpeed=speed},false);
  dropIntake();
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void redPossitiveFullWP(){ //90% done
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  int temp_pos1 = 0;
  int temp_pos2 = 0;
  COLOR = true;
  
  startSorting();
  chassis.setPose(0, -60, 270);
  temp_pos2=chassis.getPose().y;

  liftPneumaticUp();
  pros::delay(200);
  chassis.moveToPoint(20, temp_pos2, 900, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3});
  chassis.turnToPoint(42, -26, 3000, {.maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  liftPneumaticDown();
  chassis.moveToPoint(42, -26, 1500,{.minSpeed = 12, .earlyExitRange = 2});
  chassis.waitUntil(12);
  spinIntake(127);
  chassis.waitUntil(10);
  pros::Task save_ring([=] {saveRing(2000);});

  chassis.turnToPoint(24, -26, 2000, {.forwards = false, .maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(19, -26, 700,{.forwards = false, .maxSpeed = speed1-60});
  chassis.waitUntil(100);
  pros::delay(50);
  closeClamp();
  spinIntake(127);
  pros::delay(400);
  chassis.moveToPoint(5, -34, 2000,{ .forwards=false,.maxSpeed = speed1, .minSpeed = 6, .earlyExitRange = 2});
  chassis.turnToHeading(90, 500,{.maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.waitUntil(10000);
  openClamp();
  chassis.moveToPoint(35, -26, 2000,{ .maxSpeed = speed1 - 30, .minSpeed = 6, .earlyExitRange = 2});
  chassis.turnToPoint(38, 0, 2000, {.forwards = false, .maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(38, -3 , 2000,{.forwards = false, .maxSpeed = speed1-60, .minSpeed = 10, .earlyExitRange = 3});
  chassis.waitUntil(100);
  pros::delay(100);
  closeClamp();
  chassis.turnToPoint(47, -48, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 8}); 
  chassis.moveToPoint(47, -48, 1000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 8});
  /*redirectRings();
  chassis.turnToHeading(225, 2000, {.maxSpeed = speed, .minSpeed = 8, .earlyExitRange = 2},false);
  pros::delay(400);//*/
  chassis.turnToHeading(135, 2000, {.maxSpeed = speed, .minSpeed = 8, .earlyExitRange = 2},false);
  temp_pos1 = chassis.getPose().x;
  temp_pos2 = chassis.getPose().y;
  closeRedirect();
  chassis.moveToPoint(temp_pos1 +24, temp_pos2 -24, 1000,{.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(temp_pos1-3, temp_pos2+3, 1400,{.forwards= false,.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  chassis.turnToPoint(12, -45, 1000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(12, -45, 3000,{.minSpeed = 30, .earlyExitRange = 6});
  liftIntake();

  chassis.moveToPoint(6, -45, 1000,{.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  chassis.waitUntil(100000);
  dropIntake();
  chassis.moveToPoint(22, -45, 3000,{.forwards=false,.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  speed = 80;
  speed1 = int(speed);
  chassis.turnToPoint(12, -20, 1000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(12, -20, 1000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 3});
  chassis.turnToHeading(315, 1000,{.maxSpeed=speed},false);
  dropIntake();
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

void RedGoalRush(){
  int time = pros::millis();
  int speed = 40;
  float speed1 = float(speed);
  chassis.setPose(-61,-53,0);
  
}
void BlueGoalRush(){
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  lemlib::Pose temp_pos = chassis.getPose();
  startSorting();
  chassis.setPose(62.5,52,185);
  chassis.moveToPoint(57, 15, 10000,{.maxSpeed = speed1,.minSpeed = 25, .earlyExitRange = 2});
  extendSweep();
  chassis.waitUntilDone();
  extendRushClamp();
  chassis.moveToPoint(64, 28, 1000,{.forwards = false,.maxSpeed = speed1,.minSpeed = 25, .earlyExitRange = 2});
  chassis.turnToHeading(160, 1000,{.maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 1});
  chassis.moveToPoint(64, 38, 1000,{.forwards = false,.maxSpeed = speed1,.minSpeed = 25, .earlyExitRange = 2});
  chassis.waitUntil(10);
  retractRushClamp();
  chassis.turnToPoint(48, 48, 1000,{.forwards =false,.maxSpeed = speed,.minSpeed = 25, .earlyExitRange = 2});
  chassis.moveToPoint(48, 48, 1000,{.forwards = false,.maxSpeed = speed1,.minSpeed = 25, .earlyExitRange = 2});
  retractSweep();

  chassis.turnToPoint(24, 24, 1000,{.forwards =false,.maxSpeed = speed,.minSpeed = 25, .earlyExitRange = 2});
  chassis.moveToPoint(30, 30, 1000,{.forwards = false,.maxSpeed = speed1,.minSpeed = 32, .earlyExitRange = 4});
  chassis.moveToPoint(25, 25, 1000,{.forwards = false,.maxSpeed = speed1 - 70,.minSpeed = 25, .earlyExitRange = 2});
  closeClamp();
  chassis.turnToHeading(90, 800);
  spinIntake(127);
  chassis.moveToPoint(50, 25, 1000,{.maxSpeed = speed1,.minSpeed = 25, .earlyExitRange = 2});
  chassis.moveToPoint(20, 36, 1000,{.forwards = false,.maxSpeed = speed1,.minSpeed = 25, .earlyExitRange = 2});
  chassis.waitUntil(10000);
  pros::delay(1000);
  openClamp();
  chassis.moveToPoint(58, 34, 1000,{.maxSpeed = speed1,.minSpeed = 25, .earlyExitRange = 2});
  chassis.turnToHeading(0, 1000,{.minSpeed = 25, .earlyExitRange = 2});
  chassis.moveToPoint(56, 15, 1000,{.forwards=false});
  chassis.waitUntil(10000);
  closeClamp();
  pros::delay(100);
  chassis. moveToPoint(56, 60, 2000,{.minSpeed = 25, .earlyExitRange = 2});
  chassis.turnToHeading(45, 1000,{},false);
  temp_pos = chassis.getPose();
  // CORRNER RING + MOVE BACK TO SAME POINT
  chassis.turnToHeading(180,1000);
  chassis.moveToPoint(temp_pos.x, 25, 3000);



  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }

  
}

void goalRush(){
  if(COLOR){
    RedGoalRush();
  } else {
    BlueGoalRush();
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
void redNegativeHalfWP(){//EVERYTHING DONE
  int time = pros::millis();
  int speed = 115;
  float speed1 = float(speed);
  COLOR = true;

  chassis.setPose(0,-60,90);
  openClamp();
  liftPneumaticUp();
  pros::delay(300);
  chassis.moveToPoint(-7,-60,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 6});
  chassis.moveToPoint(-19,-26,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 6},false);
  liftPneumaticDown();
  chassis.moveToPoint(-25,-26,2000,{.forwards=false,.maxSpeed = 50,.minSpeed = 30, .earlyExitRange = 6},false);
  pros::delay(200);
  closeClamp();
  pros::delay(200);
  spinIntake(127);

  chassis.turnToPoint(-43,-24,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-43,-24,1000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  startSorting();

  //FIRST RING
  chassis.turnToPoint(-43,-10,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 1});
  chassis.moveToPoint(-43,-10.25,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 6});

  chassis.moveToPoint(-48,-24,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});

  chassis.turnToPoint(-48,-10,1000,{.maxSpeed = speed-30,.minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(-48,-10,1000,{.maxSpeed = speed1},false);
  pros::delay(300);

  chassis.moveToPoint(-48,-52,2000,{.forwards=false,.maxSpeed = speed1-30,.minSpeed = 30, .earlyExitRange = 6});
  chassis.waitUntil(18);
  chassis.turnToPoint(-72, -74, 4000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 5});
  chassis.moveToPoint(-72,-74,600,{.maxSpeed = speed1});
  chassis.moveToPoint(-72,-74,800,{.maxSpeed = speed1-40});
  chassis.moveToPoint(-54,-50,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 20, .earlyExitRange = 8},false);
  chassis.turnToPoint(0,-48,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.waitUntil(24);
  liftIntake();
  chassis.moveToPoint(-9,-51,1400,{.maxSpeed = 80});
  chassis.waitUntil(10000);
  dropIntake(); 
  pros::delay(100);
  chassis.moveToPoint(-24,-48,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.turnToPoint(-15,-23,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-15,-23,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.turnToHeading(45, 1000,{},false);
  master.clear_line(0);
  //*/
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }
  
}

void blueNegativeHalfWP(){//EVERYTHING DONE
int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = false;
  lemlib::Pose temp_pos = chassis.getPose();

  chassis.setPose(0,60,90);
  startSorting();
  openClamp();
  liftPneumaticUp();
  pros::delay(300);
  chassis.moveToPoint(-7,60,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(4);
  liftPneumaticDown();
  chassis.waitUntil(10000);
  spinIntake(-127);

  chassis.moveToPoint(-19,36,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 2},false);
  chassis.moveToPoint(-24,26,2000,{.forwards=false,.maxSpeed = 80,.minSpeed = 20, .earlyExitRange = 2},false);
  pros::delay(200);
  closeClamp();
  pros::delay(200);
  spinIntake(127);
  chassis.turnToPoint(-43,24,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-44,24,1000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 2});
  speed1 = float(speed);
  chassis.turnToPoint(-42,11,1300,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-43,11,1300,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  pros::delay(250);
  chassis.moveToPoint(-48,24,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(-48,11,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  pros::delay(250);
  chassis.moveToPoint(-48,48,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  speed = 127;
  speed1 = float(speed);
  temp_pos = chassis.getPose();

  chassis.turnToHeading(315,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(temp_pos.x - 22 ,temp_pos.y + 22,1000,{.maxSpeed = 127,.minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(-50,48,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  chassis.turnToPoint(0,48,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-15,48,2000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  liftIntake();
  chassis.moveToPoint(-7,47,2000,{.maxSpeed = speed1-40,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(10000);
  dropIntake();
  chassis.moveToPoint(-24,48,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.turnToPoint(-16,16,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-16,16,2000,{.maxSpeed = speed1,.minSpeed = 20, .earlyExitRange = 8});

  chassis.turnToHeading(145, 800,{},false);
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }
}

void negativeHalfWP(){//EVERYTHING DONE
  if(COLOR){
    redNegativeHalfWP();
  } else {
    blueNegativeHalfWP();
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

  chassis.setPose(0,-60,90);
  openClamp();
  liftPneumaticUp();
  pros::delay(300);
  chassis.moveToPoint(-7,-60,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(4);
  liftPneumaticDown();
  chassis.moveToPoint(-19,-30,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 2},false);
  chassis.moveToPoint(-22,-27,2000,{.forwards=false,.maxSpeed = 80,.minSpeed = 60, .earlyExitRange = 2},false);
  pros::delay(200);
  closeClamp();
  pros::delay(200);
  spinIntake(127);

  chassis.turnToPoint(-42,-24,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-42,-24,1000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 2});
  startSorting();

  chassis.turnToPoint(-42,-13,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-40,-13,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  pros::delay(200);

  chassis.moveToPoint(-48,-24,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(-48,-13,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2}, false);
  pros::delay(200);
  chassis.moveToPoint(-48,-52,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(18);
  stopSorting();


  chassis.turnToPoint(-72, -74, 4000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-72,-76,1000,{.maxSpeed = 127});
  chassis.moveToPoint(-54,-48,1000,{.forwards=false,.maxSpeed = speed1},false);
  /*
  liftIntake();
  chassis.turnToPoint(-72, -76, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-72,-76,1000,{.maxSpeed = 127});
  dropIntake();
  chassis.moveToPoint(-48,-48,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  
  
  chassis.turnToPoint(0, 0, 1000);
  chassis.moveToPoint(-14, -14, 3000,{.maxSpeed=60});
  chassis.turnToHeading(45, 1000);//*/

  chassis.turnToPoint(0,-48,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  liftIntake();
  chassis.moveToPoint(-8,-38,2000,{.maxSpeed = 80});
  chassis.moveToPoint(-14,-48,1000,{.forwards=false,.maxSpeed = 80},false);
  
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

  chassis.setPose(0,60,90);
  startSorting();
  openClamp();
  liftPneumaticUp();
  pros::delay(300);
  chassis.moveToPoint(-7,60,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(4);
  liftPneumaticDown();
  chassis.moveToPoint(-20,30,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 2},false);
  chassis.moveToPoint(-23.5,27,2000,{.forwards=false,.maxSpeed = 80,.minSpeed = 20, .earlyExitRange = 2},false);
  pros::delay(200);
  closeClamp();
  pros::delay(200);
  spinIntake(127);

  chassis.turnToPoint(-43,24,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-44,24,1000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 2});

  speed = 90;
  speed1 = float(speed);

  chassis.turnToPoint(-42,12,1300,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-43,12,1300,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  pros::delay(250);

  chassis.moveToPoint(-48,24,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(-48,12,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2},false);
  pros::delay(300);
  chassis.moveToPoint(-48,47,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  speed = 127;
  speed1 = float(speed);

  chassis.turnToHeading(315,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-75,72,1000,{.maxSpeed = 127});
  chassis.moveToPoint(-54,54,1000,{.forwards=false,.maxSpeed = speed1},false);
  /*
  liftIntake();
  chassis.turnToPoint(-72, -76, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-72,-76,1000,{.maxSpeed = 127});
  dropIntake();
  chassis.moveToPoint(-48,-48,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  */
  chassis.turnToPoint(0,52,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  liftIntake();
  chassis.moveToPoint(-8,52,2000,{.maxSpeed = 80});
  chassis.moveToPoint(-14,52,1000,{.forwards=false,.maxSpeed = 80},false);
  /*chassis.turnToPoint(0,-58,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  liftIntake();
  chassis.moveToPoint(-8,-56,3000,{.maxSpeed = 80},false);
  dropIntake();
  chassis.moveToPoint(-10,-56,3000,{.forwards=false,.maxSpeed = 80},false);

  //*/
  
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

/*********************** NEGTIVE SIX RING AND WALL STAKE ***********************/

void negSixRingWallStake(){
  if(COLOR){
    return;
  } else {
    return;
  }
}

/*********************** NEGTIVE SIX RING AND BOTH STAKE ***********************/

void negSixRingBothStake(){
  if(COLOR){
    return;
  } else {
    return;
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
  chassis.setPose(0,-60,90);

  startSorting();
  openClamp();
  liftPneumaticUp();
  pros::delay(200);
  chassis.moveToPoint(-6,-60,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(4);
  liftPneumaticDown();
  chassis.moveToPoint(-21,-27,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 2},false);
  closeClamp();
  pros::delay(300);
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
  chassis.moveToPoint(-7.5,temp_pos,1000,{.maxSpeed = 80});
  liftIntake();
  saveOurRing(2800);
  chassis.moveToPoint(15,temp_pos,2000,{.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  toggleClamp();
  chassis.turnToPoint(24,-27,1000,{.forwards=false,.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(24,-28,1000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6},false);
  toggleClamp();
  pros::delay(200);
  spinIntake(127);
  dropIntake();
  chassis.turnToHeading(90, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.waitUntilDone();
  speed1 = 127;
  temp_pos = chassis.getPose().y;
  chassis.moveToPoint(50,temp_pos-4,2000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 2});
  chassis.turnToPoint(14, temp_pos-4,1500,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 3});
  chassis.moveToPoint(14, temp_pos-4, 2000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 3});
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
  liftPneumaticUp();
  pros::delay(200);
  chassis.moveToPoint(-6,60,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(4);
  liftPneumaticDown();
  chassis.moveToPoint(-24,30,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 6});
  chassis.moveToPoint(-24,26,2000,{.forwards=false,.maxSpeed = speed1-60,.minSpeed = 30, .earlyExitRange = 2},false);
  closeClamp();
  pros::delay(300);
  spinIntake(127);

  chassis.turnToPoint(-43,24,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-43,24,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 6});

  chassis.turnToPoint(-41,11,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-40,3,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  pros::delay(300);
  chassis.moveToPoint(-45,45,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 5});
  /*
  chassis.moveToPoint(-50,-8,1000,{.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  chassis.moveToPoint(-50,-52,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  //*/
  chassis.turnToPoint(-72, 70, 2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-72,70,1000,{.maxSpeed = 127,.minSpeed = 30, .earlyExitRange = 5});

  chassis.moveToPoint(-44, 44, 2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  chassis.turnToHeading(90, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8},false);
  temp_pos = chassis.getPose().y;
  //chassis.moveToPoint(15,-43,2000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  chassis.moveToPoint(-9,temp_pos-2,1500,{.maxSpeed = 80});
  liftIntake();
  stopSorting();
  chassis.waitUntil(12);
  pros::Task lift_wall_stake([=] {saveOurRing(2000);});    
  chassis.waitUntil(10000);
  dropIntake();
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
  spinIntake(127);
  temp_pos = chassis.getPose().y;
  chassis.moveToPoint(46,temp_pos,2000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 2});
  chassis.turnToPoint(14, temp_pos-8,1500,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 3});
  chassis.moveToPoint(14, temp_pos-8, 2000,{.maxSpeed = speed1,.minSpeed = 12, .earlyExitRange = 3});
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

/*FOR WISCO FOR WISCO FOR WISCO FOR WISCO */

/*void blue_positive_half_wp(){ //Almost Done

  int time = pros::millis();
  int speed = 105;
  float speed1 = float(speed);
  int temp_pos1 = 0;
  int temp_pos2 = 0;
  
  chassis.setPose(-1, 58.5, 270);
  temp_pos2=chassis.getPose().y;

  liftPneumaticUp();
  pros::delay(200);
  chassis.moveToPoint(20, temp_pos2, 900, {.forwards = false});
  chassis.waitUntil(10);
  liftPneumaticDown();
  //pros::delay(200);
  liftIntake();
  chassis.moveToPoint(7, 50, 800, {.maxSpeed = 60});     
  spinIntake(127);
  saveOurRing(2000);
  printPositionV2((char *) "First ring");
  pros::delay(200);
  stopIntake();
  
  chassis.turnToPoint(24, 24, 2000, {.forwards = false, .maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  dropIntake();
  chassis.moveToPoint(26, 22, 3000, {.forwards = false, .maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(34);
  closeClamp();
  pros::delay(100);
  spinIntake(127);
  printPositionV2((char *) "Goal pickup");

  chassis.turnToPoint(43, 24, 2000, {.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(43, 24, 1500,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  autoIntake = true;
  chassis.waitUntil(22);
  printPositionV2((char *) "2nd Ring");
  stopSorting();
  chassis.turnToPoint(42, 48, 2000, {.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(42, 46, 3000, {.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  pros::delay(400);
  saveRing(2000);
  chassis.turnToHeading(315, 500,{});
  spinIntake(-127);
  chassis.waitUntilDone();
  pros::delay(100);
  chassis.turnToHeading(45, 800,{},false);
  temp_pos1 = chassis.getPose().x;
  temp_pos2 = chassis.getPose().y;
  startSorting();
  spinIntake(127);
  //chassis.moveToPoint(temp_pos1+11, temp_pos2+11, 1200);
  driveDistance(12, 2000, speed1);
  //chassis.moveToPoint(temp_pos1+6.5, temp_pos2+6.5, 800,{.forwards=false,.maxSpeed=40,.minSpeed = 20, .earlyExitRange = 8},false);
  driveDistance(-5, 1000, speed1);
  liftIntake();
  //chassis.moveToPoint(temp_pos1+11, temp_pos2+11, 600,{},false);
  driveDistance(6, 1000, speed1);
  dropIntake();
  //chassis.moveToPoint(temp_pos1-10, temp_pos2-10, 1000,{.forwards=false,.minSpeed = 20, .earlyExitRange = 8});
  driveDistance(-23, 2000, speed1);
  chassis.turnToHeading(225, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(20, 20, 2000,{},false);


  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }

}//*/

void auton_60s_skills_1() {
  COLOR = true;
  autoIntake = false;
  int time = pros::millis();
  float speed = 127;
  int temp = 0;
  chassis.setPose(5,-58.75,90);

  lemlib::Pose currentPose = chassis.getPose();
  liftPneumaticUp();
  pros::delay(300);
  chassis.moveToPoint(-5, -58.75,1000,{.forwards=false, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.moveToPoint(-24, -48, 2000,{.forwards=false, .maxSpeed=speed-30, .minSpeed = 12, .earlyExitRange = 0.5});
  liftPneumaticDown();
  chassis.waitUntil(17);
  spinIntake(-127);
  closeClamp();
  printPosition((char *)"Goal pickup", false);
  chassis.turnToHeading(0, 1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3});
  spinIntake(127);
  chassis.moveToPoint(-24, -24, 2000,{.maxSpeed=speed, .minSpeed = 50, .earlyExitRange = 8});
  chassis.waitUntil(12);
  spinIntake(127);
  chassis.turnToHeading(-90, 1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 2});
  chassis.moveToPoint(-48, -24, 2000,{.maxSpeed=speed, .minSpeed = 60, .earlyExitRange = 7});
  chassis.turnToPoint(-55, 0, 1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 2});
  chassis.moveToPoint(-55, 3, 2000,{.maxSpeed=speed, .minSpeed = 60, .earlyExitRange = 7});
  chassis.waitUntil(26);
  printPosition((char *)"Ring by stake", false);
  chassis.turnToPoint(-48, 24, 1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 2});
  hoodFwd();
  pros::delay(200);
  chassis.moveToPoint(-46, 28, 1000,{.maxSpeed=speed - 10, .minSpeed = 10, .earlyExitRange = 0.5});
  pros::delay(300);
  redirectRings();
  printPosition((char *)"Basket ring", false);
  pros::delay(700);

  chassis.moveToPoint(-48, 3.8, 1400,{.forwards = false, .maxSpeed=speed - 15, .minSpeed = 5, .earlyExitRange = 0.5});
  chassis.turnToHeading(270,1000, {.maxSpeed = (int) speed});
  closeRedirect();
  liftUpWallStake();
  chassis.waitUntil(100);
  closeRedirect();
  currentPose = chassis.getPose();
  chassis.setPose(-72 + fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  printPosition((char *)"Facing wall stake", false);
  chassis.moveToPoint(-61, currentPose.y, 1500, {.maxSpeed = speed, .minSpeed = 8, .earlyExitRange = 0.5}, false);
  printPosition((char *)"At stake", false);
  pros::Task lift_wall_stake([=] {moveLiftToPos(75);});    
    lemlib::Pose placehold = chassis.getPose();
    int x = 0;
    int angle = 7;
    while(x<2)
    {
      chassis.turnToHeading(placehold.theta + angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      chassis.turnToHeading(placehold.theta - angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      x++;
    }
    x=0;
  closeRedirect();
  printf("\n Time at 1st stake: %d", pros::millis() - time);
  chassis.moveToPoint(-52,0,1000,{.forwards=false, .maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5},false);
  currentPose = chassis.getPose();
  chassis.setPose(-72 + fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  printPosition((char *)"After reset", false);
  chassis.turnToHeading(180,1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 6});
  liftDown();
  hoodBwd();

  spinIntake(127);
  chassis.moveToPoint(-48, -56, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.moveToPoint(-42, -48, 2000,{.forwards=false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  printPosition((char *)"4th and 5th rings", false);
  chassis.turnToHeading(270, 1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(-58, -49, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  printPosition((char *)"6th ring", false);

  chassis.turnToPoint(-72, -72, 1000,{.forwards=false, .maxSpeed = (int) speed - 10, .minSpeed = 10, .earlyExitRange = 3}, false);

  currentPose = chassis.getPose();
  chassis.moveToPoint(currentPose.x-4,currentPose.y-7 ,1000,{.forwards=false,.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3},false);
  openClamp();
  pros::delay(300);


  printPosition((char *)"Goal drop", false);
  printf("\n Time at 1st goal: %d", pros::millis() - time);
  chassis.moveToPoint(-52, -52, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  
  chassis.turnToHeading(270,1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 1},false);
  currentPose = chassis.getPose();
  printPosition((char *)"After Reset", false);
  pros::delay(50); 
  //grabbing second goal
  chassis.moveToPoint(10, -48, 4000,{.forwards = false, .maxSpeed = speed, .minSpeed = 20, .earlyExitRange = 3});
  chassis.moveToPoint(21, -48, 2000,{.forwards=false, .maxSpeed = speed - 45, .minSpeed = 10, .earlyExitRange = 0.5});
  chassis.waitUntil(22);
  closeClamp();
  pros::delay(300);
  printPosition((char *)"Second goal", false);
  chassis.turnToHeading(0,1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(22, -24, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.turnToHeading(90,1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(48, -24, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.turnToPoint(56, 0,1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(56, 4, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});

  chassis.turnToPoint(48,24,2000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3});
  hoodFwd();
  chassis.moveToPoint(48, 24, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  pros::delay(300);
  redirectRings();
  printPosition((char *)"2nd Basket",false);
  pros::delay(700);

  chassis.moveToPoint(46, 1.5, 1400,{.forwards=false, .maxSpeed=speed,.minSpeed = 12, .earlyExitRange = 0.5});
  chassis.turnToHeading(90,600, {.maxSpeed = (int)speed});
  liftUpWallStake();
  closeRedirect();
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  printPosition((char *)"Facing 2nd Stake", false);
  pros::delay(50);
  chassis.moveToPoint(64, currentPose.y, 1000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5}, false);
  printPosition((char *)"At 2nd Wall Stake", false);
  pros::Task lift_wall_stake1([=] {moveLiftToPos(75);});    
    placehold = chassis.getPose();
    while(x<2)
    {
      chassis.turnToHeading(placehold.theta - 10, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      chassis.turnToHeading(placehold.theta + 10, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      x++;
    }
    x=0;
  pros::delay(300);
  printf("\n Time at 2nd wallstake: %d", pros::millis() - time);
  chassis.moveToPoint(48,currentPose.y,1000,{.forwards=false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5},false);
  currentPose = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  pros::delay(150);
  printPosition((char *)"After 2nd reset", false);
  chassis.turnToHeading(180,1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 6});
  hoodBwd();
  liftDown();

  spinIntake(127);
  chassis.moveToPoint(48, -59, 2000,{.maxSpeed = speed - 10, .minSpeed = 12, .earlyExitRange = 0.5},false);
  chassis.moveToPoint(40, -40, 2000,{.forwards = false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  printPosition((char *)"4th & 5th rings", false);
  chassis.moveToPoint(60, -48, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  printPosition((char *)"6th ring", false);
  chassis.turnToPoint(72, -72, 1000,{.forwards=false, .maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3},false);
  currentPose = chassis.getPose();
  chassis.moveToPoint(currentPose.x+4,currentPose.y-9 ,1000,{.forwards=false,.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3},false);
  openClamp();
  pros::delay(300);
  printf("\n Time at corner 2nd goal: %d", pros::millis() - time);
  chassis.moveToPoint(48, 0, 3000, {.maxSpeed = speed, .minSpeed = speed - 30, .earlyExitRange = 0.5});
  chassis.turnToPoint(27, 24, 1000,{.maxSpeed = (int)speed, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.moveToPoint(27, 24, 2000, {.maxSpeed = speed,.minSpeed=12,.earlyExitRange=0.5});
  hoodFwd();
  redirectRings();
  chassis.turnToPoint(0, 48, 1000, {.forwards = false, .maxSpeed = (int)max_speed , .earlyExitRange = 0.5});

  printPosition((char *)"3rd Basket", false);
  chassis.moveToPoint(10, 42, 4000,{.forwards = false, .maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3});

  chassis.moveToPoint(2, 47, 500, {.forwards = false, .maxSpeed = speed-60},false);
  closeClamp();
  pros::delay(300);
  pros::Task lift_wall_stake2([=] {moveLiftToPos(60);}); 
  printPosition((char *)"3rd Goal", false);
  chassis.turnToHeading(0, 1000, {.maxSpeed = (int)max_speed - 4, .earlyExitRange = 0.5},false);
  closeRedirect();
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, 72 - fabs(distToWallF() * cos(deg2rad(currentPose.theta))) , currentPose.theta);
  printPositionV2((char *)"Facing Blue Wall Stake");
  pros::delay(50);
  closeRedirect();
  chassis.moveToPoint(currentPose.x+0.7, 57.5, 1000, {.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 0.5}, false);
  printPosition((char *)"At Blue Wall Stake", false);
  pros::Task lift_wall_stake3([=] {moveLiftToPos(30);});    
    placehold = chassis.getPose();
    while(x<1)
    {
      chassis.turnToHeading(placehold.theta + angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      chassis.turnToHeading(placehold.theta - angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      x++;
    }
    x=0;
  pros::delay(200);
  printf("\n Time at blue wallstake: %d", pros::millis() - time);
  chassis.moveToPoint(currentPose.x, 48, 1000, {.forwards = false,.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5},false);
  liftDown();
  chassis.waitUntilDone();
  spinIntake(-127);
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, 72 - fabs(distToWallF() * cos(deg2rad(currentPose.theta))) , currentPose.theta);
  printPositionV2((char *)"After Blue Wall Stake", true, true);
  pros::delay(50);

  chassis.turnToPoint(-20, 28, 1000, {.maxSpeed = (int) speed,.minSpeed = 12, .earlyExitRange = 3});
  hoodBwd();
  autoIntake = true;
  chassis.moveToPoint(-20, 28, 3000, {.maxSpeed = speed,.minSpeed = 12,.earlyExitRange=0.5});
  chassis.waitUntil(6);
  spinIntake(127);
  printPosition((char *)"3rd Ring", false);
  chassis.turnToPoint(0, 0, 3000, {.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  chassis.moveToPoint(0, 0, 2000, {.maxSpeed = speed,.minSpeed = 12,.earlyExitRange=0.5});
  printPosition((char *)"Middle Ring", false);

  chassis.turnToPoint(34, 48, 2000, {.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  chassis.moveToPoint(18, 25.5, 2000, {.maxSpeed = speed-60, .minSpeed = 12, .earlyExitRange=2.0});
  chassis.moveToPoint(34, 48, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});
  startSorting();
  chassis.turnToPoint(50, 48, 1000,{.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  chassis.moveToPoint(50 , 48, 1000,{.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});
  chassis.turnToPoint(72, 72, 1000,{.forwards=false,.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3},false);
  openClamp();
  currentPose = chassis.getPose();
  chassis.moveToPoint(currentPose.x - 2.8, currentPose.y - 2.8, 700,{},false);
  closeClamp();
  printPosition((char *)"6th Ring", false);
  chassis.moveToPoint(71, 71, 1000,{.forwards=false});
  chassis.waitUntil(15);
  openClamp();
  printf("\n Time corner at 3rd goal: %d", pros::millis() - time);
  //*/
  spinIntake(-127);
  chassis.moveToPoint(48, 40, 4000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});
  chassis.moveToPoint(24, 40, 1000);
  closeClamp();
  chassis.turnToHeading(90, 1000, {.earlyExitRange=2.0});
  //chassis.turnToPoint(-72, 72,1000,{.forwards=false});
  chassis.moveToPoint(-72, 72,1900,{.forwards=false});
  pros::Task lift_for_climb([=] {moveLiftToPos(105);});
  printf("\n Time at 4th goal: %d", pros::millis() - time);
  chassis.moveToPoint(-12, 12, 3000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});
  chassis.moveToPoint(0, 0, 2500,{.maxSpeed = 50});
  climb_up();    

  //*/
  

  while (true) {
    if(getLiftPosition() < 95)
    {
      temp = pros::millis();
      break;
    }
  }
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }

} // end auton_60s_skills_1()
// Auton skills number 2 fully done
void auton_60s_skills_2() {
  pros::Task([=]{
    while(true){
      if(!autoSkill){
        return;
      }
    }
  });
  auton_60s_skills_1();
}

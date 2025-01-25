#include "auton_routines.h"
#include "auton_basics.h"
#include "auton_menu.h"
#include "controls.h"
#include "hal.h"
#include "lemlib/api.hpp"
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
auton_routine goal_rush{1.234, -1.234, 90, "15S Auton - Near Driver # 1",
                             &goal_rush_pull_back};
auton_routine blue_ring_rush{1.000, -1.300, 190, "15S Auton - Near Driver # 2",
                             &ring_rush};
auton_routine blue_neg_awp{1.000, -1.300, 190, "15S Auton - Near Driver # 3",
                           &blue_negative_four};
auton_routine neg_six_ring{1.000, -1.300, 190,
                               "15S Auton - Near Driver # 2",
                               &neg_6_ring}; // to be updated
auton_routine near_driver_elim2{1.000, -1.300, 190,
                                "15S Auton - Near Driver # 2",
                                &rushelim}; // to be updated

auton_routine rush_wp_a{1.000, -1.300, 190, "15S Auton - Near Driver # 2",
                        &rushWP}; // to be updated

auton_routine safe_positive{1.234, -1.234, 90, "15S Auton - Positive 1/2 Win Point",
                            &positive_half_wp};
auton_routine safe_negative{1.234, -1.234, 90, "15S Auton - Near Driver # 1",
                            &negative_half_wp};
auton_routine solo_WP{-0.600, 0.600, 180, "extra_1",
                                    &solo_wp};

auton_routine skills_1{-0000, 0.000, 00, "60S Auton - Skills # 1",
                       &auton_60s_skills_1};
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

// FULLY DONE
void rushBlue() {

  spinIntake(-127);
  pros::delay(250);
  spinIntake(0);

  setBasket(false);

  pros::delay(200);

  // basketRings();
  pros::delay(100);
  spinIntake(127);

  double newHeading = findHeading(2, 180);
  chassis.setPose(
      72 - fabs(findDistToWall(1) * cos(lemlib::degToRad(newHeading))),
      72 - fabs(findDistToWall(2) * cos(lemlib::degToRad(newHeading))),
      newHeading);
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
  //moveLiftToPos(350);
  newHeading = findHeading(0, 270);
  currentPose = chassis.getPose();
  chassis.setPose(
      currentPose.x,
      72 - fabs(findDistToWall(0) * cos(lemlib::degToRad(270 - newHeading))),
      newHeading);
  printPosition((char *)"Reset position", true, true);
  pros::delay(300);
  // chassis.moveToPoint(24, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  printPosition((char *)"TEST", true, true);
  // return;

  // return;
  pros::delay(500);
  chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  printPosition((char *)"TEST2", true, true);
  dropIntake();
  // chassis.waitUntilDone();
  pros::delay(250);
  chassis.moveToPoint(0, 56.5, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  printPosition((char *)"TEST3", true, true);
  //////moveLiftToPos(100);

  // chassis.setPose(0, -58, 180);
  //  setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
  chassis.moveToPoint(0, 48, 2000, {.forwards = false, .maxSpeed = max_speed});
  // chassis.movetoPoint(0, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  liftIntake();
  chassis.turnToHeading(-45, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.moveToPoint(30, 21, 2000,
                      {.forwards = false, .maxSpeed = max_speed + 15});
  chassis.waitUntilDone();
  closeClamp();
  pros::delay(200);
  spinIntake(127);
  pros::delay(250);
  chassis.turnToHeading(90, 2000);
  chassis.waitUntilDone();
  chassis.moveToPoint(56, 28, 2000);
  chassis.waitUntilDone();
  // chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});

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

  // pros::Task basketRingsTask(basketRingsAsTask);
  float failsafe_x =
      (fabs(24.0 - (71 - (distance_rb.get() / 25.4 + 6.75))) >
       fabs(24.0 -
            (71 - ((distance_rf.get() - RIGHT_DIFFERENCE) / 25.4 + 6.75))))
          ? (71 - ((distance_rf.get() - RIGHT_DIFFERENCE) / 25.4 + 6.75))
          : (71 - (distance_rb.get() / 25.4 + 6.75));
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
  //////moveLiftToPos(310);
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, findDistToWall(1) - 72, findHeading(1, 270));
  chassis.waitUntilDone();
  printPosition((char *)"Reset position", true, true);
  pros::delay(400);
  chassis.moveToPoint(2.5, -48, 2000, {.maxSpeed = max_speed});
  // chassis.moveToPoint(24, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  printPosition((char *)"TEST", true, true);
  // return;

  // return;
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
  //////moveLiftToPos(100);

  // chassis.setPose(0, -58, 180);
  //  setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
  chassis.moveToPoint(2.5, -48, 2000,
                      {.forwards = false, .maxSpeed = max_speed});
  // chassis.movetoPoint(0, -48, 2000, {.maxSpeed = max_speed});
  chassis.waitUntilDone();
  liftIntake();
  chassis.turnToHeading(225, 2000, {.maxSpeed = max_speed1});
  chassis.waitUntilDone();
  chassis.moveToPoint(24, -24, 2000,
                      {.forwards = false, .maxSpeed = max_speed + 15});
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
  // chassis.turnToHeading(0, 2000, {.maxSpeed = max_speed1});

  chassis.moveToPoint(14, -14, 2000, {.forwards = false});
}

void rushWP() {
  bool isRed = false; // TODO SWITCH

  if (isRed) {
    rushRed();
  } else {
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

  float failsafe_x =
      (fabs(24.0 - (distance_lb.get() / 25.4 + 6.75))) >
              fabs(24.0 - ((distance_lf.get() - LEFT_DIFFERENCE) / 25.4 + 6.75))
          ? (71 - ((distance_lf.get() - LEFT_DIFFERENCE) / 25.4 + 6.75))
          : (71 - (distance_lb.get() / 25.4 + 6.75));
  chassis.setPose(failsafe_x, 72 - findDistToWall(2), findHeading(2, 180));

  printPosition((char *)"Starting position", true, true);
  pros::delay(500);

  chassis.moveToPoint(48, 48, 1500, {.maxSpeed = max_speed});
  pros::delay(500);

  chassis.turnToPoint(24, 24, 1500,
                      {.forwards = false, .maxSpeed = max_speed1});
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
  // //////moveLiftToPos(400);

  chassis.turnToPoint(0, 0, 1500, {.maxSpeed = max_speed1});
  spinIntake(0);
  chassis.moveToPoint(14, 14, 2000, {.maxSpeed = max_speed});

  pros::delay(500);
}

void safePos() {
  bool isRed = false;

  if (isRed) {

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
  // moveLift(127);
  pros::delay(400);
  // moveLift(0);
  spinIntake(127);
  closeClamp();
  chassis.waitUntilDone();
  chassis.moveToPoint(48, -17, 1000,
                      {.forwards = false, .maxSpeed = max_speed});
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

void blue_negative_four() {

  chassis.setPose(-24, 51.5, 0);
  lemlib::Pose currentPose = chassis.getPose();
  pros::delay(200);
  double newHeading = findHeading(1, 0);
  chassis.setPose(fabs(findDistToWall(1) * cos(lemlib::degToRad(newHeading))) -
                      72,
                  currentPose.y, newHeading);
  printPosition((char *)"Starting position", true, true);
  pros::delay(200);
  chassis.moveToPoint(-26.5, 18, 2000,
                      {.forwards = false, .maxSpeed = max_speed + 20});
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
  chassis.setPose(
      fabs(findDistToWall(0) * cos(lemlib::degToRad(newHeading))) - 72,
      72 - fabs(findDistToWall(2) * cos(lemlib::degToRad(newHeading))),
      newHeading);
  printPosition((char *)"After reset", true, true);

  pros::delay(200);
  chassis.turnToHeading(165, 2000, {.maxSpeed = max_speed1});
  chassis.moveToPose(-42, 8, 180, 2000, {.maxSpeed = max_speed + 20});
  chassis.waitUntilDone();
  printPosition((char *)"Second Ring", true, true);
  pros::delay(200);
  chassis.moveToPoint(-48, 24, 2000,
                      {.forwards = false, .maxSpeed = max_speed});
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.turnToHeading(185, 2000, {.maxSpeed = max_speed1});
  chassis.moveToPoint(-50, 8, 2000, {.maxSpeed = max_speed + 20});
  chassis.waitUntilDone();
  printPosition((char *)"Third Ring", true, true);
  pros::delay(200);
  chassis.moveToPoint(-48, 18, 2000,
                      {.forwards = false, .maxSpeed = max_speed});
  chassis.turnToHeading(90, 1000, {.maxSpeed = max_speed1});
  chassis.moveToPoint(-14, 14, 1500, {.maxSpeed = max_speed});
}

void negativeFour() {
  bool isRed = false;

  if (isRed) {

  } else {
    blue_negative_four();
  }
}


/***********************V2 AUTONS***********************/

void red_positive_half_wp(){ //Almost Done
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
  printPositionV2((char *) "1st Ring");
  pros::delay(300);
  chassis.turnToPoint(48, -48, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 8}); 
  chassis.moveToPoint(48, -46, 1000,{.maxSpeed = speed1});
  saveRing(1500);
  chassis.turnToHeading(225, 1000,{});
  spinIntake(-127);
  pros::delay(500);
  chassis.turnToHeading(135, 2000, {.maxSpeed = speed, .minSpeed = 8, .earlyExitRange = 2},false);
  spinIntake(127);
  temp_pos1 = chassis.getPose().x;
  temp_pos2 = chassis.getPose().y;
  chassis.moveToPoint(temp_pos1 +19, temp_pos2 - 19, 2000,{.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(48, -50, 2000,{.forwards= false,.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  liftIntake();
  chassis.turnToHeading(270, 1000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(25, -50, 3000,{.minSpeed = 12, .earlyExitRange = 3});

  chassis.moveToPoint(17.5, -50, 1000,{.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(30, -50, 3000,{.forwards=false,.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  speed = 80;
  speed1 = int(speed);
  chassis.turnToPoint(23, -20, 1000);
  chassis.moveToPoint(23, -20, 1000,{.maxSpeed = speed1});
  chassis.turnToHeading(315, 1000,{.maxSpeed=speed},false);
  dropIntake();

  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }

}

void blue_positive_half_wp(){ //Almost Done
  int time = pros::millis();
  int speed = 123;
  float speed1 = float(speed);
  int temp_pos1 = 0;
  int temp_pos2 = 0;
  COLOR = false;

  stopSorting();
  
  chassis.setPose(-1, 58.5, 270);
  temp_pos2=chassis.getPose().y;

  liftPneumaticUp();
  pros::delay(200);
  chassis.moveToPoint(20, temp_pos2, 900, {.forwards = false, .minSpeed = 12, .earlyExitRange = 0.5}, false);
  liftPneumaticDown();
  
  chassis.turnToPoint(24, 24, 2000, {.forwards = false, .maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(22, 33.5, 4000,{.forwards = false, .maxSpeed = speed1, .minSpeed = 10, .earlyExitRange = 3});
  chassis.moveToPoint(22, 24, 2000,{.forwards=false, .maxSpeed = speed1 - 30, .minSpeed = 6, .earlyExitRange = 2});
  spinIntake(127);
  chassis.waitUntil(14);
  closeClamp();
  pros::delay(10);
  printPositionV2((char *) "Goal pickup");

  chassis.turnToPoint(42, 24, 2000, {.maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});

  chassis.moveToPoint(41, 24, 1500,{.minSpeed = 12, .earlyExitRange = 2}, false);
  printPositionV2((char *) "1st Ring");
  pros::delay(300);
  chassis.turnToPoint(42, 48, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 8}); 
  chassis.moveToPoint(42, 48, 1000,{.maxSpeed = speed1});
  saveRing(1500);
  chassis.turnToHeading(315, 1000,{});
  spinIntake(-127);
  pros::delay(500);
  chassis.turnToHeading(45, 2000, {.maxSpeed = speed, .minSpeed = 8, .earlyExitRange = 2},false);
  spinIntake(127);
  temp_pos1 = chassis.getPose().x;
  temp_pos2 = chassis.getPose().y;
  chassis.moveToPoint(temp_pos1 +19, temp_pos2 + 19, 2000,{.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(48, 50, 2000,{.forwards= false,.maxSpeed = 60, .minSpeed = 12, .earlyExitRange = 3});
  chassis.turnToHeading(270, 1000,{.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(8, 48, 3000,{.minSpeed = 12, .earlyExitRange = 3});
  liftIntake();

  chassis.moveToPoint(0, 48, 1000,{.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(24, 48, 3000,{.forwards=false,.maxSpeed=50,.minSpeed = 12, .earlyExitRange = 3});
  speed = 80;
  speed1 = int(speed);
  chassis.turnToPoint(12, 20, 1000);
  chassis.moveToPoint(12, 20, 1000,{.maxSpeed = speed1});
  chassis.turnToHeading(225, 1000,{.maxSpeed=speed},false);
  dropIntake();

  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

/** SafePos
 * Starts at wall stake, facing towards negative corner
 * Preload in alliance stake device
 * 
 * Alliance stake
 * 3/4 rings on one goal
 * Touch bar
 */
void positive_half_wp(){
  if(COLOR){
    red_positive_half_wp();
  } else {
    blue_positive_half_wp();
  }
}




void red_negative_half_wp(){
  int time = pros::millis();
  int speed = 105;
  float speed1 = float(speed);
  int temp_pos1 = 0;
  int temp_pos2 = 0;
  
  chassis.setPose(1, -58.5, 90);
  temp_pos2=chassis.getPose().y;

  liftPneumaticUp();
  pros::delay(200);
  chassis.moveToPoint(-20, temp_pos2, 900, {.forwards = false, .minSpeed = 12, .earlyExitRange = 0.5}, false);
  liftPneumaticDown();

  chassis.turnToPoint(-24, -24, 2000, {.forwards = false, .maxSpeed = speed,.minSpeed = 12, .earlyExitRange = 8});
  chassis.moveToPoint(-24, -33.5, 4000,{.forwards = false, .maxSpeed = speed1, .minSpeed = 10, .earlyExitRange = 3});
  chassis.moveToPoint(-24, -21, 2000,{.forwards=false, .maxSpeed = speed1 - 30, .minSpeed = 6, .earlyExitRange = 2});
  spinIntake(127);
  //chassis.moveToPoint(26, -22, 3000, {.forwards = false, .maxSpeed = speed1,.minSpeed = 8, .earlyExitRange = 2});
  chassis.waitUntil(14);
  closeClamp();
  pros::delay(70);
  printPositionV2((char *) "Goal pickup");

  chassis.turnToPoint(-48, -24, 2000, {.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3}, false);


  
}

void blue_negative_half_wp(){

}

/** NegSafe
 * 
 */
void negative_half_wp(){
  if(COLOR){
    red_negative_half_wp();
  } else {
    blue_negative_half_wp();
  }
}




void red_solo_wp(){ // DONE execpt for ring hold
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
  stopSorting();
  chassis.moveToPoint(-66,-70,1000,{.maxSpeed = 127,.minSpeed = 60, .earlyExitRange = 5});

  chassis.moveToPoint(-32, -54, 2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  chassis.turnToHeading(90, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8},false);
  temp_pos = chassis.getPose().y;

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
  chassis.moveToPoint(50,temp_pos-12,2000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
  chassis.turnToHeading(180, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(12, temp_pos, 2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 6});
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

/** solo_wp
 * Start at alliance stake, facing positive corner
 * Score AS and get ring
 * Get Neg goal, score 3 rings by line
 * Drop goal by middle
 * Get positive goal, score one ring
 * Touch ladder
 */
void solo_wp(){
  if(COLOR){
    red_solo_wp();
  } else {
    return;
  }
}

void neg_6_ring(){
  if(COLOR){
    neg_6_ring_red();
  } else {
    neg_6_ring_blue();
  }
}

void neg_6_ring_red(){ //DONE
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = true;

  chassis.setPose(0,-60,90);
  startSorting();
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

  speed = 100;
  speed1 = float(speed);

  chassis.turnToPoint(-42,-13,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-40,-13,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});

  chassis.moveToPoint(-48,-24,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(-48,-11,1000,{.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2}, false);
  pros::delay(500);
  chassis.moveToPoint(-48,-52,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  speed = 127;
  speed1 = float(speed);

  chassis.turnToPoint(-72, -74, 4000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-72,-76,1000,{.maxSpeed = 127});
  chassis.moveToPoint(-54,-54,1000,{.forwards=false,.maxSpeed = speed1},false);
  /*
  liftIntake();
  chassis.turnToPoint(-72, -76, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-72,-76,1000,{.maxSpeed = 127});
  dropIntake();
  chassis.moveToPoint(-48,-48,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  */
  
  chassis.turnToPoint(0, 0, 1000);
  chassis.moveToPoint(-14, -14, 3000,{.maxSpeed=60});
  chassis.turnToHeading(45, 1000);
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

void neg_6_ring_blue(){ //DONE
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
  pros::delay(500);
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
  
  chassis.turnToPoint(0, 0, 1000);
  chassis.moveToPoint(-13, 15, 3000,{.maxSpeed=60});
  chassis.turnToHeading(135, 1000);
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


void goal_rush_pull_back(){
  int time = pros::millis();
  int speed = 40;
  float speed1 = float(speed);
  chassis.setPose(-60,-53,0);
  chassis.moveToPoint(-56, -16, 5000,{.maxSpeed = speed1});
  pros::Task lift_wall_stake([=] {moveLiftToPos(81);});
  hoodFwd();
  extendSweep();
  chassis.waitUntil(36);
  extendRushClamp();
  chassis.turnToHeading(70, 1000,{.maxSpeed = speed},false);
  retractRushClamp();
  chassis.moveToPoint(-60, -16, 1000,{.forwards=false,.maxSpeed = speed1},false);
  retractSweep();
  chassis.turnToHeading(0, 1000);
  chassis.moveToPoint(-61, -8, 4000,{.maxSpeed = speed1},false);
  pros::Task lift_wall_stake1([=] {moveLiftToPos(60);});
  chassis.turnToHeading(0, 1300);
  chassis.moveToPoint(-60, -24, 4000,{.forwards=false,.maxSpeed = speed1},false);


  //chassis.turnToPoint(-48, -24, 2000,{.maxSpeed = speed});


  /*chassis.moveToPoint(-56, -16, 5000,{.maxSpeed = speed1});  
  pros::Task lift_wall_stake([=] {moveLiftToPos(79);});
  hoodFwd();
  extendSweep();
  chassis.waitUntil(36);
  extendRushClamp();
  chassis.moveToPoint(-63, -10, 5000,{.maxSpeed = speed1}); 
  //*/


}


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

/*void neg_6_ring_red(){ //DONE
  int time = pros::millis();
  int speed = 127;
  float speed1 = float(speed);
  COLOR = true;

  chassis.setPose(0,-60,90);
  startSorting();
  openClamp();
  liftPneumaticUp();
  pros::delay(300);
  chassis.moveToPoint(-7,-60,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 30, .earlyExitRange = 2});
  chassis.waitUntil(4);
  liftPneumaticDown();
  chassis.moveToPoint(-22,-27,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 2},false);
  pros::delay(200);
  closeClamp();
  pros::delay(200);
  spinIntake(127);

  chassis.turnToPoint(-42,-24,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-42,-24,1000,{.maxSpeed = speed1,.minSpeed = 10, .earlyExitRange = 2});

  chassis.turnToPoint(-44,-11,1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-42,-11,1000,{.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 2});

  chassis.moveToPoint(-50,-24,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 2});
  chassis.moveToPoint(-50,-8,1000,{.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 2});
  chassis.moveToPoint(-48,-52,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 2});

  chassis.turnToPoint(-72, -74, 2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-72,-76,1000,{.maxSpeed = 127});
  chassis.moveToPoint(-54,-54,1000,{.forwards=false,.maxSpeed = speed1},false);

  liftIntake();
  chassis.turnToPoint(-72, -76, 1000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-72,-76,1000,{.maxSpeed = 127});
  dropIntake();

  chassis.moveToPoint(-48,-48,2000,{.forwards=false,.maxSpeed = speed1,.minSpeed = 60, .earlyExitRange = 5});
  chassis.turnToPoint(0,-58,2000,{.maxSpeed = speed,.minSpeed = 20, .earlyExitRange = 8});
  liftIntake();
  chassis.moveToPoint(-8,-56,3000,{.maxSpeed = 80},false);
  dropIntake();
  chassis.moveToPoint(-10,-56,3000,{.forwards=false,.maxSpeed = 80},false);
  
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
  master.print(0, 0, "Time: %d", (temp-time));
  }

}//*/

void auton_15s_far_driver_elim(){ 
  printf("%s(): Exiting\n", __func__); }

void DescoreRushElim() { 
  printf("%s(): Exiting\n", __func__); }
// STILL WORKING ON IT

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
  chassis.moveToPoint(-24, -48, 2000,{.forwards=false, .maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  liftPneumaticDown();
  chassis.waitUntil(18);
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

  chassis.moveToPoint(-46, 28, 1000,{.maxSpeed=speed - 10, .minSpeed = 10, .earlyExitRange = 0.5});
  pros::delay(300);
  redirectRings();
  printPosition((char *)"Basket ring", false);

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
  pros::Task lift_wall_stake([=] {moveLiftToPos(70);});    
    lemlib::Pose placehold = chassis.getPose();
    int x = 0;
    int angle = 7;
    while(x<2)
    {
      chassis.turnToHeading(placehold.theta + angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      chassis.turnToHeading(placehold.theta - angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      x++;
    }//*/
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
  //chassis.moveToPoint(-48, -46, 2000,{.maxSpeed=speed - 17, .minSpeed = 10, .earlyExitRange = 0.5});
  chassis.moveToPoint(-48, -56, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.moveToPoint(-42, -48, 2000,{.forwards=false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  printPosition((char *)"4th and 5th rings", false);
  chassis.turnToHeading(270, 1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3});
  chassis.moveToPoint(-58, -49, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  printPosition((char *)"6th ring", false);

  chassis.turnToPoint(-72, -72, 1000,{.forwards=false, .maxSpeed = (int) speed - 10, .minSpeed = 10, .earlyExitRange = 3}, false);

  currentPose = chassis.getPose();
  /*chassis.moveToPoint(currentPose.x - 10, currentPose.y - 10, 2000, {.forwards = false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});//*/
  chassis.moveToPoint(currentPose.x-4,currentPose.y-7 ,1000,{.forwards=false,.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3},false);
  openClamp();
  pros::delay(300);


  //driveDistance(-15, 2000, speed);
  printPosition((char *)"Goal drop", false);
  printf("\n Time at 1st goal: %d", pros::millis() - time);
  chassis.moveToPoint(-52, -52, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  
  chassis.turnToHeading(270,1000, {.maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 1},false);
  currentPose = chassis.getPose();
  //chassis.setPose(-72 + fabs(distToWallF() * sin(deg2rad(currentPose.theta))), -72 + fabs(distToWallL() * sin(deg2rad(currentPose.theta))), currentPose.theta);
  printPosition((char *)"After Reset", false);
  pros::delay(50);
  chassis.moveToPoint(0, -48, 4000,{.forwards = false, .maxSpeed = speed, .minSpeed = 20, .earlyExitRange = 3});

  chassis.moveToPoint(21, -48, 2000,{.forwards=false, .maxSpeed = speed - 40, .minSpeed = 10, .earlyExitRange = 0.5});
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

  chassis.moveToPoint(46, 3.4, 1400,{.forwards=false, .maxSpeed=speed,.minSpeed = 12, .earlyExitRange = 0.5});
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
  pros::Task lift_wall_stake1([=] {moveLiftToPos(70);});    
    placehold = chassis.getPose();
    while(x<2)
    {
      chassis.turnToHeading(placehold.theta - angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      chassis.turnToHeading(placehold.theta + angle, 300, {.maxSpeed = (int) speed,.minSpeed=50,.earlyExitRange = 2});
      x++;
    }
    x=0;
  pros::delay(200);
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
  //chassis.moveToPoint(48, -46, 2000,{.maxSpeed = speed - 5, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.moveToPoint(48, -56, 2000,{.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5},false);
  chassis.moveToPoint(40, -40, 2000,{.forwards = false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  printPosition((char *)"4th & 5th rings", false);
  //chassis.turnToHeading(120, 1000, {.maxSpeed = (int) speed, .earlyExitRange = 0.5});
  chassis.moveToPoint(60, -48, 2000,{.maxSpeed=speed, .minSpeed = 12, .earlyExitRange = 0.5});
  printPosition((char *)"6th ring", false);

  chassis.turnToPoint(72, -72, 1000,{.forwards=false, .maxSpeed = (int) speed, .minSpeed = 12, .earlyExitRange = 3},false);
  currentPose = chassis.getPose();
  /*chassis.moveToPoint(currentPose.x - 10, currentPose.y - 10, 2000, {.forwards = false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});//*/
  chassis.moveToPoint(currentPose.x+4,currentPose.y-9 ,1000,{.forwards=false,.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3},false);
  openClamp();
  pros::delay(300);
  printf("\n Time at 2nd goal: %d", pros::millis() - time);
  /*currentPose = chassis.getPose();
  chassis.moveToPoint(currentPose.x + sin(deg2rad(currentPose.theta)), currentPose.y + cos(deg2rad(currentPose.theta)), 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.moveToPoint(currentPose.x - (12 * sin(deg2rad(currentPose.theta))), currentPose.y - (12 * cos(deg2rad(currentPose.theta))), 2000, {.forwards = false, .maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  // driveDistance(3, 2000, speed);
  // driveDistance(-10, 2000, speed);
  printPosition((char *)"Goal drop", false);//*/

  /*chassis.moveToPoint(48, -48, 1000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange = 0.5});
  /*return;
  chassis.waitUntil(17);
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, -72 - (distToWallL() * cos(deg2rad(currentPose.theta))) , currentPose.theta);//*/
  chassis.moveToPoint(48, 0, 3000, {.maxSpeed = speed, .minSpeed = speed - 30, .earlyExitRange = 0.5});
  chassis.turnToPoint(27, 24, 1000,{.maxSpeed = (int)speed, .minSpeed = 12, .earlyExitRange = 0.5});
  chassis.moveToPoint(27, 24, 2000, {.maxSpeed = speed,.minSpeed=12,.earlyExitRange=0.5});
  hoodFwd();
  redirectRings();
  chassis.turnToPoint(0, 48, 1000, {.forwards = false, .maxSpeed = (int)max_speed , .earlyExitRange = 0.5});

  printPosition((char *)"3rd Basket", false);
  chassis.moveToPoint(10, 42, 4000,{.forwards = false, .maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 3});

  chassis.moveToPoint(2, 48, 500, {.forwards = false, .maxSpeed = speed-20},false);
  closeClamp();
  pros::delay(300);
  pros::Task lift_wall_stake2([=] {moveLiftToPos(55);}); 
  printPosition((char *)"3rd Goal", false);
  chassis.turnToHeading(0, 1000, {.maxSpeed = (int)max_speed - 4, .earlyExitRange = 0.5},false);
  closeRedirect();
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, 72 - fabs(distToWallF() * cos(deg2rad(currentPose.theta))) , currentPose.theta);
  printPositionV2((char *)"Facing Blue Wall Stake");
  pros::delay(50);
  closeRedirect();
  chassis.moveToPoint(currentPose.x+0.2, 57.5, 1000, {.maxSpeed = speed, .minSpeed = 10, .earlyExitRange = 0.5}, false);
  printPosition((char *)"At Blue Wall Stake", false);
  pros::Task lift_wall_stake3([=] {moveLiftToPos(27);});    
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
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, 72 - fabs(distToWallF() * cos(deg2rad(currentPose.theta))) , currentPose.theta);
  printPositionV2((char *)"After Blue Wall Stake", true, true);
  pros::delay(50);

  chassis.turnToPoint(-20, 28, 1000, {.maxSpeed = (int) speed,.minSpeed = 12, .earlyExitRange = 3});
  hoodBwd();
  autoIntake = true;
  
  /*chassis.moveToPoint(-45, 48, 3000);
  chassis.waitUntil(46);
  pros::delay(100);
  chassis.moveToPoint(-55, 48, 1000, {.maxSpeed = speed});
  printPosition((char *)"1st & 2nd Rings", false);

  chassis.turnToPoint(0, 0, 1000, {.maxSpeed = (int) speed, .earlyExitRange = 0.5});//*/
  chassis.moveToPoint(-20, 28, 3000, {.maxSpeed = speed,.minSpeed = 12,.earlyExitRange=0.5});
  chassis.waitUntil(6);
  spinIntake(127);
  printPosition((char *)"3rd Ring", false);
  chassis.turnToPoint(0, 0, 3000, {.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  chassis.moveToPoint(0, 0, 2000, {.maxSpeed = speed,.minSpeed = 12,.earlyExitRange=0.5});
  printPosition((char *)"Middle Ring", false);

  chassis.turnToPoint(34, 48, 2000, {.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  chassis.moveToPoint(34, 48, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});
  startSorting();
  chassis.turnToPoint(50, 48, 1000,{.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  chassis.moveToPoint(50 , 48, 1000,{.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});
  //chassis.turnToPoint(48, 60, 2000, {.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  //schassis.moveToPoint(60, 56, 2000, {.maxSpeed = speed, .minSpeed = 100, .earlyExitRange=10});
  /*chassis.turnToPoint(48, 48, 1000,{.maxSpeed = (int)speed, .minSpeed = 100, .earlyExitRange=10});
  chassis.moveToPoint(48, 48, 1000, {.maxSpeed = speed, .minSpeed = 100, .earlyExitRange=10});
  //chassis.turnToPoint(44, 48, 1000,{.maxSpeed = (int)speed, .minSpeed = 100, .earlyExitRange=10});
  
  //chassis.waitUntil(90);
  /*currentPose = chassis.getPose();
  chassis.setPose(72 - (distToWallF() * sin(deg2rad(currentPose.theta))), 72 - (distToWallL() * sin(deg2rad(currentPose.theta))), currentPose.theta);
  pros::delay(50);
  printPositionV2((char *)"5th Ring", true, true);
  pros::delay(50);//*/
  /*chassis.turnToPoint(48, 48, 2000, {.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  chassis.moveToPoint(48, 48, 2000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});
  chassis.turnToPoint(60, 48, 2000, {.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  chassis.moveToPoint(60, 48, 1000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});/*/
  //chassis.moveToPoint(44, 48, 2000, {.forwards=false,.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5},false);

  /*chassis.turnToPoint(50, 50, 1000,{.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  chassis.moveToPoint(50, 50, 2000, { .maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});*/
  chassis.turnToPoint(72, 72, 1000,{.forwards=false,.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3},false);
  openClamp();
  currentPose = chassis.getPose();
  chassis.moveToPoint(currentPose.x - 1, currentPose.y - 1, 1000,{},false);
  closeClamp();
  printPosition((char *)"6th Ring", false);
  // extendSweep();
  // chassis.turnToHeading(270, 1000,{.maxSpeed = (int) speed, .minSpeed = 12,.earlyExitRange = 3});
  // stopIntake();
  // chassis.turnToHeading(225, 1000,{},false);
  // currentPose = chassis.getPose();
  // chassis.moveToPoint(currentPose.x-10, currentPose.y-10, 1000,{.forwards=false});
  chassis.moveToPoint(68, 68, 1000,{.forwards=false});
  openClamp();
  printf("\n Time at 3rd goal: %d", pros::millis() - time);
  spinIntake(-127);
  chassis.moveToPoint(48, 48, 4000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});
  chassis.moveToPoint(24, 48, 1000);
  chassis.turnToPoint(-72, 72,1000,{.forwards=false});
  chassis.moveToPoint(-72, 72,1900,{.forwards=false});
  printf("\n Time at 4th goal: %d", pros::millis() - time);
  pros::Task lift_for_climb([=] {moveLiftToPos(105);});
  chassis.moveToPoint(-12, 12, 3000, {.maxSpeed = speed, .minSpeed = 12, .earlyExitRange=0.5});
  chassis.moveToPoint(0, 0, 2500,{.maxSpeed = 50});
  climb_up();    
  /*chassis.turnToHeading(-158, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 0.5});
  printPosition((char *)"Corner Sweep", false);
  chassis.waitUntil(164);

  currentPose = chassis.getPose();
  chassis.moveToPoint(currentPose.x + 10, currentPose.y + 10, 2000, {.forwards = false, .maxSpeed = speed});
  //driveDistance(-15, 2000, speed);
  retractSweep();
  openClamp();
  currentPose = chassis.getPose();
  chassis.moveToPoint(currentPose.x - 3.5, currentPose.y - 3.5, 2000, {.maxSpeed = speed});
  //driveDistance(5, 2000, speed);
  pros::delay(75);
  printPosition((char *)"Goal Drop", false);

  chassis.moveToPoint(30, 30, 2000, {.maxSpeed = speed});
  autoIntake = false;
  chassis.turnToPoint(-24, 60, 1000, {.forwards = false, .maxSpeed = (int) speed, .earlyExitRange = 0.5});
  printPosition((char *)"Return to Center", false);
  chassis.moveToPoint(-24, 60, 4000, {.forwards = false, .maxSpeed = speed});
  closeClamp();
  chassis.moveToPoint(-58, 60, 2000, {.forwards = false, .maxSpeed = speed + 20});
  chassis.moveToPoint(-24, 48, 1000, {.maxSpeed = speed});
  printPosition((char *)"Goal Drop", false);
  
  //chassis.turnToHeading(270,1000);

  /*moveLiftToPos(30.00, 1000);
  chassis.moveToPoint(44,0,1000,{.forwards=false,.maxSpeed=speed});
  chassis.turnToHeading(180,1000);
  hoodFwd();
  liftDown();
  spinIntake(127);  
  chassis.moveToPoint(44, -56, 2000,{.maxSpeed=speed});
  chassis.moveToPoint(40, -48, 2000,{.forwards=false,.maxSpeed=speed});
  chassis.turnToHeading(90, 1000);
  chassis.moveToPoint(54, -48, 2000,{.maxSpeed=speed});
  chassis.turnToPoint(72, -72, 1000,{.forwards=false});
  chassis.moveToPoint(68, -68, 1000,{.forwards=false, .maxSpeed=speed});







  /*chassis.turnToPoint(-48, 0,2000);
  chassis.moveToPoint(-48, 0, 2000,{.maxSpeed=speed});




  

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

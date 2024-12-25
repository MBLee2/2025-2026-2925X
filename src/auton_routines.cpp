#include "auton_routines.h"
#include "auton_basics.h"
#include "auton_menu.h"
#include "controls.h"
#include "hal.h"
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

ASSET(matchloadturn4ball_txt);
ASSET(touchbar_txt);

// Auton routine start positions
auton_routine null_routine{0, 0, 0, "None - Invalid Routine", nullptr};
auton_routine blue_goal_rush{1.234, -1.234, 90, "15S Auton - Near Driver # 1",
                             &goal_rush};
auton_routine blue_ring_rush{1.000, -1.300, 190, "15S Auton - Near Driver # 2",
                             &ring_rush};
auton_routine blue_neg_awp{1.000, -1.300, 190, "15S Auton - Near Driver # 3",
                           &blue_negative_four};
auton_routine near_driver_elim{1.000, -1.300, 190,
                               "15S Auton - Near Driver # 2",
                               &DescoreRushElim}; // to be updated
auton_routine near_driver_elim2{1.000, -1.300, 190,
                                "15S Auton - Near Driver # 2",
                                &rushelim}; // to be updated

auton_routine rush_wp_a{1.000, -1.300, 190, "15S Auton - Near Driver # 2",
                        &rushWP}; // to be updated

auton_routine safe_positive{1.234, -1.234, 90, "15S Auton - Near Driver # 1",
                            &safePos};
auton_routine negative_four{1.234, -1.234, 90, "15S Auton - Near Driver # 1",
                            &negativeFour};
auton_routine far_from_driver_elim2{-0.600, 0.600, 180, "extra_1",
                                    &safe_6_ball};

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
  chassis.moveToPoint(-48, -7.5, 2000,
                      {
                          .forwards = false,
                      });
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

void safe_6_ball() // FAR ELIM
{}

void auton_15s_far_driver_elim() { printf("%s(): Exiting\n", __func__); }
void DescoreRushElim() { printf("%s(): Exiting\n", __func__); }
// STILL WORKING ON IT

ASSET(skillsPathPart1_txt);
void auton_60s_skills_1() {
  /*chassis.setPose(0, 0, 0);
  driveDistance(24, 2000);
  return;*/

  COLOR = true;
  COLOR_SIG = 2;
  autoIntake = true;
  int time = pros::millis();
  float speed = 80;

  spinIntake(127);
  pros::delay(200);



  /*pros::vision_object_s_t nearestRing = getMostRelevantObject();
  if(nearestRing.signature != VISION_OBJECT_ERR_SIG){
    printf("X_Coord: %d\n", nearestRing.x_middle_coord);
  } else {
    printf("No relevant Rings Detected\n");
  }*/
  //turnToRing(5000, speed);
  driveToRing(5000, speed);
  return;

  chassis.setPose(4,-58.75,90);
  lemlib::Pose currentPose = chassis.getPose();
  double temp = 0;
  /*chassis.moveToPoint(-4, -58.75,1000,{.forwards=false, .minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(-24, -48, 2000,{.forwards=false, .maxSpeed=speed, .minSpeed = 60, .earlyExitRange = 2});
  chassis.waitUntil(18);
  closeClamp();
  printPosition((char *)"Goal pickup", false);

  chassis.turnToHeading(0, 1000, {.minSpeed = 20, .earlyExitRange = 8});
  chassis.moveToPoint(-24, -24, 2000,{.maxSpeed=speed, .minSpeed = 60, .earlyExitRange = 5});
  spinIntake(127);
  chassis.turnToHeading(-90, 1000, {.minSpeed = 40, .earlyExitRange = 8});
  chassis.moveToPoint(-48, -24, 2000,{.maxSpeed=speed, .minSpeed = 60, .earlyExitRange = 5});
  chassis.turnToPoint(-56, 0, 1000, {.minSpeed = 40, .earlyExitRange = 8});
  chassis.moveToPoint(-56, 2, 2000,{.maxSpeed=speed, .minSpeed = 60, .earlyExitRange = 3});
  chassis.turnToPoint(-48, 24, 1000, {.minSpeed = 40, .earlyExitRange = 8}); 
  hoodFwd();

  chassis.moveToPoint(-45, 28, 1000,{.maxSpeed=speed, .minSpeed = 30, .earlyExitRange = 2});
  pros::delay(200);
  redirectRings();
  printPosition((char *)"Basket ring", false);
  pros::delay(700);

  chassis.moveToPoint(-48, 3.25, 2000,{.forwards=false,.maxSpeed=speed, .minSpeed = 20, .earlyExitRange = 1});
  closeRedirect();
  chassis.turnToHeading(270,2000);
  liftUpWallStake();
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(-72 + fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  printPosition((char *)"Facing wall stake", false);
  chassis.moveToPoint(-62, currentPose.y, 2000, {.maxSpeed = speed});
  chassis.waitUntilDone();
  printPosition((char *)"At stake", false);
  moveLiftToPos(50.00, 1000); //long time
  printPosition((char *)"After stake", false);
  chassis.moveToPoint(-52,0,1000,{.forwards=false, .maxSpeed=speed},false);
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(-72 + fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  printPosition((char *)"After reset", false);
  chassis.turnToHeading(180,1000, {.minSpeed = 40, .earlyExitRange = 8});
  hoodBwd();
  liftDown();

  spinIntake(127);
  chassis.moveToPoint(-48, -46, 2000,{.maxSpeed=speed - 5, .minSpeed = 40, .earlyExitRange = 3});
  chassis.moveToPoint(-48, -56, 2000,{.maxSpeed=speed, .minSpeed = 60, .earlyExitRange = 2});
  chassis.moveToPoint(-42, -48, 2000,{.forwards=false, .maxSpeed = speed, .minSpeed = 60, .earlyExitRange = 6});
  printPosition((char *)"4th and 5th rings", false);
  chassis.turnToHeading(270, 1000, {.minSpeed = 40, .earlyExitRange = 8});
  chassis.moveToPoint(-60, -48, 2000,{.maxSpeed=speed, .minSpeed = 60, .earlyExitRange = 5});
  printPosition((char *)"6th ring", false);

  chassis.turnToPoint(-72, -72, 1000,{.forwards=false, .minSpeed = 30, .earlyExitRange = 7});
  chassis.waitUntil(125);
  pros::delay(800);
  openClamp();
  pros::delay(600);
  chassis.moveToPoint(-67, -67, 1000,{.forwards=false, .maxSpeed=speed, .minSpeed = 20, .earlyExitRange = 3}, false);
  pros::delay(1000);
  printPosition((char *)"Goal drop", false);
  chassis.moveToPoint(-24, -45, 2000, {.maxSpeed = max_speed, .minSpeed = 70, .earlyExitRange = 5});
  //chassis.turnToHeading(270,1000);

  chassis.moveToPoint(0, -48, 4000,{.maxSpeed=speed});
  chassis.turnToPoint(24, -48, 1000,{.forwards=false});


  chassis.moveToPoint(24, -48, 2000,{.forwards=false,.maxSpeed=speed});
  chassis.waitUntil(18);
  closeClamp();
  pros::delay(600);
  printPosition((char *)"Second goal", false);
  chassis.turnToHeading(0,1000);
  chassis.moveToPoint(24, -24, 2000,{.maxSpeed=speed});
  chassis.turnToHeading(90,1000);
  chassis.moveToPoint(48, -24, 2000,{.maxSpeed=speed});
  chassis.turnToPoint(56, 0,1000);
  chassis.moveToPoint(56, 4, 2000,{.maxSpeed=speed});

  chassis.turnToPoint(48,24,2000);
  hoodFwd();
  chassis.moveToPoint(48, 24, 2000,{.maxSpeed=speed});
  toggleRedirect();
  pros::delay(400);
  chassis.waitUntilDone();
  printPosition((char *)"2nd Basket", false);

  chassis.moveToPoint(48, -2, 2000,{.forwards=false,.maxSpeed=speed});
  chassis.turnToHeading(90,1000);
  liftUpWallStake();
  closeRedirect();
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  printPosition((char *)"Facing 2nd Wall Stake", false);
  pros::delay(50);
  chassis.moveToPoint(65, 0, 2000,{.maxSpeed=speed}, false);
  printPosition((char *)"At 2nd Wall Stake", false);
  moveLiftToPos(50.00, 1000);
  pros::delay(600);
  chassis.moveToPoint(48,0,1000,{.forwards=false,.maxSpeed=80},false);
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallF() * sin(deg2rad(currentPose.theta))), currentPose.y, currentPose.theta);
  printPosition((char *)"After 2nd reset", false);
  chassis.turnToHeading(180,1000);
  hoodBwd();
  liftDown();

  spinIntake(127);
  chassis.moveToPoint(48, -46, 2000,{.maxSpeed=speed - 5});
  chassis.moveToPoint(48, -56, 2000,{.maxSpeed=speed});
  chassis.moveToPoint(42, -48, 2000,{.forwards=false, .maxSpeed = speed});
  printPosition((char *)"4th and 5th rings", false);
  chassis.turnToHeading(90, 1000);
  chassis.moveToPoint(60, -48, 2000,{.maxSpeed=speed});
  printPosition((char *)"6th ring", false);

  chassis.turnToPoint(72, -72, 1000,{.forwards=false});
  chassis.waitUntil(125);
  pros::delay(800);
  openClamp();
  chassis.moveToPoint(57, -45, 1000, {.maxSpeed = speed});
  chassis.moveToPoint(60, -60, 1000,{.forwards=false, .maxSpeed=speed}, false);
  pros::delay(1000);
  printPosition((char *)"Goal drop", false);
  chassis.moveToPoint(48, -48, 1000, {.maxSpeed = speed});
  chassis.moveToPoint(48, 0, 3000, {.maxSpeed = speed});*/

  /*chassis.moveToPoint(24, 24, 2000, {.maxSpeed = speed});
  hoodFwd();
  redirectRings();
  chassis.turnToPoint(0, 48, 1000, {.forwards = false, .maxSpeed = (int)max_speed - 4});
  printPosition((char *)"3rd Basket", false);
  chassis.moveToPoint(3, 48, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.waitUntil(33);
  closeClamp();
  printPosition((char *)"3rd Goal", false);

  chassis.turnToHeading(0, 1000, {.maxSpeed = (int)max_speed - 4});
  moveLiftToPos(50.0, 1000);
  closeRedirect();
  chassis.waitUntilDone();
  currentPose = chassis.getPose();
  chassis.setPose(currentPose.x, 72 - fabs(distToWallF() * cos(deg2rad(currentPose.theta))) , currentPose.theta);
  printPosition((char *)"Facing Blue Wall Stake", false);
  pros::delay(50);
  chassis.moveToPoint(currentPose.x, 58, 1000, {.maxSpeed = speed}, false);
  printPosition((char *)"At Blue Wall Stake", false);
  moveLiftToPos(27, 1000);
  pros::delay(300);
  chassis.moveToPoint(currentPose.x, 48, 1000, {.forwards = false, .maxSpeed = speed});
  liftDown();*/

  chassis.setPose(0, 48, 0);
  closeClamp();

  chassis.turnToPoint(-48, 48, 1000);
  spinIntake(127);
  hoodBwd();
  autoIntake = true;
  chassis.moveToPoint(-45, 48, 3000, {.maxSpeed = speed});
  chassis.waitUntil(46);
  pros::delay(100);
  chassis.moveToPoint(-55, 48, 1000, {.maxSpeed = speed});
  printPosition((char *)"1st & 2nd Rings", false);

  chassis.turnToPoint(0, 0, 1000);
  chassis.moveToPoint(-24, 24, 3000, {.maxSpeed = speed});
  chassis.waitUntil(34);
  printPosition((char *)"3rd Ring", false);
  pros::delay(100);
  chassis.moveToPoint(0, 0, 2000, {.maxSpeed = speed});
  printPosition((char *)"Middle Ring", false);

  chassis.turnToPoint(48, 48, 2000);
  chassis.moveToPoint(48, 48, 4000, {.maxSpeed = speed});
  chassis.turnToPoint(60, 48, 2000);
  printPosition((char *)"5th Ring", false);
  chassis.moveToPoint(58, 48, 4000, {.maxSpeed = speed});

  chassis.moveToPoint(52, 48, 2000, {.forwards = false, .maxSpeed = speed});
  printPosition((char *)"6th Ring", false);
  extendSweep();
  chassis.turnToPoint(72, 65, 2000);
  stopIntake();
  chassis.moveToPoint(56, 58, 1000, {.maxSpeed = speed});
  chassis.turnToHeading(-158, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  printPosition((char *)"Corner Sweep", false);
  chassis.waitUntil(164);

  driveDistance(-15, 2000, speed);
  retractSweep();
  openClamp();
  driveDistance(5, 2000, speed);
  pros::delay(75);
  currentPose = chassis.getPose();
  printPosition((char *)"Goal Drop", false);

  driveDistance(15, 2000, speed);
  chassis.turnToHeading(90, 1000);
  chassis.waitUntil(120);
  currentPose = chassis.getPose();
  chassis.setPose(72 - fabs(distToWallF() * cos(deg2rad(currentPose.theta - 90))), 72 - fabs(distToWallL() * cos(deg2rad(currentPose.theta - 90))), currentPose.theta);
  printPosition((char *)"After 3rd Goal Drop", false);
  currentPose = chassis.getPose();
  chassis.moveToPoint(48, currentPose.y, 1000, {.forwards = false, .maxSpeed = speed,});
  chassis.turnToHeading(90, 1000);
  chassis.waitUntil(5);

  chassis.setPose(0, 0, 0);
  pros::delay(50);
  chassis.moveToPoint(0, -48, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.waitUntil(48);
  chassis.setPose(0, 48, 90);
  
  autoIntake = false;

  //chassis.moveToPoint(0, 48, 4000, {.forwards = false, .maxSpeed = speed});
  //spinIntake(127);
  chassis.turnToPoint(-24, 60, 1000, {.forwards = false});
  printPosition((char *)"Return to Center", false);
  chassis.moveToPoint(-24, 55, 2000, {.forwards = false, .maxSpeed = speed});
  closeClamp();
  chassis.moveToPoint(-58, 57, 4000, {.forwards = false, .maxSpeed = speed + 20});
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




  

  //*/pros::delay(5);
  temp = (pros::millis()-time);
  master.print(0, 0, "Time: %f", temp);

} // end auton_60s_skills_1()
// Auton skills number 2 fully done
void auton_60s_skills_2() {}

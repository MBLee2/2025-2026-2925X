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
auton_routine null_routine  {    0,     0,   0,   "None - Invalid Routine", &blankAuton};

auton_routine top_WP{0, 0, 0, "None - Invalid Routine", &topWP};

auton_routine bottom_WP{0, 0, 0, "None - Invalid Routine", &bottomWP};

auton_routine long_goal_left{0, 0, 0, "None - Invalid Routine", &longGoalLeft};

auton_routine long_goal_right{0, 0, 0, "None - Invalid Routine", &longGoalRight};

auton_routine bottom_center{0, 0, 0, "None - Invalid Routine", &bottomCenter}; //EVERYTHING DONE

auton_routine negetive_6_ring{0, 0, 0, "None - Invalid Routine", nullptr};

auton_routine negetive_safe_wall_stake{0, 0, 0, "None - Invalid Routine", 
                                         nullptr};

auton_routine safe_six_ring{0, 0, 0, "None - Invalid Routine", 
                                         nullptr};

auton_routine skills_1{-0000, 0.000, 00, "60S Auton - Skills # 1", &auton_60s_skills_1};
                                         
auton_routine solo_WP{-0.600, 0.600, 180, "extra_1", nullptr};

void printPosition(int time) {
  lemlib::Pose temp_pose = chassis.getPose();
  printf("X: %d,\t Y:%d,\t Theta:%d,\t, Time: %d\n", temp_pose.x, temp_pose.y, temp_pose.theta, pros::millis - time);
}

void blankAuton() {
  chassis.setPose(0, 0, 0);
  return;
}

/******************************** AUTONS ********************************/
/////////////////////////////////////////////////////////////////////////

/******************************* DONE ****************************/
//////////////////////////////////////////////////////////////////


/**************************** IN PROGRESS ****************************/
//////////////////////////////////////////////////////////////////////


/*********************** LEFT SIDE FULL WIN POINT ***********************/
/**
 * 1 in long goal
 * 4 in center top goal
 * 2 in center low goal
 * 
 * Descores match loader
 */

void topWP() {
  int time = pros::millis();
  int tspeed = 127;
  float speed = (float) tspeed;

  chassis.setPose(-17, -51, -90);

  //Descore from loader
  chassis.moveToPoint(-44, -49, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  intakeAll(90);
  extendLoader();
  pros::delay(100);
  chassis.moveToPoint(-49, -60, 800, {.maxSpeed = speed, .minSpeed = 127, .earlyExitRange = 0.5}, false);
  for(int i = 0; i < 2; i++){
    chassis.moveToPoint(-49, -57, 2000, {.maxSpeed = speed, .minSpeed = 80, .earlyExitRange = 0.5}, false);
    pros::delay(100);
    chassis.moveToPoint(-49, -60, 800, {.maxSpeed = speed, .minSpeed = 127, .earlyExitRange = 0.5}, false);
    pros::delay(100);
  }
  pros::Task color_sort([=]{
    autoIntake = true;
    while(autoIntake){
      if(detectTheirColor(getIntakeColor())){
        stopStorage();
        stopReload();
        spinIntake(-127);
        autoIntake = false;
      }
      pros::delay(20);
    }
  }, "color-sort");
  chassis.moveToPoint(-50, -48, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.waitUntil(6);
  retractLoader();
  chassis.moveToPoint(-50, -50, 2000, {.maxSpeed = speed}, false);

  //Score in long goal
  chassis.turnToHeading(0, 2500, {.maxSpeed = 80}, false);
  autoIntake = false;
  chassis.moveToPoint(-46.5, -35, 2000, {.maxSpeed = speed, .earlyExitRange = 3}, false);
  chassis.moveToPoint(-46.5, -35, 2000, {.maxSpeed = 60}, false);
  topFromStorage(127);
  count_blocks(2, 1000);
  pros::delay(300);
  stopAllIntake();
  chassis.moveToPoint(-48, -48, 2000, {.forwards = false, .maxSpeed = speed});

  chassis.turnToPoint(-23, -27, 2000, {.maxSpeed = tspeed}, false);
  intakeAll(90);
  chassis.moveToPoint(-23, -27, 2000, {.maxSpeed = speed - 20}, false);

  chassis.turnToPoint(2, -5, 3000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-16, -19, 2000, {.maxSpeed = speed});
  chassis.waitUntil(15);
  middleFromStorage(100);

  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(1, 0, "Time: %d", (temp-time));
  }
  
}

/*********************** RIGHT SIDE FULL WIN POINT ***********************/
/**
 * 1 in long goal
 * 2 in center low goal
 * 4 in center top goal
 * 
 * Descores match loader
 */

void bottomWP(){
  int time = pros::millis();
  int tspeed = 80;
  float speed = (float) tspeed;

  chassis.setPose(-17, -51, -90);

  //Descore from loader
  chassis.moveToPoint(53, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  intakeAll(127);
  extendLoader();
  //count_blocks(3, 2000);
  chassis.moveToPoint(48, -65, 2000, {.maxSpeed = speed});
  chassis.moveToPoint(48, -46, 2000, {.forwards = false, .maxSpeed = speed});

  //Score in long goal
  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(48, -30, 2000, {.maxSpeed = speed});
  scoreTop(127);
  chassis.moveToPoint(48, -40, 2000, {.forwards = false, .maxSpeed = speed});

  //Score in center top goal
  chassis.turnToPoint(0, 0, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(8, -8, 3000, {.maxSpeed = speed});
  scoreMiddle(127);
  chassis.moveToPoint(24, -24, 2500, {.forwards = false, .maxSpeed = speed});

  //Score in center bottom goal
  chassis.turnToPoint(-24, -24, 2100, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(-36, -24, 3000, {.maxSpeed = speed});
  chassis.turnToPoint(0, 0, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(-8, -8, 2000, {.maxSpeed = speed});
  outakeAll(127);
  
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

/*********************** LEFT SIDE LONG GOAL ***********************/
/**
 * 7 in long goal
 * 
 * Descores match loader
 */

void longGoalLeft(){
  int time = pros::millis();
  int tspeed = 80;
  float speed = (float) tspeed;

  chassis.setPose(-9.25, -49, -25.5);

  //Collect 3 center balls
  chassis.moveToPoint(-22, -24, 2000, {.maxSpeed = speed});
  intakeAll(90);

  //Descore from loader
  chassis.turnToPoint(-46, -49, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(-46, -49, 3000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  intakeAll(100);

  chassis.moveToPoint(-48, -62, 3000, {.maxSpeed = 127, .minSpeed = 127, .earlyExitRange = 0.5}, false);
  pros::delay(200);
  for(int i = 0; i < 1; i++){
    chassis.moveToPoint(-48, -55, 2000, {.maxSpeed = speed, .minSpeed = 80, .earlyExitRange = 0.5}, false);
    chassis.moveToPoint(-48, -62.5, 1000, {.maxSpeed = speed, .minSpeed = 120, .earlyExitRange = 0.5}, false);
    pros::delay(200);
  }
  chassis.moveToPoint(-50, -48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  retractLoader();
  pros::delay(300);
  stopAllIntake();
  hoodUp();

  //Score in long goal
  chassis.turnToHeading(0, 2000, {.maxSpeed = 70}, false);
  chassis.moveToPoint(-44.75, -32, 2000, {.maxSpeed = speed, .earlyExitRange = 3}, false);
  chassis.moveToPoint(-44.75, -34, 2000, {.maxSpeed = 50}, false);
  topFromStorage(127);
  
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

/*********************** RIGHT SIDE LONG GOAL ***********************/
/**
 * 7 in long goal
 * 
 * Descores match loader
 */

void longGoalRight(){
  int time = pros::millis();
  int tspeed = 80;
  float speed = (float) tspeed;

  chassis.setPose(11.5, -50.5, 25.5);

  //Collect 3 center balls
  chassis.moveToPoint(22, -24, 2000, {.maxSpeed = speed});
  intakeAll(90);

  //Descore from loader
  chassis.turnToPoint(47, -49, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(47, -49, 3000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  intakeAll(100);

  chassis.moveToPoint(47, -62.5, 3000, {.maxSpeed = 127, .minSpeed = 127, .earlyExitRange = 0.5}, false);
  pros::delay(150);
  for(int i = 0; i < 1; i++){
    chassis.moveToPoint(47, -55, 2000, {.maxSpeed = speed, .minSpeed = 80, .earlyExitRange = 0.5}, false);
    chassis.moveToPoint(47, -62.5, 1000, {.maxSpeed = speed, .minSpeed = 120, .earlyExitRange = 0.5}, false);
    pros::delay(150);
  }
  chassis.moveToPoint(46, -48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  retractLoader();
  pros::delay(300);
  stopAllIntake();
  hoodUp();

  //Score in long goal
  chassis.turnToHeading(0, 2000, {.maxSpeed = 80}, false);
  chassis.moveToPoint(48.5, -37, 2000, {.maxSpeed = speed, .earlyExitRange = 3}, false);
  chassis.moveToPoint(48.5, -34.25, 2000, {.maxSpeed = 50}, false);
  chassis.turnToHeading(0, 1000, {.maxSpeed = tspeed}, false);
  setDriveBrake(pros::E_MOTOR_BRAKE_HOLD);
  topFromStorage(127);
  
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void bottomCenter(){
  int time = pros::millis();
  int tspeed = 80;
  float speed = (float) tspeed;

  chassis.setPose(11.5, -50.5, 25.5);

  //Collect 3 center balls
  chassis.moveToPoint(22, -24, 2000, {.maxSpeed = speed});
  intakeAll(90);

  chassis.turnToHeading(0, 1000, {.maxSpeed = tspeed});
  chassis.moveToPoint(24, -29, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(0, 0, 1000, {.maxSpeed = tspeed});
  chassis.moveToPoint(15, -16.5, 2000, {.maxSpeed = speed}, false);
  while(true){
    outakeAll(70);
    if(detectBlock(getIntakeColor())){
      pros::delay(50);
      stopStorage();
      stopReload();
    }
    pros::delay(300);
  }
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void auton_60s_skills_1() {
  COLOR = true;
  int time = pros::millis();
  float speed = 100;
  int speed1 = (int) speed;
  int temp = 0;

  chassis.setPose(-17,-56,-90);
  lemlib::Pose currentPose = chassis.getPose();

  chassis.moveToPoint(-48, -56, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 1000, {.maxSpeed = speed1}, false);
  intakeAll(127);
  pros::delay(300);

  chassis.moveToPoint(-48, -60, 1000, {.maxSpeed = speed - 20});
  count_blocks(6, 3000);
  stopAllIntake();
  return;

  chassis.moveToPoint(-48, -48, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToHeading(0, 1000, {.maxSpeed = speed1});
  chassis.moveToPoint(-48, -33, 2000, {.maxSpeed = speed}, false);
  topFromStorage(127);
  count_blocks(3, 1000);
  topFromIntake(127);
  pros::delay(1000);
  chassis.moveToPoint(-48, -48, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(-24,-24, 1000, {.maxSpeed = speed1});

  chassis.moveToPoint(-24, -24, 2000);

  
  master.clear_line(0);
  temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
} // end auton_60s_skills_1()
// Auton skills number 2 fully done
void auton_60s_skills_2() {
  COLOR = true;
  int time = pros::millis();
  float speed = 130;
  int speed1 = (int) speed;
  int temp = 0;

  chassis.setPose(0,-61.3,0);
  lemlib::Pose currentPose = chassis.getPose();
  
  master.clear_line(0);
  temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

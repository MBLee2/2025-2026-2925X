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

auton_routine win_point{0, 0, 0, "None - Invalid Routine", &winPoint};

auton_routine long_goal{0, 0, 0, "None - Invalid Routine", &longGoal};

auton_routine long_goal_left{0, 0, 0, "None - Invalid Routine", &longGoalLeft};

auton_routine long_goal_right{0, 0, 0, "None - Invalid Routine", &longGoalRight};

auton_routine top_center{0, 0, 0, "None - Invalid Routine", &topCenter}; //EVERYTHING DONE

auton_routine bottom_center{0, 0, 0, "None - Invalid Routine", &bottomCenter};

auton_routine negetive_safe_wall_stake{0, 0, 0, "None - Invalid Routine", 
                                         nullptr};

auton_routine safe_six_ring{0, 0, 0, "None - Invalid Routine", 
                                         nullptr};

auton_routine skills_1{-0000, 0.000, 00, "60S Auton - Skills # 1", &auton_60s_skills_1};
                                         
auton_routine solo_WP{-0.600, 0.600, 180, "extra_1", nullptr};

bool SIDE = true; // true = left, false = right

void printPosition(int time) {
  lemlib::Pose temp_pose = chassis.getPose();
  printf("X: %d,\t Y:%d,\t Theta:%d,\t, Time: %d\n", temp_pose.x, temp_pose.y, temp_pose.theta, pros::millis() - time);
}

void blankAuton() {
  chassis.setPose(0, 0, 0);
  //chassis.turnToHeading(180, 15000);
  //chassis.turnToHeading(0, 4000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 100});
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
  int tspeed = 115;
  float speed = (float) tspeed;
  //startSorting();
  setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(-9.5, -50, -30);
  intakeAll(127);
  // chassis.moveToPoint(-9, -40, 2000, {.maxSpeed = speed - 60});
  chassis.moveToPoint(-18, -36, 2000, {.maxSpeed = speed - 40});
  chassis.moveToPoint(-25, -24, 2000, {.maxSpeed = speed - 75});
  chassis.turnToPoint(2, 0, 2000, {.forwards = false, .maxSpeed = tspeed});
  stopAllIntake();
  chassis.moveToPoint(-13, -23.5, 2000, {.forwards = false, .maxSpeed = speed}, false);
  stopperDown();
  pros::delay(100);
  intakeAll(100);
  setScoringTrue();
  pros::delay(1600);
  master.clear_line(0);
  
  chassis.moveToPoint(-50, -55, 4000, {.maxSpeed = speed});
  stopperUp();
  goalUp();
  chassis.turnToHeading(175, 3000, {.maxSpeed = tspeed}, false);
  extendLoader();
  pros::delay(200);
  chassis.moveToPoint(chassis.getPose().x + 2, -80, 4000, {.maxSpeed = speed - 30}, false);
  pros::delay(350);
  stopAllIntake();
  chassis.moveToPoint(chassis.getPose().x - 4, -32, 1500, {.forwards = false, .maxSpeed = speed - 60}, false);
  stopperDown();
  setScoringTrue();
  pros::delay(1000);

  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
  return;

  //*/
  // master.clear_line(0);
  // int temp = pros::millis();
  // while (true) {
  //   master.print(0, 0, "Time: %d", (temp-time));
  // }
  // return;
  
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
  //DONT USE PLEASE
  int time = pros::millis();
  int tspeed = 100;
  float speed = (float) tspeed;
  //startSorting();
  setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(8.5, -48.25, 90);
  chassis.moveToPoint(48, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  intakeAll(127);
  pros::delay(200);
  chassis.moveToPoint(chassis.getPose().x, -63, 2000, {.maxSpeed = speed - 30});
  chassis.moveToPoint(48, -48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  retractLoader();

  chassis.moveToPoint(-48, -32, 1500, {.forwards = false, .maxSpeed = speed - 40}, false);
  stopperDown();
  setScoringTrue();
  pros::delay(3000);
  chassis.moveToPoint(48, -48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  chassis.turnToPoint(24, -24, 2000, {.forwards = true, .maxSpeed = tspeed}, false);
  intakeAll(127);
  chassis.moveToPoint(24, -24, 2000, {.forwards = true, .maxSpeed = speed}, false);
  pros::delay(100);
  chassis.moveToPoint(16, -16, 2000, {.forwards = true, .maxSpeed = speed}, false);
  outakeAll(60);
  pros::delay(15000);


  // chassis.setPose(8, -49, 30);
  // intakeAll(127);
  // chassis.moveToPoint(9, -40, 2000, {.maxSpeed = speed - 60});
  // chassis.moveToPoint(23, -24, 2000, {.maxSpeed = speed - 60});
  // chassis.turnToPoint(0, -3, 2000, {.forwards = true, .maxSpeed = tspeed});
  // stopAllIntake();
  // setScoringTrue();
  // chassis.moveToPoint(12, -12, 2000, {.forwards = true, .maxSpeed = speed}, false);
  // extendLoader();
  // outakeAll(50);
  // pros::delay(2300);
  // chassis.moveToPoint(45, -48, 2000, {.maxSpeed = speed});
  // goalUp();
  // chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  // chassis.moveToPoint(45.5, -63, 2000, {.maxSpeed = speed - 30}, false);
  // pros::delay(1000);
  // chassis.moveToPoint(45.5, -32, 2000, {.forwards = false, .maxSpeed = speed - 40}, false);
  // stopperDown();
  // pros::delay(15000);

  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
  return;

  }

  void winPoint() {
    if (SIDE) {
      topWP();
    } else {
      bottomWP();
    }
  }

/*********************** LEFT SIDE LONG GOAL ***********************/
/**
 * 4 in long goal
 * 
 * Descores match loader
 */

void longGoalLeft(){
  int time = pros::millis();
  int tspeed = 100;
  float speed = (float) tspeed;
  //startSorting();
  setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(-9.5, -50, -30);
  intakeAll(127);
  chassis.moveToPoint(-9, -40, 2000, {.maxSpeed = speed - 60});
  chassis.moveToPoint(-23, -24, 2000, {.maxSpeed = speed - 60});
  chassis.turnToPoint(0, -3, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-45, -48, 2000, {.maxSpeed = speed});
  stopperUp();
  goalUp();
  extendLoader();
  intakeAll(127);
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-46, -63, 2000, {.maxSpeed = speed - 30}, false);
  pros::delay(1000);
  chassis.moveToPoint(-46, -32, 2000, {.forwards = false, .maxSpeed = speed - 40}, false);
  stopperDown();
  pros::delay(15000);

  return;

  chassis.moveToPoint(-50.5, -48, 3000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  setScoringFalse();
  pros::delay(300);
  intakeAll(100);
  chassis.moveToPoint(-47, -61, 2000, {.maxSpeed = speed, .minSpeed = 40, .earlyExitRange = 0.5});
  messageStep((char *) "Start match load");
  count_blocks_in(3, 1000);
  messageStep((char *) "End match load");

  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  retractLoader();
  chassis.moveToPoint(-48, -30, 2000, {.forwards = false, .maxSpeed = speed - 20}, false);
  setScoringTrue();
  pros::delay(300);
  intakeAll(127);
  count_blocks_out(4, 2000);
  pros::delay(300);
  setScoringFalse();

  chassis.moveToPoint(-48, -47, 2000, {.maxSpeed = speed});
  descoreUp();

  chassis.turnToPoint(-37, -30, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-37, -30, 2000, {.forwards = false, .maxSpeed = speed});

  chassis.turnToPoint(-39, 0, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-38, -19, 2000, {.forwards = false, .maxSpeed = 30}, false);
  descoreDown();

  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-38.4, -12, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 0.5}, false);
  chassis.moveToPoint(-38.4, -8, 2000, {.forwards = false, .maxSpeed = 50}, false);
  end_log();


  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

/*********************** RIGHT SIDE LONG GOAL ***********************/
/**
 * 4 in long goal
 * 
 * Descores match loader
 */

void longGoalRight(){
  int time = pros::millis();
  int tspeed = 100;
  float speed = (float) tspeed;
  //startSorting();
  setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(8, -49, 30);
  intakeAll(127);
  chassis.moveToPoint(9, -40, 2000, {.maxSpeed = speed - 60});
  chassis.moveToPoint(23, -24, 2000, {.maxSpeed = speed - 60});
  chassis.turnToPoint(46, -48, 2000, {.maxSpeed = tspeed});
  stopAllIntake();
  chassis.moveToPoint(46, -48, 2000, {.maxSpeed = speed});
  goalUp();
  extendLoader();
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  intakeAll(127);
  chassis.moveToPoint(46, -62.5 , 2000, {.maxSpeed = speed - 30}, false);
  pros::delay(1000);
  chassis.moveToPoint(49, -31, 1500, {.forwards = false, .maxSpeed = speed - 40}, false);
  stopperDown();
  pros::delay(15000);


  end_log();
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }

  return;

  chassis.moveToPoint(50.5, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  setScoringFalse();
  pros::delay(300);
  intakeAll(127);
  chassis.moveToPoint(47.5, -62, 2000, {.maxSpeed = speed, .minSpeed = 40, .earlyExitRange = 0.5});
  messageStep((char *) "Start match load");
  count_blocks_in(3, 1600);
  messageStep((char *) "End match load");
  chassis.moveToPoint(48, -63, 1000, {.maxSpeed = speed, .minSpeed = 40, .earlyExitRange = 1});

  retractLoader();
  pros::delay(200);

  chassis.moveToPoint(48, -30, 2000, {.forwards = false, .maxSpeed = speed - 10}, false);
  setScoringTrue();
  pros::delay(300);
  intakeAll(127);
  count_blocks_out(4, 500);
  pros::delay(100);
  setScoringFalse();

  chassis.moveToPoint(48, -52, 2000, {.maxSpeed = speed});
  descoreUp();

  chassis.moveToPoint(37.8, -30, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(37.8, -26, 2000, {.forwards = false, .maxSpeed = 120, .minSpeed = 2, .earlyExitRange = 3}, false);
  chassis.moveToPoint(37.8, -26.5, 2000, {.forwards = false, .maxSpeed = 120}, false);
  descoreDown();
  chassis.moveToPoint(39, -8, 2000, {.forwards = false, .maxSpeed = speed, .minSpeed = 5, .earlyExitRange = 5}, false);

  end_log();


}

void longGoal(){
  if (SIDE) {
    longGoalLeft();
  } else {
    longGoalRight();
  }
}

/*********************** LEFT SIDE CENTER GOAL ***********************/
/**
 * 4 in center top goal
 * 
 */

void topCenter(){
  int time = pros::millis();
  int tspeed = 80;
  float speed = (float) tspeed;

  //startSorting();
  //setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(16.5, -52.425, 90);
  logging(3);

  chassis.moveToPoint(24, -52.3, 2000, {.maxSpeed = speed});
  chassis.turnToPoint(24, -24, 2000, {.maxSpeed = tspeed});
  stopperUp();
  pros::delay(500);
  intakeAll(127);

  chassis.moveToPoint(24, -32, 2000, {.maxSpeed = speed});
  chassis.moveToPoint(24, -25.5, 2000, {.maxSpeed = 30}, false);
  pros::delay(300);
  chassis.turnToPoint(1, 0, 1000, {.forwards = false, .maxSpeed = tspeed});

  chassis.moveToPoint(15, -18.5, 2000, {.forwards = false, .maxSpeed = speed - 20}, false);
  stopperDown();
  pros::delay(300);
  intakeAll(127);

  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

/*********************** RIGHT SIDE CENTER GOAL ***********************/
/**
 * 4 in center bottom goal
 * 
 */
void bottomCenter(){
  int time = pros::millis();
  int tspeed = 80;
  float speed = (float) tspeed;

  //startSorting();
  //setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(16.5, -52.425, 90);
  logging(3);

  chassis.moveToPoint(24, -52.3, 2000, {.maxSpeed = speed});
  chassis.turnToPoint(24, -24, 2000, {.maxSpeed = tspeed});
  stopperUp();
  pros::delay(500);
  intakeAll(127);

  chassis.moveToPoint(24, -32, 2000, {.maxSpeed = speed});
  chassis.moveToPoint(24, -25.5, 2000, {.maxSpeed = 30}, false);
  pros::delay(300);
  chassis.turnToPoint(1, 0, 1000, {.maxSpeed = tspeed});

  chassis.moveToPoint(15, -18.5, 2000, {.maxSpeed = speed - 20}, false);
  outakeAll(100);

  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

void auton_60s_skills_1() {
  COLOR = true;
  slowMiddle = true;
  int time = pros::millis();
  float speed = 100;
  int tspeed = (int) speed;
  int temp = 0;
  chassis.setPose(0.5+7.5, -48, 91);
  lemlib::Pose current_pose = chassis.getPose();
  chassis.setPose(current_pose.x, -70.5 + distToWallR() * sin(deg2rad(current_pose.theta)), current_pose.theta);
  pros::delay(30);

  chassis.moveToPoint(47, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  pros::delay(30);
  current_pose = chassis.getPose();
  chassis.setPose(70.125 + distToWallL() * cos(deg2rad(current_pose.theta)), current_pose.y, current_pose.theta);

  goalUp();
  extendLoader();
  intakeAll(127);
  pros::delay(200);
  chassis.moveToPoint(47, -60, 4000, {.maxSpeed = speed - 20});
  antiJam = false;
  count_blocks_in(6, 4000);
  chassis.cancelAllMotions();
  
  chassis.moveToPoint(chassis.getPose().x, -48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(70);
  retractLoader();

  chassis.turnToPoint(24, -24, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(24, -24, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(24, 24, 1500, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(23, 23, 2000, {.forwards = false, .maxSpeed = speed});

  chassis.turnToPoint(0, 0, 2000, {.forwards = false, .maxSpeed = tspeed}, false);
  chassis.moveToPoint(13.5, 12, 2000, {.forwards = false, .maxSpeed = speed}, false);
  goalDown();
  pros::delay(200);
  antiJam = true;
  intakeAll(127);
  stopperDown();
  count_blocks_out(3, 2000);
  stopperUp();
  stopAllIntake();

  chassis.turnToPoint(49.5, 48, 4000, {.maxSpeed = tspeed});
  goalUp();
  chassis.moveToPoint(49.5, 48, 4000, {.maxSpeed = speed}, false);
  chassis.setPose(70.125 - distToWallR() * cos(deg2rad(current_pose.theta)), 70.125 - distToWallL() * cos(deg2rad(current_pose.theta)), current_pose.theta);
  pros::delay(30);
  chassis.turnToHeading(0, 3000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(49.5, 26, 2000, {.forwards = false, .maxSpeed = speed - 20}, false);
  extendLoader();
  antiJam = true;
  intakeAll(127);
  pros::delay(200);
  stopperDown();
  
  count_blocks_out(3, 2000);
  stopperUp();
  clearQueue();

  pros::delay(200);
  
  chassis.moveToPoint(chassis.getPose().x, 58.5, 4000, {.maxSpeed = speed - 20});
  antiJam = false;
  count_blocks_in(6, 6000); 
  chassis.cancelAllMotions();

  chassis.moveToPoint(chassis.getPose().x, 26, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  count_blocks_out(6, 4000);
  clearQueue();
  retractLoader();
  return;
  
  chassis.moveToPoint(chassis.getPose().x, 45, 2000, {.maxSpeed = speed});
  pros::delay(200);
  goalDown();
  chassis.turnToHeading(-90, 2000, {.maxSpeed = tspeed});
  stopperUp();
  chassis.moveToPoint(-40, 45, 4000, {.maxSpeed = speed});

  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  intakeAll(127);
  pros::delay(200);
  chassis.moveToPoint(chassis.getPose().x, 70, 2000, {.maxSpeed = speed - 25});
  antiJam = false;
  count_blocks_in(6, 5000); 
  chassis.moveToPoint(chassis.getPose().x, 48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  retractLoader();

  chassis.turnToPoint(-24, 24, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-24, 24, 2000, {.forwards = false, .maxSpeed = speed});

  return;
  chassis.turnToPoint(-28, -22, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-28, -22, 2000, {.forwards = false, .maxSpeed = speed});
  /*chassis.turnToPoint(-21, -24, 2000, {.forwards = false, .maxSpeed = tspeed});

  chassis.moveToPoint(-21, -24, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(80);
  stopperDown();
  antiJam = true;
  count_blocks_out(3, 3000);
  stopAllIntake();
  stopperUp();*/

  chassis.turnToPoint(-55, -58, 3000, {.maxSpeed = tspeed});

  chassis.moveToPoint(-55, -58, 4000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-53, -35, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  count_blocks_out(6, 5000);
  clearQueue();

  extendLoader();
  chassis.moveToPoint(chassis.getPose().x, -71, 2000, {.maxSpeed = speed - 40});
  antiJam = false;
  count_blocks_in(3, 3000); 
  intakeAll(60);
  count_blocks_in(3, 2000);
  chassis.moveToPoint(chassis.getPose().x, -35, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  count_blocks_out(6, 4000);
  clearQueue();
  retractLoader();
  return;

  chassis.moveToPoint(-48, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(200, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-37, -64, 2000, {.maxSpeed = speed, .minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(0, -64, 2000, {.maxSpeed = 127});

  
  master.clear_line(0);
  temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
} // end auton_60s_skills_1()




// Auton skills number 2 fully done
void auton_60s_skills_2() {
   COLOR = true;
  slowMiddle = true;
  int time = pros::millis();
  float speed = 100;
  int tspeed = (int) speed;
  int temp = 0;
  chassis.setPose(0.5+7.5625, -48-0.4375, 90);

  chassis.moveToPoint(47, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  intakeAll(127);
  pros::delay(200);
  chassis.moveToPoint(chassis.getPose().x, -59, 2000, {.maxSpeed = speed - 40});
  antiJam = false;
  count_blocks_in(3, 3000); 
  intakeAll(60);
  count_blocks_in(3, 2000);
  chassis.moveToPoint(48, -48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  retractLoader();

  chassis.turnToPoint(24, -24, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(24, -24, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(24, 24, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(24, 23, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(0, 0, 2000, {.forwards = false, .maxSpeed = tspeed});

  chassis.moveToPoint(15, 15, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(80);
  stopperDown();
  antiJam = true;
  count_blocks_out(3, 3000);
  stopAllIntake();
  return;

  chassis.moveToPoint(48, 48, 2000, {.maxSpeed = speed});
  pros::delay(200);
  goalUp();
  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(48, 32, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  count_blocks_out(3, 3000);
  stopperUp();
  clearQueue();

  extendLoader();
  chassis.moveToPoint(chassis.getPose().x, 59, 2000, {.maxSpeed = speed - 40});
  antiJam = false;
  count_blocks_in(3, 3000); 
  intakeAll(60);
  count_blocks_in(3, 2000);
  chassis.moveToPoint(48, 32, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  count_blocks_out(5, 4000);
  stopperUp();
  retractLoader();

  chassis.moveToPoint(48, 48, 2000, {.maxSpeed = speed});
  pros::delay(200);
  goalDown();

  chassis.turnToPoint(0, 0, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(15, 15, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  count_blocks_out(1, 2000);
  stopperUp();
  clearQueue();

  chassis.moveToPoint(24, 24, 2000, {.maxSpeed = speed});


  chassis.turnToPoint(-48, 45, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-48, 45, 4000, {.maxSpeed = speed});

  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-48, 59, 2000, {.maxSpeed = speed - 40});
  antiJam = false;
  count_blocks_in(3, 3000); 
  intakeAll(60);
  count_blocks_in(3, 2000);
  chassis.moveToPoint(-48, 48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  retractLoader();

  chassis.turnToPoint(-24, 24, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-24, 24, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(-24, -24, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-24, -24, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(0, 0, 2000, {.forwards = false, .maxSpeed = tspeed});

  chassis.moveToPoint(-15, -15, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(80);
  stopperDown();
  antiJam = true;
  count_blocks_out(3, 3000);
  stopAllIntake();

  chassis.moveToPoint(-48, -48, 2000, {.maxSpeed = speed});
  pros::delay(200);
  goalUp();
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-48, -32, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  count_blocks_out(3, 4000);
  clearQueue();

  extendLoader();
  chassis.moveToPoint(-48, -59, 2000, {.maxSpeed = speed - 40});
  antiJam = false;
  count_blocks_in(3, 3000); 
  intakeAll(60);
  count_blocks_in(3, 2000);
  chassis.moveToPoint(-48, -32, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  count_blocks_out(6, 4000);
  clearQueue();
  retractLoader();

  chassis.moveToPoint(-48, -48, 2000, {.maxSpeed = speed});
  chassis.turnToPoint(-40, -60, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-36, -60, 2000, {.maxSpeed = speed, .minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(0, -60, 2000, {.maxSpeed = 127});

  
  master.clear_line(0);
  temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

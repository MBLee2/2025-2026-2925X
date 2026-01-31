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
  intakeAll(127);
  count_blocks_in(1, 100000);
  stopAllIntake();
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
  int tspeed = 100;
  float speed = (float) tspeed;
  //startSorting();
  setScoringFalse();
  addToQueue(COLOR);
  logging(1);

  chassis.setPose(-15.5, -70.5 + distToWallB(), 0);
  intakeAll(127);
  chassis.moveToPoint(-15.5, -36.5, 2000, {.maxSpeed = speed});
  chassis.turnToPoint(-23.5, -22, 1000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-23.5, -22, 2000, {.maxSpeed = speed - 60});

  chassis.turnToPoint(0, 0, 1000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-12.5, -12.5, 2000, {.forwards = false, .maxSpeed = speed}, false);
  stopperDown();
  //count_blocks_out(4, 1000);
  pros::delay(1300);
  
  chassis.moveToPoint(-48.5, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  goalUp();
  extendLoader();
  intakeAll(127);
  chassis.moveToPoint(-48.5, -59.5, 2000, {.maxSpeed = speed - 30});
  count_blocks_in(3, 1100);
  stopAllIntake();
  chassis.cancelAllMotions();
  chassis.moveToPoint(-48.5, -26.5, 1500, {.forwards = false, .maxSpeed = speed - 40}, false);
  intakeAll(127);
  stopperDown();
  end_log();

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

  chassis.setPose(-15.5, -70.5 + distToWallB(), 0);
  goalUp();
  intakeAll(127);
  chassis.moveToPoint(-15.5, -36.5, 2000, {.maxSpeed = speed});
  chassis.turnToPoint(-24.5, -22, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-24.5, -22, 2000, {.maxSpeed = speed - 60});
  chassis.turnToPoint(0, 0, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-47.5, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  extendLoader();
  intakeAll(127);
  chassis.moveToPoint(-47.5, -60, 2000, {.maxSpeed = speed - 30}, false);
  count_blocks_in(2, 700);
  chassis.moveToPoint(-47.5, -26, 2000, {.forwards = false, .maxSpeed = speed - 40}, false);
  stopperDown();
  
  end_log();


  //*/t
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

  chassis.setPose(15.5, -70.5 + distToWallB(), 0);
  goalUp();
  intakeAll(127);
  chassis.moveToPoint(15.5, -35, 2000, {.maxSpeed = speed});
  chassis.turnToPoint(24, -24, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(23, -24, 2000, {.maxSpeed = speed - 60});
  chassis.turnToPoint(47.5, -48, 2000, {.maxSpeed = tspeed});
  stopAllIntake();
  chassis.moveToPoint(47.5, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  extendLoader();
  intakeAll(127);
  chassis.moveToPoint(47.5, -59.5, 2000, {.maxSpeed = speed - 30}, false);
  count_blocks_in(2, 700);
  stopAllIntake();
  chassis.moveToPoint(47.5, -28.5, 1500, {.forwards = false, .maxSpeed = speed - 40}, false);
  intakeAll(127);
  stopperDown();


  end_log();
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }


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
  int tspeed = 100;
  float speed = (float) tspeed;
  //startSorting();
  setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(15.5, -70.5 + distToWallB(), 0);
  intakeAll(127);
  chassis.moveToPoint(15.5, -35, 2000, {.maxSpeed = speed});
  chassis.turnToPoint(24, -24, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(23, -24, 2000, {.maxSpeed = speed - 60});

  chassis.turnToPoint(0, 0, 1000, {.forwards = false, .maxSpeed = tspeed});

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
  float speed = 115;
  int tspeed = (int) speed;
  int temp = 0;
  chassis.setPose(0.5+7.5, -48, 91);
  lemlib::Pose current_pose = chassis.getPose();
  chassis.setPose(current_pose.x, -70.5 + distToWallR() * sin(deg2rad(current_pose.theta)), current_pose.theta);
  pros::delay(30);

  chassis.moveToPoint(46.5, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 1500, {.maxSpeed = tspeed}, false);
  pros::delay(30);
  current_pose = chassis.getPose();
  chassis.setPose(70.125 + distToWallL() * cos(deg2rad(current_pose.theta)), current_pose.y, current_pose.theta);

  goalUp();
  extendLoader();
  intakeAll(127);
  pros::delay(100);
  chassis.moveToPoint(46.5, -60.5, 4000, {.maxSpeed = speed - 20});
  antiJam = false;
  count_blocks_in(6, 4000);
  chassis.cancelAllMotions();
  
  chassis.moveToPoint(chassis.getPose().x, -48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(70);
  retractLoader();

  chassis.turnToPoint(24, -24, 800, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(24, -24, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(24, 24, 800, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(23, 23, 2000, {.forwards = false, .maxSpeed = speed});

  chassis.turnToPoint(0, 0, 1000, {.forwards = false, .maxSpeed = tspeed}, false);
  chassis.moveToPoint(13.25, 11.75, 2000, {.forwards = false, .maxSpeed = speed}, false);
  goalDown();
  pros::delay(200);
  antiJam = true;
  intakeAll(127);
  stopperDown();
  count_blocks_out(4, 1700);
  stopperUp();
  stopAllIntake();

  chassis.turnToPoint(48.5, 48, 1000, {.maxSpeed = tspeed});
  goalUp();
  chassis.moveToPoint(48.5, 48, 2000, {.maxSpeed = speed}, false);
  //chassis.setPose(70.125 + distToWallR() * cos(deg2rad(current_pose.theta)), 70.125 + distToWallL() * cos(deg2rad(current_pose.theta)), current_pose.theta);
  pros::delay(30);
  chassis.turnToHeading(0, 1000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(48.5, 26, 2000, {.forwards = false, .maxSpeed = speed - 20}, false);
  extendLoader();
  antiJam = true;
  intakeAll(127);
  pros::delay(200);
  stopperDown();
  
  count_blocks_out(3, 2000);
  stopperUp();
  clearQueue();
  
  chassis.moveToPoint(chassis.getPose().x, 58, 4000, {.maxSpeed = speed - 20});
  antiJam = false;
  count_blocks_in(6, 4000); 
  chassis.cancelAllMotions();

  chassis.moveToPoint(chassis.getPose().x, 26, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  current_pose = chassis.getPose();
  count_blocks_out(6, 4000);
  clearQueue();
  retractLoader();
  //chassis.setPose(70.5 - distToWallR() * cos(deg2rad(current_pose.theta)), current_pose.y, current_pose.theta);
  //pros::delay(20);


  
  chassis.moveToPoint(chassis.getPose().x, 45, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(-90, 800, {.maxSpeed = tspeed});
  stopAllIntake();
  chassis.moveToPoint(-44.5, 45, 4000, {.maxSpeed = speed});

  chassis.turnToHeading(0, 1000, {.maxSpeed = tspeed}, false);
  pros::delay(20);
  current_pose = chassis.getPose();
  chassis.setPose(-70.5 + distToWallL() * cos(deg2rad(current_pose.theta)), current_pose.y, current_pose.theta);
  pros::delay(20);
  extendLoader();
  intakeAll(127);
  pros::delay(100);

  chassis.moveToPoint(-46.5, 61.75, 4000, {.maxSpeed = speed - 25});
  antiJam = false;
  count_blocks_in(6, 4000); 
  chassis.cancelAllMotions();

  chassis.moveToPoint(-46.5, 48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  retractLoader();

  chassis.turnToPoint(-24, 24, 1000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-24, 24, 2000, {.forwards = false, .maxSpeed = speed});

  chassis.turnToPoint(-24, -24, 1000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-24, -21.7, 2000, {.forwards = false, .maxSpeed = speed}, false);

  current_pose = chassis.getPose();
  chassis.setPose(current_pose.x, -70.5 + distToWallB() * cos(deg2rad(current_pose.theta)), current_pose.theta);
  pros::delay(20);

  chassis.turnToPoint(-15.25, -12.75, 1000, {.forwards = false, .maxSpeed = tspeed});

  chassis.moveToPoint(-15.25, -12.75, 1500, {.forwards = false, .maxSpeed = speed}, false);
  goalDown();
  pros::delay(50);

  toggleSlowMiddle();
  intakeAll(127);
  stopperDown();
  antiJam = true;
  count_blocks_out(3, 1200);
  stopAllIntake();
  stopperUp();

  chassis.turnToPoint(-50.5, -48, 1000, {.maxSpeed = tspeed});

  chassis.moveToPoint(-50.5, -48, 4500, {.maxSpeed = speed});
  goalUp();
  chassis.turnToHeading(180, 1000, {.maxSpeed = tspeed}, false);

  current_pose = chassis.getPose();
  chassis.setPose(-70.5 - distToWallR() * cos(deg2rad(current_pose.theta)), current_pose.y, current_pose.theta);

  chassis.moveToPoint(-48.5, -24, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();

  count_blocks_out(3, 2000);
  stopperUp();
  clearQueue();

  extendLoader();
  chassis.moveToPoint(chassis.getPose().x, -58, 2000, {.maxSpeed = speed - 40});
  antiJam = false;
  count_blocks_in(6, 4000); 
  chassis.cancelAllMotions();

  chassis.moveToPoint(chassis.getPose().x, -24, 2000, {.forwards = false, .maxSpeed = speed}, false);
  intakeAll(127);
  stopperDown();
  count_blocks_out(6, 4000);
  clearQueue();
  retractLoader();

  chassis.moveToPoint(-48.5, -48, 2000, {.maxSpeed = speed});
  chassis.moveToPoint(-24, -63, 2000, {.maxSpeed = speed, .minSpeed = 30, .earlyExitRange = 2});
  chassis.moveToPoint(2, -64, 4000, {.maxSpeed = 127, .minSpeed = 80, .earlyExitRange = 1});


  master.clear_line(0);
  temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
  return;

  
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

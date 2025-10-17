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

auton_routine top_center{0, 0, 0, "None - Invalid Routine", &topCenter}; //EVERYTHING DONE

auton_routine bottom_center{0, 0, 0, "None - Invalid Routine", &bottomCenter};

auton_routine negetive_safe_wall_stake{0, 0, 0, "None - Invalid Routine", 
                                         nullptr};

auton_routine safe_six_ring{0, 0, 0, "None - Invalid Routine", 
                                         nullptr};

auton_routine skills_1{-0000, 0.000, 00, "60S Auton - Skills # 1", &auton_60s_skills_2};
                                         
auton_routine solo_WP{-0.600, 0.600, 180, "extra_1", nullptr};

void printPosition(int time) {
  lemlib::Pose temp_pose = chassis.getPose();
  printf("X: %d,\t Y:%d,\t Theta:%d,\t, Time: %d\n", temp_pose.x, temp_pose.y, temp_pose.theta, pros::millis() - time);
}

void blankAuton() {
  chassis.setPose(24, -48, 90);

  // chassis.turnToHeading(90, 2000);
  //  chassis.moveToPoint(0, 48, 2000);
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

  chassis.setPose(-16.5, -52.425, -90);
  logging(3);

  chassis.moveToPoint(-50.5, -48, 3000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  setScoringFalse();
  pros::delay(300);
  intakeAll(127);
  chassis.moveToPoint(-47, -60, 2000, {.maxSpeed = speed, .minSpeed = 40, .earlyExitRange = 0.5});
  messageStep((char *) "Start match load");
  count_blocks_in(3, 500);
  messageStep((char *) "End match load");
  

  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  retractLoader();

  chassis.moveToPoint(-48, -30, 2000, {.forwards = false, .maxSpeed = speed - 10}, false);
  setScoringTrue();
  pros::delay(300);
  intakeAll(127);
  count_blocks_out(3, 2000);
  pros::delay(300);

  setScoringFalse();
  chassis.moveToPoint(-48, -47, 2000, {.maxSpeed = speed});

  chassis.turnToPoint(-24, -24, 2000, {.forwards = true, .maxSpeed = tspeed});
  chassis.moveToPoint(-22, -22, 2000, {.forwards = true, .maxSpeed = speed - 75});

  chassis.turnToPoint(5, 0, 1000, {.forwards = false, .maxSpeed = tspeed});

  chassis.moveToPoint(-11, -12, 2000, {.forwards = false, .maxSpeed = speed - 10}, false);
  goalDown();
  setScoringTrue();
  pros::delay(100);
  intakeAll(100);
  // count_blocks_out(3, 2000);
  pros::delay(500);
  stopperUp();
  end_log();
  
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
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
  int tspeed = 115;
  float speed = (float) tspeed;
  //startSorting();
  setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(-16.5, -52.425, -90);
  logging(3);

  chassis.moveToPoint(-50.5, -48, 3000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  setScoringFalse();
  pros::delay(300);
  intakeAll(127);
  chassis.moveToPoint(-47, -60, 2000, {.maxSpeed = speed, .minSpeed = 40, .earlyExitRange = 0.5});
  messageStep((char *) "Start match load");
  count_blocks_in(3, 600);
  messageStep((char *) "End match load");

  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});

  chassis.moveToPoint(-48, -30, 2000, {.forwards = false, .maxSpeed = speed - 10}, false);
  setScoringTrue();
  pros::delay(300);
  intakeAll(127);
  count_blocks_out(3, 2000);
  pros::delay(300);

  retractLoader();
  setScoringFalse();
  chassis.moveToPoint(-48, -47, 2000, {.maxSpeed = speed});

  chassis.turnToPoint(-24, -24, 2000, {.forwards = true, .maxSpeed = tspeed});
  chassis.moveToPoint(-22, -22, 2000, {.forwards = true, .maxSpeed = speed - 75});

  chassis.turnToPoint(5, 0, 1000, {.forwards = true, .maxSpeed = tspeed});

  chassis.moveToPoint(11, -12, 2000, {.maxSpeed = speed - 20}, false);
  pros::delay(100);
  outakeAll(100);
  end_log();
  
  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
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
  int tspeed = 80;
  float speed = (float) tspeed;
  //startSorting();
  setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(-16.5, -52.425, -90);
  logging(3);

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
  int tspeed = 120;
  float speed = (float) tspeed;
  //startSorting();
  setScoringFalse();
  addToQueue(COLOR);

  chassis.setPose(16.5, -52.425, 90);
  logging(3);

  chassis.moveToPoint(50.5, -48, 3000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  setScoringFalse();
  pros::delay(300);
  intakeAll(100);
  chassis.moveToPoint(47.5, -62, 2000, {.maxSpeed = speed, .minSpeed = 40, .earlyExitRange = 0.5});
  messageStep((char *) "Start match load");
  count_blocks_in(3, 1700);
  messageStep((char *) "End match load");

  retractLoader();
  pros::delay(200);

  chassis.moveToPoint(48, -30, 2000, {.forwards = false, .maxSpeed = speed - 20}, false);
  setScoringTrue();
  pros::delay(300);
  intakeAll(127);
  count_blocks_out(4, 800);
  pros::delay(100);
  setScoringFalse();

  chassis.moveToPoint(48, -50, 2000, {.maxSpeed = speed});
  descoreUp();

  chassis.moveToPoint(37.8, -30, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(37.8, -26, 2000, {.forwards = false, .maxSpeed = 120}, false);
  chassis.moveToPoint(37.8, -26, 2000, {.forwards = false, .maxSpeed = speed, .minSpeed = 5, .earlyExitRange = 5}, false);
  descoreDown();
  chassis.moveToPoint(39, -8, 2000, {.forwards = false, .maxSpeed = speed, .minSpeed = 5, .earlyExitRange = 5}, false);

  end_log();


  //*/
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
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
  int time = pros::millis();
  float speed = 100;
  int tspeed = (int) speed;
  int temp = 0;

  //startSorting();
  setScoringFalse();
  //addRed();

  chassis.setPose(16.6, -52, 90);
  logging(3);

  chassis.moveToPoint(54, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 3000, {.maxSpeed = tspeed}, false);
  extendLoader();
  setScoringFalse();
  intakeAll(300);
  pros::delay(200);
  lemlib::Pose temp_pose = chassis.getPose();
  chassis.moveToPoint(temp_pose.x, -60, 2000, {.maxSpeed = speed - 30, .minSpeed = 30, .earlyExitRange = 0.5});
  messageStep((char *) "Start match load");
  chassis.moveToPoint(temp_pose.x, -100, 3000, {.maxSpeed = 45, .minSpeed = 30});
  count_blocks_in(6, 3000);
  messageStep((char *) "End match load");

  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});

  temp_pose = chassis.getPose();
  chassis.moveToPoint(temp_pose.x + 1, -30, 2000, {.forwards = false, .maxSpeed = speed - 20}, false);
  setScoringTrue();
  intakeAll(127);
  pros::delay(100);
  count_blocks_out(6, 2000);
  stopperUp();

  chassis.moveToPoint(45, -45, 2000, {.maxSpeed = speed});
  COLOR = false;
  setScoringFalse();

  chassis.turnToPoint(37, -33, 2000, {.maxSpeed = tspeed});
  descoreUp();
  chassis.moveToPoint(37, -33, 2000, {.maxSpeed = speed});

  chassis.turnToPoint(37, 0, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(37, -3, 2000, {.maxSpeed = 40});
  chassis.waitUntil(10);
  descoreDown();
  chassis.moveToPoint(37, 36, 2000, {.maxSpeed = speed});
  descoreUp();
  chassis.turnToPoint(48, 48, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(48, 48, 2000, {.maxSpeed = speed});

  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed}, false);
  temp_pose = chassis.getPose();
  intakeAll(100);
  chassis.moveToPoint(temp_pose.x, 60, 2000, {.maxSpeed = speed - 30, .minSpeed = 30, .earlyExitRange = 0.5});
  chassis.moveToPoint(temp_pose.x, 100, 4000, {.maxSpeed = 45, .minSpeed = 30});
  stopperDown();
  count_blocks_in(3, 2000);
  stopperUp();
  count_blocks_in(3, 2000);
  chassis.moveToPoint(48, 50, 2000, {.forwards = false, .maxSpeed = speed}, false);
  retractLoader();
  return;


  goalDown();
  setScoringTrue();
  count_blocks_out(3, 2000);
  pros::delay(100);
  stopAllIntake();
  setScoringFalse();
  goalUp();
  return;

  retractLoader();
  chassis.turnToPoint(24, 60, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(24, 60, 2000, {.maxSpeed = speed});
  chassis.moveToPoint(-24, 60, 2000, {.maxSpeed = speed});

  chassis.turnToPoint(-48, 50, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-48, 50, 2000, {.maxSpeed = speed}, false);
  extendLoader();
  COLOR = true;
  startSorting();
  intakeAll(100);
  chassis.turnToPoint(-48, 60, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-48, 60, 2000, {.maxSpeed = speed});
  count_blocks_in(6, 2000);

  chassis.moveToPoint(-48, 30, 2000, {.forwards = false, .maxSpeed = speed}, false);
  setScoringTrue();
  intakeAll(127);
  count_blocks_out(3, 2000);

  chassis.moveToPoint(-45, 40, 2000, {.maxSpeed = speed});
  COLOR = false;
  setScoringFalse();

  chassis.turnToPoint(-37, 30, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-37, 30, 2000, {.forwards = false, .maxSpeed = speed});

  chassis.turnToPoint(-42, 0, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-40, 18, 2000, {.forwards = false, .maxSpeed = 40}, false);

  chassis.turnToPoint(-22, -22, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-22, -22, 2000, {.maxSpeed = speed});

  chassis.turnToPoint(0, 0, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-15, -15, 2000, {.forwards = false, .maxSpeed = speed}, false);
  goalDown();
  setScoringTrue();
  count_blocks_out(4, 2000);
  stopAllIntake();
  stopSorting();

  chassis.moveToPoint(-48, -48, 2000, {.maxSpeed = speed});
  chassis.turnToPoint(-48, -60, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  intakeAll(100);

  chassis.moveToPoint(-48, -60, 2000, {.maxSpeed = speed});
  count_blocks_in(6, 2000);
  
  chassis.moveToPoint(-48, -50, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(-24, -60, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-24, -60, 2000, {.maxSpeed = speed});
  chassis.moveToPoint(0, -60, 2000, {.maxSpeed = speed});

  
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
  int tspeed = 100;
  float speed = (float) tspeed;
  setScoringFalse();

  chassis.setPose(16.5, -52.425, 90);
  logging(3);

  chassis.moveToPoint(50.5, -46, 3000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  setScoringFalse();
  pros::delay(300);
  intakeAll(100);
  lemlib::Pose temp_pose = chassis.getPose();
  chassis.moveToPoint(temp_pose.x, -59, 2000, {.maxSpeed = 70, .minSpeed = 70, .earlyExitRange = 0.5}, false);
  messageStep((char *) "Start match load");
  pros::Task wiggle([=]{
    for(int i = 0; i < 6; i++){
      driveStraight(-35);
      pros::delay(30);
      driveStraight(35);
      pros::delay(30);
    }
  });
  count_blocks_in(4, 2000);
  messageStep((char *) "End match load");
  intakeAll(70);

  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});

  chassis.moveToPoint(48, -28, 2000, {.forwards = false, .maxSpeed = speed - 10}, false);
  setScoringTrue();
  //retractLoader();
  pros::delay(500);
  intakeAll(127);
  count_blocks_out(5, 3000);
  pros::delay(300);
  chassis.moveToPoint(48, -47, 2000, {.maxSpeed = speed});
  descoreUp();
  setScoringFalse();

  chassis.turnToPoint(33.5, -30, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(33.5, -30, 2000, {.forwards = false, .maxSpeed = speed});

  chassis.turnToPoint(40.4, 0, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(39, -16, 2000, {.forwards = false, .maxSpeed = 30}, false);
  descoreDown();
  pros::delay(400);

  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(38.7, -11.5, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 0.5}, false);
  //chassis.moveToPoint(38.7, -10, 2000, {.forwards = false, .maxSpeed = 50}, false);
  descoreUp();
  chassis.moveToPoint(35, 24, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(48, 48, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = tspeed});

  chassis.moveToPoint(50, 48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed});

  chassis.moveToPoint(49, 61, 2000, {.maxSpeed = 35, .minSpeed = 35, .earlyExitRange = 0.5});
  messageStep((char *) "Start match load");
  driveStraight(35);
  count_blocks_in(5, 3000);
  messageStep((char *) "End match load");
  intakeAll(70);
  
  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed});
  
  chassis.moveToPoint(48.5, 28, 2000, {.forwards = false, .maxSpeed = speed - 10}, false);
  setScoringTrue();
  pros::delay(200);
  intakeAll(127);
  count_blocks_out(5, 3000);
  stopperUp();
  pros::delay(300);
  chassis.moveToPoint(50, 40, 2000, {.maxSpeed = speed});
  return;

  chassis.turnToPoint(-50, 40, 2000, {.maxSpeed = tspeed});

  chassis.moveToPoint(-50.5, 40, 3000, {.maxSpeed = speed});
  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed}, false);
  extendLoader();
  setScoringFalse();
  pros::delay(300);
  intakeAll(100);
  temp_pose = chassis.getPose();
  chassis.moveToPoint(temp_pose.x, 61, 2000, {.maxSpeed = 35, .minSpeed = 35, .earlyExitRange = 0.5}, false);
  messageStep((char *) "Start match load");
  driveStraight(35);
  count_blocks_in(4, 2000);
  messageStep((char *) "End match load");
  intakeAll(70);

  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed});

  chassis.moveToPoint(-48, 28, 2000, {.forwards = false, .maxSpeed = speed - 10}, false);
  setScoringTrue();
  //retractLoader();
  pros::delay(500);
  intakeAll(127);
  count_blocks_out(5, 3000);
  pros::delay(300);
  chassis.moveToPoint(-48, 47, 2000, {.maxSpeed = speed});
  descoreUp();
  setScoringFalse();

  chassis.turnToPoint(-33.5, 30, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-33.5, 30, 2000, {.forwards = false, .maxSpeed = speed});

  chassis.turnToPoint(-40.4, 0, 2000, {.forwards = false, .maxSpeed = tspeed});
  chassis.moveToPoint(-39, 16, 2000, {.forwards = false, .maxSpeed = 30}, false);
  descoreDown();
  pros::delay(400);

  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  chassis.moveToPoint(-38.7, 11.5, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 50, .earlyExitRange = 0.5}, false);
  //chassis.moveToPoint(38.7, -10, 2000, {.forwards = false, .maxSpeed = 50}, false);
  descoreUp();
  chassis.moveToPoint(-35, -24, 2000, {.forwards = false, .maxSpeed = speed});
  chassis.turnToPoint(-48, -48, 2000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = tspeed});

  chassis.moveToPoint(-50, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});

  chassis.moveToPoint(-49, -61, 2000, {.maxSpeed = 35, .minSpeed = 35, .earlyExitRange = 0.5});
  messageStep((char *) "Start match load");
  driveStraight(35);
  count_blocks_in(5, 3000);
  messageStep((char *) "End match load");
  intakeAll(70);
  
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed});
  
  chassis.moveToPoint(-48.5, -28, 2000, {.forwards = false, .maxSpeed = speed - 10}, false);
  setScoringTrue();
  pros::delay(200);
  intakeAll(127);
  count_blocks_out(5, 3000);
  stopperUp();
  pros::delay(300);
  chassis.moveToPoint(-50, -40, 2000, {.maxSpeed = speed});
  
  master.clear_line(0);
  int temp = pros::millis();
  while (true) {
    master.print(0, 0, "Time: %d", (temp-time));
  }
}

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
auton_routine null_routine  {    0,     0,   0,   "None - Invalid Routine", nullptr};

auton_routine top_WP{0, 0, 0, "None - Invalid Routine", &topWP};

auton_routine bottom_WP{0, 0, 0, "None - Invalid Routine", &bottomWP};

auton_routine long_goal_left{0, 0, 0, "None - Invalid Routine", &longGoalLeft};

auton_routine goal_rush_and_stake{0, 0, 0, "None - Invalid Routine", nullptr};

auton_routine safe_negative{0, 0, 0, "None - Invalid Routine", nullptr}; //EVERYTHING DONE

auton_routine negetive_6_ring{0, 0, 0, "None - Invalid Routine", nullptr};

auton_routine negetive_safe_wall_stake{0, 0, 0, "None - Invalid Routine", 
                                         nullptr};

auton_routine safe_six_ring{0, 0, 0, "None - Invalid Routine", 
                                         nullptr};

auton_routine skills_1{-0000, 0.000, 00, "60S Auton - Skills # 1", &auton_60s_skills_1};
                                         
auton_routine solo_WP{-0.600, 0.600, 180, "extra_1", nullptr};

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
             getFrontDistance(), getLeftDistance());
    }
    printf("Front: %3.2f     \tLeft: %3.2f\n",
           distToWallF(), distToWallL());
    printf("Front position: %3.2f     \tLeft position: %3.2f\n",
            72 - distToWallF() * cos(deg2rad(fmodf(currentPose.theta + 360, 90))), 72 - distToWallL() * cos(deg2rad(fmodf(currentPose.theta + 360, 90))));
  }
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
  int tspeed = 80;
  float speed = (float) tspeed;

  chassis.setPose(-20, -48, -90);

  //Descore from loader
  chassis.moveToPoint(-53, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  intakeAll(127);
  chassis.moveToPoint(-48, -60, 2000, {.maxSpeed = speed});
  chassis.moveToPoint(-48, -48, 2000, {.forwards = false, .maxSpeed = speed});

  //Score in long goal
  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(-48, -24, 2000, {.maxSpeed = speed});
  scoreTop(127);
  chassis.moveToPoint(-48, -48, 2000, {.forwards = false, .maxSpeed = speed});

  //Score in center top goal
  chassis.turnToPoint(0, 0, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(-15, -15, 2000, {.maxSpeed = speed});
  scoreMiddle(127);
  chassis.moveToPoint(-24, -24, 2000, {.forwards = false, .maxSpeed = speed});

  //Score in center bottom goal
  chassis.turnToPoint(24, -24, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(26.5, -24, 2000, {.maxSpeed = speed});
  chassis.turnToPoint(0, 0, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(8, -8, 2000, {.maxSpeed = speed});
  outakeAll(127);
  
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
  int tspeed = 80;
  float speed = (float) tspeed;

  chassis.setPose(20, -48, 90);

  //Descore from loader
  chassis.moveToPoint(53, -48, 2000, {.maxSpeed = speed});
  chassis.turnToHeading(180, 2000, {.maxSpeed = tspeed}, false);
  intakeAll(127);
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

  chassis.setPose(-16, -48, 0);

  //Collect 3 center balls
  chassis.moveToPoint(-24, -24, 2000, {.maxSpeed = speed});
  intakeAll(127);
  pros::Task::delay(3000);

  //Descore from loader
  chassis.turnToHeading(-150, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(-50, -56, 3000, {.maxSpeed = speed});
  chassis.turnToHeading(-180, 2000, {.maxSpeed = tspeed}, false);
  printf("Current Position: x: %3.2f, y: %3.2f\n", chassis.getPose().x, chassis.getPose().y);
  chassis.moveToPoint(-50, -65, 3000, {.maxSpeed = 127}, false);
  printf("Current Position: x: %3.2f, y: %3.2f\n", chassis.getPose().x, chassis.getPose().y);
  intakeAll(127);
  pros::Task::delay(3000);
  chassis.moveToPoint(-50, -48, 2000, {.forwards = false, .maxSpeed = speed}, false);
  pros::Task::delay(300);

  //Score in long goal
  chassis.turnToHeading(0, 2000, {.maxSpeed = tspeed}, false);
  chassis.moveToPoint(-48, -20, 2000, {.maxSpeed = speed});
  scoreTop(127);
  
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

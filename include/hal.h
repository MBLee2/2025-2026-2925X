#ifndef _HAL_H_
#define _HAL_H_

#include "api.h"
#include "auton_basics.h"
#include "pros/motors.h"

#define LAT_KP  7
#define LAT_KI  0
#define LAT_KD  0 //2

#define LAT_SMALL_RANGE 1
#define LAT_SMALL_RANGE_TIMEOUT 150

#define TURN_KP 0
#define TURN_KI 0
#define TURN_KD 0

#define VISION_KP 0.5
#define VISION_RANGE 15
#define VISION_RANGE_TIMEOUT 200

#define F_DISTANCE_OFFSET 6.25
#define B_DISTANCE_OFFSET 6.25
#define L_DISTANCE_OFFSET 7.5
#define PROXI_OFFSET 1.5

extern bool COLOR;
extern int COLOR_SIG;

extern bool auton, autoSkill;
extern bool autoDrive, autoLift, autoIntake;

void stopAllMotors();

void spinLFMotor(int speed);
void stopLFMotor();
void spinLMMotor(int speed);
void stopLMMotor();
void spinLBMotor(int speed);
void stopLBMotor();
void spinRFMotor(int speed);
void stopRFMotor();
void spinRMMotor(int speed);
void stopRMMotor();
void spinRBMotor(int speed);
void stopRBMotor();

void spinLeftMotors(int speed);
void stopLeftMotors();
void spinRightMotors(int speed);
void stopRightMotors();

void drive(int leftSpeed, int rightSpeed);
void driveStraight(int speed);
void stopDrive();
void stopDriveHold();
void setDriveBrake(pros::motor_brake_mode_e mode);

void spinIntake(int speed);
void stopIntake();
void stopIntakeHold();
void setIntakeBrake(pros::motor_brake_mode_e mode);
void intakeAntiJam();
void intakeAntiJamTaskFunc();

void spinLift(int speed);
void stopLift();
void stopLiftPID();
void stopLiftHold();
void setLiftBrake(pros::motor_brake_mode_e mode);

void openClamp();
void closeClamp();
void toggleClamp();

void extendLeftSweeper();
void retractLeftSweeper();
void toggleLeftSweeper();

void extendRightSweeper();
void retractRightSweeper();
void toggleRightSweeper();

void toggleHood();
void hoodFwd();
void hoodBwd();
bool getHood();

void liftIntake();
void dropIntake();

void extendSweep();
void retractSweep();
void extendRushClamp();
void retractRushClamp();

void extendSixRing();
void retractSixRing();
void toggleSixRing();

void redirectRings();
void closeRedirect();
void toggleRedirect();

void liftPneumaticUp();
void liftPneumaticDown();
bool getLiftPneumatic();

float distToWallF();
float distToWallB();
float distToWallL();
float distToObject();

int getIntakeColor();
int get2ndIntakeColor();
void setIntakeColorLED(int value);
void setIntakeColor2LED(int value);

void setDriveEncoder(pros::motor_encoder_units_e mode);
float getLFPosition();
float getLMPosition();
float getLBPosition();
float getRFPosition();
float getRMPosition();
float getRBPosition();

void resetLeftMotorPosition();
void resetRightMotorPosition();
void resetDriveMotorPosition();

float getLeftMotorPosition();
float getRightMotorPosition();
float getLeftMotorPositionInInches();
float getRightMotorPositionInInches();

float wheelDegToInches(float degrees);
float wheelRotToInches(float rotations);

void resetIMUHeading();
float getHeading();
bool getLimitSwitch();

void driveFor(float speed, int ms);
void driveDistance(float distance, int timeout = 15000, int maxSpeed = 130);
void turn(float degrees, int timeout = 15000);

void resetLiftPosition();
void resetLiftPositionWithDistance();
void resetLiftWithDistTaskFunc();
float getLiftPosition();

void moveLiftToPos(float position,int speed = 127, int timeout = 5000);
void liftUpWallStake();
void liftPickup();
void liftDown();

void resetIntakePosition();
void setIntakeEncoder(pros::motor_encoder_units_e mode);
float getIntakePosition();

void intakeFor(int ms);
void intakeFor(float degrees);
void intakeFor(float speed, int ms);
void intakeFor(float speed, float degrees);

void outakeFor(float degrees);
void outakeFor(int ms);
void outakeFor(float speed, int ms);
void outakeFor(float speed, float degrees);

bool detectRed(int hue);
bool detectBlue(int hue);

bool sort_color(bool sort);
void sort_color_queue();
void startSorting();
void stopSorting();
void clearRingQueue();
void basicColorSort();

pros::vision_object_s_t getOurColorObject();
pros::vision_object_s_t getMostRelevantObject();
void turnToRing(int timeout = 15000, float maxSpeed = 130);
void driveToRing(int timeout = 15000, int maxSpeed = 130);
float calcDistance();
void driveFullVision(int timeout = 15000, int maxSpeed = 130);

int getIntakeDist();
void saveOurRing(int timeout = 15000);
void saveRing(int timeout = 15000);
void saveOurRing1(int timeout = 15000);
void saveRing1(int timeout = 15000);
void basketRings(bool withSave = true);
void liftIntakeWallStake();
void openRedirectAfterOurRing(int timeout = 1000);

void climb_piston_extend();
void climb_piston_retract();
void climb_piston_toggle();
void climb_up();


#endif
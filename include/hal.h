#ifndef _HAL_H_
#define _HAL_H_

#include "api.h"
#include "auton_basics.h"
#include "pros/motors.h"

#define LAT_KP  7
#define LAT_KI  0
#define LAT_KD  2

#define LAT_SMALL_RANGE 1
#define LAT_SMALL_RANGE_TIMEOUT 150

#define TURN_KP 0
#define TURN_KI 0
#define TURN_KD 0

#define VISION_KP 0.5
#define VISION_RANGE 15
#define VISION_RANGE_TIMEOUT 200

#define F_DISTANCE_OFFSET 6.25
#define L_DISTANCE_OFFSET 7.5

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
void setDriveBrake(pros::motor_brake_mode_e mode);

void spinIntake(int speed);
void stopIntake();
void stopIntakeHold();
void setIntakeBrake(pros::motor_brake_mode_e mode);

void openClamp();
void closeClamp();
void toggleClamp();

void toggleHood();
void hoodFwd();
void hoodBwd();
bool getHood();

void liftIntake();
void dropIntake();

void extendSweep();
void retractSweep();

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
float distToWallL();

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

void driveDistance(float distance, int timeout = 15000, int maxSpeed = 130);
void turn(float degrees, int timeout = 15000);

void resetLiftPosition();
float getLiftPosition();

void moveLiftToPos(float position, int timeout = 5000);
void liftUpWallStake();
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

pros::vision_object_s_t getOurColorObject();
pros::vision_object_s_t getMostRelevantObject();
void turnToRing(int timeout = 15000, float maxSpeed = 130);
void driveToRing(int timeout = 15000, int maxSpeed = 130);

int getIntakeDist();
void saveRingsAsTask(int speed = 127);
void saveRings();
void basketRings(bool withSave = true);


#endif
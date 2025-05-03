#ifndef _HAL_H_
#define _HAL_H_

#include "api.h"
#include "auton_basics.h"
#include "pros/motors.h"

#define LAT_KP  8
#define LAT_KI  0
#define LAT_KD  2

#define LAT_SMALL_RANGE 1
#define LAT_SMALL_RANGE_TIMEOUT 150

#define TURN_KP 4
#define TURN_KI 0
#define TURN_KD 0

#define VISION_CENTER 0
#define VISION_TURN_KP 0.8
#define VISION_TURN_KD 2
#define VISION_RANGE 20
#define VISION_RANGE_TIMEOUT 250
#define VISION_LAT_KP 1.2

#define F_DISTANCE_OFFSET 6.25

#define B_DISTANCE_OFFSET 2.5
#define L_DISTANCE_OFFSET 6
#define R_DISTANCE_OFSET 6
#define PROXI_OFFSET 1.5

extern bool COLOR;
extern int COLOR_SIG;

extern bool auton, autoSkill;
extern bool autoDrive, autoLift, autoIntake;

float distBetweenPts(float x1, float y1, float x2, float y2);

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
void stopLiftCoast();
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

void closePTO();
void openPTO();
void togglePTO();

void retractClimbBalance();

float distToWallF();
float distToWallB();
float distToWallL();
float distToWallR();
float distToObject();

int getIntakeColor();
int get2ndIntakeColor();
void setIntakeColorLED(int value);
void setIntakeColor2LED(int value);

void setDriveEncoder(pros::motor_encoder_units_e_t mode);
double getLFPosition();
double getLMPosition();
double getLBPosition();
double getRFPosition();
double getRMPosition();
double getRBPosition();

void resetLeftMotorPosition();
void resetRightMotorPosition();
void resetDriveMotorPosition();

double getLeftMotorPosition();
double getRightMotorPosition();
double getLeftMotorPositionInInches();
double getRightMotorPositionInInches();

double wheelDegToInches(double degrees);
double wheelRotToInches(double rotations);

void resetIMUHeading();
float getHeading();
bool getLBLimitSwitch();

void driveFor(float speed, int ms);
void driveDistance(float distance, int timeout = 15000, int maxSpeed = 130);
void turn(float degrees, int timeout = 15000);

void setLiftEncoder(pros::motor_encoder_units_e mode);
void setLiftZero(double pos);
void resetLiftPosition();
void resetLiftPositionWithDistance();
void resetLiftWithDistTaskFunc();
float getLiftPosition();

void moveLiftToPos(float position,int speed = 127, int timeout = 5000);
void moveLiftToPosCancel(float pos, int time, int speed = 127, int timeout = 5000);
void moveLiftToPosCancel(float pos, bool dir, int time, int speed = 127, int timeout = 5000);
void liftUpWallStake();
int moveToReset(float speed = 127);
void liftPickup();
void liftDown();

void resetIntakePosition();
void setIntakeEncoder(pros::motor_encoder_units_e mode);
float getIntakePosition();

int getRightLine();

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
pros::vision_object_s_t getMostRelevantObject(bool color = COLOR);
pros::vision_object_s_t getRed();
pros::vision_object_s_t getBlue();
bool checkRing(pros::vision_object_s_t ring);
void turnToRing(int timeout = 15000, float maxSpeed = 130, bool color = COLOR);
void turnToGoal(int timeout = 15000, float maxSpeed = 130);
void driveTowardsRing(int timeout = 15000, int maxSpeed = 130, bool color = COLOR);
struct driveToRingParams {
    float maxSpeed = 130;
    float maxDist = 300;
    float xLimit = 80;
    float yLimit = 80;
    bool driveThrough = false;
    bool keepDriving = false;
    bool color = COLOR;
    bool useLeftLine = false;
    bool useRightLine = false;
};
void driveToRing(int timeout = 15000, driveToRingParams params = {});
void moveToPointWithVis(float x, float y, int timeout = 15000, driveToRingParams params = {}, int delay = 0);
void turnToHeadingWithVis(float angle, int timeout = 15000, int range = 30, driveToRingParams prams = {},int delay = 0);
void turnToHeadingWithVisGoal(float angle, int timeout = 15000, int range = 30, int speed = 130, int delay = 0);
float calcDistance();
float calcDistanceGoal();
void driveFullVision(int timeout = 15000, int maxSpeed = 130);

int getIntakeDist();
void saveOurRing(int timeout = 15000);
void saveRing(int timeout = 15000);
void saveOurRing1(int timeout = 15000);
void saveRing1(int timeout = 15000);
void saveRingDist(int timeout = 15000);
void basketRings(bool withSave = true);
void liftIntakeWallStake();
void openRedirectAfterOurRing(int timeout = 1000);

void climb_piston_extend();
void climb_piston_retract();
void climb_piston_toggle();
void climb_up();


#endif
#ifndef _HAL_H_
#define _HAL_H_

#include "api.h"
#include "pros/motors.h"

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
void stopDrive();
void setDriveBrake(pros::motor_brake_mode_e mode);

void spinIntake(int speed);
void stopIntake();

void moveLift(int speed);
void stopLift();
void setLiftBrake(pros::motor_brake_mode_e mode);

void openClamp();
void closeClamp();

void hoodFwd();
void hoodBwd();

void liftIntake();
void dropIntake();

void extendSweep();
void retractSweep();

#endif
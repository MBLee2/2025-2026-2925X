#include "hal.h"
#include "auton_basics.h"
#include "pros/motors.h"
#include "robot_config.h"
#include "controls.h"


void stopAllMotors() {
    stopDrive();
    stopIntake();
    stopLift();
}

// Drive Base Movement
void spinLFMotor(int speed) {
    lf.move(speed);
}

void stopLFMotor() {
    lf.brake();
}

void spinLMMotor(int speed) {
    lm.move(speed);
}

void stopLMMotor() {
    lm.brake();
}

void spinLBMotor(int speed) {
    lb.move(speed);
}

void stopLBMotor() {
    lb.brake();
}

void spinRFMotor(int speed) {
    rf.move(speed);
}

void stopRFMotor() {
    rf.brake();
}

void spinRMMotor(int speed) {
    rm.move(speed);
}

void stopRMMotor() {
    rm.brake();
}

void spinRBMotor(int speed) {
    rb.move(speed);
}

void stopRBMotor() {
    rb.brake();
}

void spinLeftMotors(int speed) {
    spinLFMotor(speed);
    spinLMMotor(speed);
    spinLBMotor(speed);
}

void spinRightMotors(int speed) {
    spinRFMotor(speed);
    spinRMMotor(speed);
    spinRBMotor(speed);
}

void stopLeftMotors() {
    stopLFMotor();
    stopLMMotor();
    stopLBMotor();
}

void stopRightMotors() {
    stopRFMotor();
    stopRMMotor();
    stopRBMotor();
}

void drive(int leftSpeed, int rightSpeed) {
    spinLeftMotors(leftSpeed);
    spinRightMotors(rightSpeed);
}

void stopDrive() {
    stopLeftMotors();
    stopRightMotors();
}

void setDriveBrake(pros::motor_brake_mode_e mode) {
    lf.set_brake_mode(mode);
    lm.set_brake_mode(mode);
    lb.set_brake_mode(mode);
    rf.set_brake_mode(mode);
    rm.set_brake_mode(mode);
    rb.set_brake_mode(mode);
}

// Intake Movement
void spinIntake(int speed) {
    intake.move(speed);
}

void stopIntake() {
    intake.brake();
}

// Lift Movement
void moveLift(int speed) {
    lift.move(speed);
}

void stopLift() {
    lift.brake();
}

void setLiftBrake(pros::motor_brake_mode_e mode) {
    lift.set_brake_mode(mode);
}

// Clamp
void openClamp() {
    mogo_clamp.extend();
}

void closeClamp() {
    mogo_clamp.retract();
}

// Hood
void hoodFwd() {
    hood1.extend();
    hood2.extend();
}

void hoodBwd() {
    hood1.retract();
    hood2.retract();
}

// Intake Lift
void liftIntake() {
    intake_lift.retract();
}

void dropIntake() {
    intake_lift.extend();
}

// Sweeper
void extendSweep() {
    mogo_rush.extend();
}

void retractSweep() {
    mogo_rush.retract();
}
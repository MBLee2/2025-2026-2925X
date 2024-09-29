#include "hal.h"
#include "fmt/format.h"
#include "pros/motors.h"
#include "robot_config.h"
#include "controls.h"
#include "main.h"

bool basket_state = false;

//Basic Motor Movement

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

void driveStraight(int speed) {
    drive(speed, speed);
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


//Pneumatics

// Clamp
void openClamp() {
    mogo_clamp.extend();
}

void closeClamp() {
    mogo_clamp.retract();
}

// Hood

void hoodFwd() {
    basket_state = true;
    hood1.extend();
    hood2.extend();
}

void hoodBwd() {
    basket_state = false;
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

// Sixth-ring
void extendSixRing() {
    lastring.set_value(true);
}

void retractSixRing() {
    lastring.set_value(false);
}

// Sensors

//IMU
void resetIMUHeading() {
    imu.tare_heading();
}

float getHeading() {
    return imu.get_heading();
}

//Distance
int getIntakeDist() {
    return intake_dist.get();
}

// Motor Encoder
float getLFPosition() {
    return lf.get_position() * DRIVEBASE_GEAR_RATIO;
}

float getLMPosition() {
    return lm.get_position() * DRIVEBASE_GEAR_RATIO;
}

float getLBPosition() {
    return lb.get_position() * DRIVEBASE_GEAR_RATIO;
}

float getRFPosition() {
    return rf.get_position() * DRIVEBASE_GEAR_RATIO;
}

float getRMPosition() {
    return rm.get_position() * DRIVEBASE_GEAR_RATIO;
}

float getRBPosition() {
    return rb.get_position() * DRIVEBASE_GEAR_RATIO;
}

float getLeftMotorPosition() {
    return (getLFPosition() + getLMPosition() + getLBPosition()) / 3;
}

float getRightMotorPosition() {
    return (getRFPosition() + getRMPosition() + getRBPosition()) / 3;
}

float getLeftMotorPositionInInches() {
    return wheelDegToInches(getLeftMotorPosition());
}

float getRightMotorPositionInInches() {
    return wheelDegToInches(getLeftMotorPosition());
}

float getLiftPosition() {
    return lift.get_position() * LIFT_GEAR_RATIO;
}

float getIntakePosition() {
    return intake.get_position();
}

// Reset Motor Positions
void resetLeftMotorPosition() {
    lf.tare_position();
    lm.tare_position();
    lb.tare_position();
}

void resetRightMotorPosition() {
    rf.tare_position();
    rm.tare_position();
    rb.tare_position();
}

void resetDriveMotorPosition() {
    resetLeftMotorPosition();
    resetRightMotorPosition();
}

void resetLiftPosition() {
    lift.tare_position();
}

void resetIntakePosition() {
    intake.tare_position();
}

float wheelDegToInches(float degrees) {
    return (PI * DRIVEBASE_WHEEL_DIAMETER) * (degrees / 360);
}


// Controlled Functions

// Drive
void driveDistance(float distance, int timeout) {
    bool notReached = true;
    float error, prevError = 0, totalError = 0;
    float derivative;
    int counter = 0;
    resetDriveMotorPosition(); //TEST TO MAKE SURE THIS DOES NOT AFFECT LEMLIB
    while(notReached && timeout > 0 && (auton || autoSkill || autoDrive)) {
        float leftMotorsPosition = getLeftMotorPositionInInches();
        float rightMotorsPosition = getRightMotorPositionInInches();

        float averagePosition = (leftMotorsPosition + rightMotorsPosition) / 2;

        error = distance - averagePosition;

        derivative = error - prevError;

        totalError += error;
        if(error == 0 || (error > 0 && prevError < 0) || (error < 0 && prevError > 0))
            totalError = 0;

        float motorPower = (LAT_KP * error) + (LAT_KD * derivative) + (LAT_KI * totalError);

        driveStraight(motorPower);

        prevError = error;

        if(fabs(error) < LAT_SMALL_RANGE) {
            if(counter >= LAT_SMALL_RANGE_TIMEOUT) {
                notReached = false;
            } else {
                counter += 15;
            }
        } else {
            counter = 0;
        }

        pros::delay(15);
        timeout -= 15;
    }
}

void turn(float degrees, int timeout) {
    bool notReached = true;
    float error, prevError = 0, totalError = 0;
    float derivative;
    int counter = 0, passed = 0;
    resetIMUHeading(); // CHECK TO MAKE SURE THIS DOES NOT MESS WITH LEMLIB
    while(notReached && timeout > 0 && (auton || autoSkill || autoDrive)) {
        float currentHeading = getHeading();

        error = degrees - currentHeading;

        derivative = error - prevError;

        totalError += error;
        if(error == 0 || (error > 0 && prevError < 0) || (error < 0 && prevError > 0))
            totalError = 0;

        float motorPower = (TURN_KP * error) + (TURN_KD * derivative) + (TURN_KI * totalError);

        drive(motorPower, -motorPower);

        prevError = error;

        if(totalError == 0) {
            passed++;
        }

        if(passed >= 3) {
            notReached = false;
        }

        pros::delay(15);
        timeout -= 15;
    }
}

// Lift
void moveLiftToPos(float position){
    setLiftBrake(pros::E_MOTOR_BRAKE_HOLD);

    if(position > getLiftPosition()){
        moveLift(127);
        while(getLiftPosition() < position && (auton || autoSkill || autoLift)) {
            pros::delay(20);
        }
    } else if(position < getLiftPosition()){
        moveLift(-127);
        while(getLiftPosition() > position && (auton || autoSkill || autoLift)) {
            pros::delay(20);
        }
    }

    if(auton || autoSkill || autoLift)
        stopLift();
}

void liftUp(float degrees) {
    moveLiftToPos(getLiftPosition() + degrees);
}

void liftDown(float degrees) {
    moveLiftToPos(getLiftPosition() - degrees);
}

// Intake
void intakeFor(int ms){
    spinIntake(127);
    pros::delay(ms);

    if(auton || autoSkill || autoIntake)
        stopIntake();
}

void intakeFor(float degrees) {
    resetIntakePosition();
    spinIntake(127);
    while(getIntakePosition() < degrees && (auton || autoSkill || autoIntake)){
        pros::delay(20);
    }

    if(auton || autoSkill || autoIntake)
        stopIntake();
}

void intakeFor(float speed, int ms) {
    spinIntake(speed);
    pros::delay(ms);
    
    if(auton || autoSkill || autoIntake)
        stopIntake();
}

void intakeFor(float speed, float degrees) {
    resetIntakePosition();
    spinIntake(speed);
    while(getIntakePosition() < degrees && (auton || autoSkill || autoIntake)){
        pros::delay(20);
    }
    
    if(auton || autoSkill || autoIntake)
        stopIntake();
}

void outakeFor(int ms) {
    spinIntake(-127);
    pros::delay(ms);
    
    if(auton || autoSkill || autoIntake)
        stopIntake();
}

void outakeFor(float degrees) {
    resetIntakePosition();
    spinIntake(-127);
    while(getIntakePosition() > -degrees && (auton || autoSkill || autoIntake)){
        pros::delay(20);
    }
    
    if(auton || autoSkill || autoIntake)
        stopIntake();
}

void outakeFor(float speed, int ms) {
    spinIntake(-speed);
    pros::delay(ms);
    
    if(auton || autoSkill || autoIntake)
        stopIntake();
}

void outakeFor(float speed, float degrees) {
    resetIntakePosition();
    spinIntake(-speed);
    while(getIntakePosition() > -degrees && (auton || autoSkill || autoIntake)){
        pros::delay(20);
    }
    
    if(auton || autoSkill || autoIntake)
        stopIntake();
}

void saveRings(int timeout){
    spinIntake(127);
    if(getIntakeDist() > 20){
        while(getIntakeDist() > 20 && timeout > 0 && (auton || autoSkill || autoIntake)){
            pros::delay(10);
            timeout -= 10;
        }
        if(auton || autoSkill || autoIntake)
            stopIntake();
    }
}

void basketRings(bool withSave){
    autoIntake = true;
    if(withSave){
     saveRings();
    }

    if (basket_state == false)
    {
        spinIntake(90);
        while(getIntakeDist() < 50 && (auton || autoSkill || autoIntake))  // If hue matches specific values
        {
            pros::delay(10);
        }
        intakeFor(90, 23.f);
        outakeFor(105, 370);
    }
    spinIntake(127);
}
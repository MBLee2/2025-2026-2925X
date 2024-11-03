#include "hal.h"
#include "fmt/format.h"
#include "pros/motors.h"
#include "robot_config.h"
#include "controls.h"
#include "main.h"

bool basket_state = false;
bool COLOR = false; // false = red, true = blue

//Basic Motor Movement

void stopAllMotors() {
    stopDrive();
    stopIntake();
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
    intakeL.move(speed);
    intakeR.move(speed);
}

void stopIntake() {
    intakeR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intakeR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intakeL.brake();
    intakeR.brake();
}

void stopIntakeHold() {
    intakeL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intakeR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intakeL.brake();
    intakeR.brake();
}


void setIntakeBrake(pros::motor_brake_mode_e mode) {
    intakeL.set_brake_mode(mode);
    intakeR.set_brake_mode(mode);
}



//Pneumatics

// Clamp
void openClamp() {
    mogo_clamp.extend();
    //mogo_clamp2.extend();
}

void closeClamp() {
    mogo_clamp.retract();
    //mogo_clamp2.retract();
}

void toggleClamp() {
    mogo_clamp.toggle();
    //mogo_clamp2.toggle();
}


// Hood

void toggleHood(){
    if(redirect1.is_extended()){
        hoodBwd();
    }else{
        hoodFwd();
    }
}

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
    //lastring.set_value(true);
}

void retractSixRing() {
    //lastring.set_value(false);
}

void toggleSixRing() {
    //lastring.toggle();
}

//Redirect
void redirectRings() {
    redirect1.extend();
}

void closeRedirect() {
    redirect1.retract();
}

void toggleRedirect() {
    redirect1.toggle();
}

//Lift Helper
void liftPneumaticUp() {
    lift_helper1.extend();
    lift_helper2.extend();
}

void liftPneumaticDown() {
    lift_helper1.retract();
    lift_helper2.retract();
}

bool getLiftHelper() {
    return lift_helper1.is_extended();
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
int getIntakeColor() {
    return intake_color.get_hue();
}

//Limit Switch
bool getLimitSwitch() {
    return limitSwitch.get_value();
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

void resetLiftPosition(){
    lift_rotation.reset();
    lift_rotation.reset_position();
    lift_rotation.set_position(0);
}

float getLiftPosition() {
    return lift_rotation.get_angle() / 100.0;
}

void setIntakeEncoder(pros::motor_encoder_units_e mode) {
    intakeL.set_encoder_units(mode);
    intakeR.set_encoder_units(mode);
}

float getIntakePosition() {
    return (intakeL.get_position() + intakeR.get_position()) / 2.0;
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


void resetIntakePosition() {
    intakeL.tare_position();
    intakeR.tare_position();
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
void sort_color(bool sort) {
    //false = red, true = blue 
    int hue = getIntakeColor();
    if (sort == true){

        if(COLOR == false) // Sorting out Red
        {
            printf("Sorting out red \n"); // Log the function exit for debugging
            if(hue >= 0 && hue <= 25)
            {
                master.rumble("-");
                printf("Detected Red \n"); // Log the function exit for debugging
                redirectRings();
                pros::delay(500);

            }
            closeRedirect();
        }
        if(COLOR == true)// Sorting out Blue
        {
            if(hue >= 200 && hue <= 215)
            {
                printf("Detected Blue \n"); // Log the function exit for debugging
                redirectRings();
                pros::delay(500);
            }
            closeRedirect();
        }
    }
}


/*void saveRings(int timeout){
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
}*/
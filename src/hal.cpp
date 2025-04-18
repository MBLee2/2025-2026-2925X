#include "hal.h"
#include "auton_basics.h"
#include "auton_routines.h"
#include "fmt/format.h"
#include "pros/device.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/vision.h"
#include "robot_config.h"
#include "controls.h"
#include "main.h"
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>
#include <type_traits>


bool COLOR = false; // true = red, false = blue
int COLOR_SIG = (COLOR) ? 1 : 2;


bool auton = false, autoSkill = false;
bool autoDrive = false, autoLift = false, autoIntake = false, isintaking = false, LBPickup = false;

std::queue<bool> ringQueue;


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
void stopDriveHold() {
    setDriveBrake(pros::E_MOTOR_BRAKE_HOLD);
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
    if(speed == 0)
    {
        isintaking = false;
    }
    isintaking = true;
}

void stopIntake() {
    setIntakeBrake(pros::E_MOTOR_BRAKE_COAST);
    isintaking = false;
    intake.brake();
}

void stopIntakeHold() {
    setIntakeBrake(pros::E_MOTOR_BRAKE_HOLD);
    isintaking = false;
    intake.brake();
}


void setIntakeBrake(pros::motor_brake_mode_e mode) {
    intake.set_brake_mode(mode);
    //intakeR.set_brake_mode(mode);
}

void intakeAntiJam() {
    if(intake.get_actual_velocity() < 5)
    {
        spinIntake(-127);
        pros::delay(100);
        spinIntake(127);
    }
}
void intakeAntiJamTaskFunc(){
    int counter = 0;
    while (true) {
        if(intake.get_actual_velocity() < 5 && isintaking && !LBPickup)
        {
            counter++;
        }
        if(counter >= 15)
        {
            intakeAntiJam();
            counter = 0;
        }
        pros::delay(20);
    }
}


// Lift Movement
void spinLift(int speed) {
    ladybrown.move(speed);
}

void stopLift(){
    setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
    ladybrown.brake();
}
void stopLiftHold() {
    setLiftBrake(pros::E_MOTOR_BRAKE_HOLD);
    ladybrown.brake();
}


void setLiftBrake(pros::motor_brake_mode_e mode) {
    ladybrown.set_brake_mode_all(mode);
}


//Pneumatics

// Clamp
void openClamp() {
    mogo_clamp.retract();
}

void closeClamp() {
    mogo_clamp.extend();
}

void toggleClamp() {
    mogo_clamp.toggle();
}

// leftsweeper
void extendLeftSweeper() {
    left_sweeper.extend();
}

void retractLeftSweeper() {
    left_sweeper.retract();
}

void toggleLeftSweeper() {
    left_sweeper.toggle();
}

// right sweeper
void extendRightSweeper() {
    right_sweeper.extend();
}

void retractRightSweeper() {
    right_sweeper.retract();
}

void toggleRightSweeper() {
    right_sweeper.toggle();
}
//IntakeLift
void liftIntake() {
    intake_lift.extend();
}
void dropIntake()
{
    intake_lift.retract();
}


//IMU
void resetIMUHeading() {
    imu.tare_heading();
}

float getHeading() {
    return imu.get_heading();
}



//Distance
int getFrontDistance() {
    return distance_front.get();
}

int getBackDistance() {
    return distance_back.get();
}

int getLeftDistance() {
    return distance_left.get();
}

int getRightDistance() {
    return distance_right.get();
}

int getProximity() {
    return distance_proxi.get();
}

float distToWallF() {
    return (getFrontDistance() / 25.4) + F_DISTANCE_OFFSET;
}

float distToWallB() {
    return (getBackDistance() / 25.4) + B_DISTANCE_OFFSET;
}

float distToWallL() {
    return (getLeftDistance() / 25.4) + L_DISTANCE_OFFSET;
}

float distToWallR() {
    return (getRightDistance() / 25.4) + R_DISTANCE_OFSET;
}

float distToObject() {
    return (getProximity() / 25.4) + PROXI_OFFSET;
}

int getIntakeDist(){
    return intake_dist.get();
}

//Color
int getIntakeColor() {
    return intake_color.get_hue();
}

int get2ndIntakeColor() {
    return intake_color2.get_hue();
}

void setIntakeColorLED(int value){
    intake_color.set_led_pwm(value);
}

void setIntakeColor2LED(int value){
    intake_color2.set_led_pwm(value);
}

// Limit Switch
bool getLBLimitSwitch() {
    return LB_limit.get_value();
}

// Vision Sensor
pros::vision_object_s_t getRed() {
    return vision_sensor.get_by_sig(0, 1);
}
pros::vision_object_s_t getBlue() {
    return vision_sensor.get_by_sig(0, 2);
}

pros::vision_object_s_t getOurColorObject() {
    return vision_sensor.get_by_sig(0, COLOR_SIG);
}

void swap_objects(pros::vision_object_s_t obj_arr[], int a, int b){
    pros::vision_object_s_t temp = obj_arr[a];
    obj_arr[a] = obj_arr[b];
    obj_arr[b] = temp;
}

pros::vision_object_s_t getMostRelevantObject(bool color) {
    pros::vision_object_s_t object_arr[5];
    int availableObjects = vision_sensor.read_by_sig(0, (color) ? 1 : 2, 5, object_arr);

    int highestY = 0;
    int highestYIndex = 0;

    for(int i = 0; i < 5; i++){
        if(object_arr[i].signature != VISION_OBJECT_ERR_SIG){
            if(abs(object_arr->x_middle_coord - VISION_CENTER) > 120){
                object_arr[i].signature = VISION_OBJECT_ERR_SIG;
                availableObjects--;
            } else if (object_arr[i].y_middle_coord > highestY){
                highestY = object_arr[i].y_middle_coord;
                highestYIndex = i;
            }
        }
    }

    if(availableObjects <= 1){
        return object_arr[highestYIndex];
    }

    int lowestXOffset = 160;
    int lowestXOffsetIndex = 0;

    for(int i = 0; i < 5; i++){
        if(object_arr[i].signature != VISION_OBJECT_ERR_SIG){
            if(abs(object_arr[i].y_middle_coord - highestY) < 10) {
                if(abs(object_arr[i].x_middle_coord - VISION_CENTER) < lowestXOffset){
                    lowestXOffset = abs(object_arr[i].x_middle_coord - VISION_CENTER);
                    lowestXOffsetIndex = i;
                }
            }
        }
    }

    return object_arr[lowestXOffsetIndex];
}

// Motor Encoder
void setDriveEncoder(pros::motor_encoder_units_e_t mode){
    lf.set_encoder_units(mode);
    lm.set_encoder_units(mode);
    lb.set_encoder_units(mode);
    rf.set_encoder_units(mode);
    rm.set_encoder_units(mode);
    rb.set_encoder_units(mode);
}

double getLFPosition() {
    return lf.get_position() * DRIVEBASE_GEAR_RATIO;
}

double getLMPosition() {
    return lm.get_position() * DRIVEBASE_GEAR_RATIO;
}

double getLBPosition() {
    return lb.get_position() * DRIVEBASE_GEAR_RATIO;
}

double getRFPosition() {
    return rf.get_position() * DRIVEBASE_GEAR_RATIO;
}

double getRMPosition() {
    return rm.get_position() * DRIVEBASE_GEAR_RATIO;
}

double getRBPosition() {
    return rb.get_position() * DRIVEBASE_GEAR_RATIO;
}

double getLeftMotorPosition() {
    return getLFPosition();
}

double getRightMotorPosition() {
    return getRFPosition();
}

double getLeftMotorPositionInInches() {
    return wheelRotToInches(getLeftMotorPosition());
}

double getRightMotorPositionInInches() {
    return wheelRotToInches(getRightMotorPosition());
}

void resetLiftPosition(){
    ladybrownL.tare_position();
    ladybrownR.tare_position();
}
void resetLiftPositionWithDistance(){
    if(getLBLimitSwitch())
    { 
        LBPickup = false;
        resetLiftPosition();
    }
}
void resetLiftWithDistTaskFunc(){
    while (true) {
        resetLiftPositionWithDistance();
        pros::delay(20);
    }
}

void setLiftEncoder(pros::motor_encoder_units_e mode) {
    ladybrownL.set_encoder_units(mode);
    ladybrownR.set_encoder_units(mode);
}


float getLiftPosition() {
    float pos = (ladybrownL.get_position() + ladybrownR.get_position()) / 2 ;
    return (pos);
}

void setIntakeEncoder(pros::motor_encoder_units_e mode) {
    intake.set_encoder_units(mode);
}

float getIntakePosition() {
    return (intake.get_position());
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
    intake.tare_position();
}

double wheelDegToInches(double degrees) {
    return (PI * DRIVEBASE_WHEEL_DIAMETER) * (degrees / 360.0);
}

double wheelRotToInches(double rotations){
    return (PI * DRIVEBASE_WHEEL_DIAMETER) * rotations;
}

int getRightLine() {
    return lineRight.get_value();
}

int getLeftLine() {
    return lineLeft.get_value();
}

bool detectLine(int value) {
    return value < 2700;
}

bool detectRightLine() {
    return detectLine(getRightLine());
}

bool detectLeftLine() {
    return detectLine(getLeftLine());
}

// Controlled Functions

void driveFor(float speed, int ms){
    driveStraight(speed);
    pros::delay(ms);
    stopDrive();
}

// Drive
void driveDistance(float distance, int timeout, int maxSpeed) {
    float startLeft = getLeftMotorPositionInInches();
    float startRight = getRightMotorPositionInInches();
    autoDrive = true;
    bool notReached = true;
    float error, prevError = 0, totalError = 0;
    float derivative;
    int counter = 0;
    pros::delay(50);
    while(notReached && timeout > 0 && (auton || autoSkill) && autoDrive) {
        float leftMotorsPosition = getLeftMotorPositionInInches() - startLeft;
        float rightMotorsPosition = getRightMotorPositionInInches() - startRight;

        float averagePosition = (leftMotorsPosition + rightMotorsPosition) / 2.0;

        error = distance - averagePosition;
        printf("error: %f\t", error);

        derivative = error - prevError;

        totalError += error;
        if(error == 0 || (error > 0 && prevError < 0) || (error < 0 && prevError > 0))
            totalError = 0;

        float motorPower = (LAT_KP * error) + (LAT_KD * derivative) + (LAT_KI * totalError);
        printf("motor power: %f\n", motorPower);

        if(motorPower > maxSpeed){
            motorPower = maxSpeed;
        } 
        else if(motorPower < -maxSpeed){
            motorPower = -maxSpeed;
        }

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

    stopDrive();
}

void turn(float degrees, int timeout) {
    autoDrive = true;
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

    stopDrive();
}


// Lift
void liftUpWallStake() {
    ladybrown.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    moveLiftToPos(240,127);
    printf("WALL STAKE");
}

void liftPickup() {
    int time = 0;
    LBPickup = true;
    autoLift = true;
    ladybrown.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    if(getLiftPosition() < 70)
    {
        while (!getLBLimitSwitch() && time < 1200 && autoLift) {
            spinLift(-40);
            pros::delay(20);
            time=+20;
        }
        resetLiftPositionWithDistance();
        int count = 0;
        while (getLiftPosition() < 40 && time < 1200 && autoLift) {
            spinLift(25);
            pros::delay(15);
            time =+15;
            count++;
        }
    } else {
        float error, prevError, derivative;
        while(fabs(45 - getLiftPosition()) > 5 && time < 1200 && autoLift){
            error = 45 - getLiftPosition();

            derivative = error - prevError;

            float motorPower = 0.8 * error;

            spinLift(motorPower);

            prevError = error;
            pros::delay(20);
            time += 20;
        }
    }
    stopLift();
}

void liftDown() {
    moveLiftToPos(4);
}

void moveLiftToPos(float pos,int speed,int timeout){
    
    int time = pros::millis();
    autoLift = true;

    if(pos <= 0){
        pos = 0;
    }
    else if(pos >= 2000){
        pos = 2000;
    }
    autoLift = true;
    if(getLiftPosition() > pos){

        while(getLiftPosition() >= pos && (pros::millis() - time) < timeout && autoLift)
        {
            spinLift(-speed);
            pros::delay(20);
            printf("Lift %f\n",getLiftPosition());
        }
        stopLift();
    }
    else if(getLiftPosition() < pos){
        
        while(getLiftPosition() <= pos && (pros::millis() - time) < timeout && autoLift)
        {
            spinLift(speed);
            pros::delay(20);
            printf("Lift %f\n",getLiftPosition());
        }
        stopLift();
    }
    autoLift = false;
}

void moveLiftToPosCancel(float pos, int time, int speed, int timeout){
    if(autoLift){
        if(getLiftPosition() > pos){
            spinLift(-speed);
            pros::delay(20);
            if(getLiftPosition() < pos || pros::millis() - time > timeout)  {
                autoLift = false;
                stopLift();
            }
        } else if(getLiftPosition() < pos){
            spinLift(speed);
            pros::delay(20);
            if(getLiftPosition() > pos || pros::millis() - time > timeout) {
                autoLift = false;
                stopLift();
            }
        }
    }
}

void moveLiftToPosCancel(float pos, bool dir, int time, int speed, int timeout){
    if(autoLift){
        if((getLiftPosition() < pos) == dir && pros::millis() - time < timeout){
            spinLift(dir ? speed : -speed);
        } else {
            autoLift = false;
            stopLift();
        }
    }
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


// Color Sorting

bool sort_color(bool sort) {
    //false = red, true = blue 
    int hue = getIntakeColor();
    int hue2 = get2ndIntakeColor();
    int timeout = 0;
    if (sort == true){

        setIntakeColorLED(100);
        setIntakeColor2LED(100);

        if(COLOR == false) // Sorting out Red
        {
            //printf("Sorting out red \n"); // Log the function exit for debugging 
            if(hue >= 0 && hue <= 25)
            {
                printf("Detected Red \n"); // Log the function exit for debugging
                while(hue2 >= 155 && hue2 <= 185){
                    pros::delay(20);
                    hue2 = get2ndIntakeColor();
                }

                redirectRings();

                while((hue2 < 0 || hue2 > 25) && timeout < 500){
                    pros::delay(20);
                    hue2 = get2ndIntakeColor();
                    timeout += 20;
                }

                timeout = 0;
                while((hue2 >= 0 && hue2 <= 25) && timeout < 500){
                    pros::delay(20);
                    hue2 = get2ndIntakeColor();
                    timeout += 20;
                }

                pros::delay(50);
                closeRedirect();
                return true;
            }
        }
        if(COLOR == true)// Sorting out Blue
        {
            if(hue >= 135 && hue <= 185)
            {
                printf("Detected Blue \n"); // Log the function exit for debugging
                while(hue2 >= 0 && hue2 <= 25){
                    pros::delay(20);
                    hue2 = get2ndIntakeColor();
                } 

                redirectRings();

                while((hue2 < 155 || hue2 > 185) && timeout < 500){
                    pros::delay(20);
                    hue2 = get2ndIntakeColor();
                    timeout += 20;
                }

                timeout = 0;
                while((hue2 >= 155 && hue2 <= 185) && timeout < 500){
                    pros::delay(20);
                    hue2 = get2ndIntakeColor();
                    timeout += 20;
                }

                pros::delay(50);
                closeRedirect();
                return true;
            }
        }
    }
    return false;
}

bool detectRingFront(){
    return distToObject() < 3.7;
}

int redLower = 345;
int redUpper = 15;

int blueLower = 210;
int blueUpper = 250;

bool detectRing() {
    return getIntakeDist() < 30;
}

bool detectRing(int hue){
    return (hue >= 310 || hue <= 345) || (hue >= 210 && hue <= 250);
}

bool detectRed(int hue){
    return hue >= 340 || hue <= 10;
}

bool detectBlue(int hue){
    return hue >= 200 && hue <= 250;
}

bool detectOurColor(int hue){
    if(COLOR){
        return detectRed(hue);
    } else {
        return detectBlue(hue);
    }
}

bool detectTheirColor(int hue){
    if(COLOR){
        return detectBlue(hue);
    } else {
        return detectRed(hue);
    }
}

void printRingQueue(){
    if(!ringQueue.empty()){
        bool first;
        for(int i = 0; i < ringQueue.size(); i++){
            first = ringQueue.front();
            if(first){
                printf("Red  ");
            } else {
                printf("Blue ");
            }
            ringQueue.pop();
            ringQueue.push(first);
        }
        printf("\n");
    }
}


void clearRingQueue() {
    while(!ringQueue.empty()){
        ringQueue.pop();
    }
}

void addCurrentRing(){
    while(true){
        if(autoIntake){
            int hue = getIntakeColor();

            if(detectRed(hue)) //If we detect red
            {
                printf("Enter red %i\n", hue);
                ringQueue.push(true); //Add to queue as upcoming
                
                while(detectRed(hue)){ //Wait for ring to continue through intake
                    pros::delay(20);
                    hue = getIntakeColor();
                }
                printRingQueue();
            }
            else if(detectBlue(hue)) //If we detect blue
            {
                printf("Enter blue %i\n", hue);
                ringQueue.push(false); //Add to queue as upcoming

                while(detectBlue(hue)){ //Wait for ring to continue through intake
                    pros::delay(20);
                    hue = getIntakeColor();
                }
                printRingQueue();
            }
        }
        pros::delay(10);
    }
}

void checkQueue() {
    // while(true){
    //     if(autoIntake){
    //         if(!ringQueue.empty()){
    //             if(ringQueue.front() == COLOR && getRedirect()){ //If next ring is ours and redirect is up
    //                 closeRedirect(); //close redirect
    //                 printf("Scoring rings\n");
    //             } else if(ringQueue.front() != COLOR && !getRedirect()) { //If next ring is opp.'s and redirect is closed
    //                 redirectRings(); //open redirect
    //                 printf("Sorting out\n");
    //             }
    //         }
    //     }
    //     pros::delay(5);
    // }
}

void waitForExitRed(){
    int hue = get2ndIntakeColor();

    while(detectRed(hue)){
        pros::delay(20);
        hue = get2ndIntakeColor();
    }
}

void waitForExitBlue(){
    int hue = get2ndIntakeColor();

    while(detectBlue(hue)){
        pros::delay(20);
        hue = get2ndIntakeColor();
    }
}

void waitForExitRing() {
    while(detectRing()){
        pros::delay(20);
    }
    if(ringQueue.front()){
        printf("Exiting Red \n");
    } else {
        printf("Exiting Blue \n");
    }
}

void countRings() {
    while(true){
        if(autoIntake && !ringQueue.empty()){
            int hue = get2ndIntakeColor();

            if(detectRed(hue) && ringQueue.front() == true) //If we detect red
            {
                if(COLOR == true){ //If we are red 
                    waitForExitRed(); //wait for it to score to avoid opening early
                    ringQueue.pop(); //then remove from queue
                    printf("Exiting red %i\n", hue);
                    pros::delay(50);
                } else { //If we are not red
                    pros::delay(30);
                    ringQueue.pop(); //remove from queue as it's exiting to close as early as possible
                    printf("Exiting red %i\n", hue);
                    waitForExitRed(); //wait for it to exit
                }
            }

            if(detectBlue(hue) && ringQueue.front() == false) //If we detect blue
            {
                if(COLOR == false){ //If we are blue
                    waitForExitBlue(); //wait for it to score to avoid opening early
                    ringQueue.pop(); //then remove from queue
                    printf("Exiting blue %i\n", hue);
                    pros::delay(50);
                } else {
                    pros::delay(30);
                    ringQueue.pop(); //remove from queue as it's exiting to close as early as possible
                    printf("Exiting blue %i\n", hue);
                    waitForExitBlue(); //wait for it to exit
                }
            }
        }
        pros::delay(10);
    }
}

void countRingsDist() {
    while(true) {
        if(autoIntake && !ringQueue.empty()){
            if(detectRing())
            {
                printRingQueue();
                if(COLOR == ringQueue.front()){
                    waitForExitRing();
                    ringQueue.pop();
                } else {
                    printf("sorting out\n");
                    waitForExitRing();
                    //pros::delay(100);
                    spinIntake(-127);
                    ringQueue.pop();
                    pros::delay(70);
                    spinIntake(127);
                }
            }
        }
        pros::delay(20);
    }
}

void sort_color_queue(){
    printf("Sorting started\n");
    pros::Task add_ring_task(addCurrentRing);
    pros::Task count_ring_task(countRingsDist);
}

void startSorting() {
    if(!autoIntake ){
        master.print(1,0,"Sorting");
        autoIntake = true;
    }
}

void stopSorting() {
    if(autoIntake){
        master.clear_line(1);
        autoIntake = false;
        clearRingQueue();
    }
}


//Vision sensor
/*void driveToRing(int timeout){
    int hue = getIntakeColor();

    while(!detectOurColor(hue) && timeout > 0){

        float latPower = 50;
        float turnPower = 20;

        pros::vision_object_s_t nearestRing = getOurColorObject();

        if(nearestRing.signature != VISION_OBJECT_ERR_SIG){

        }

        pros::delay(15);
        timeout -= 15;
    }

    stopDrive();
}*/

bool checkRing(pros::vision_object_s_t ring){
    return (ring.x_middle_coord != 0 || ring.y_middle_coord != 0) && abs(ring.x_middle_coord) < 160 && abs(ring.y_middle_coord) < 110;
}

void turnToRing(int timeout, float maxSpeed, bool color){
    bool reached = false;
    int counter = 0;
    int error;

    while(!reached && timeout > 0){

        pros::vision_object_s_t nearestRing = getMostRelevantObject(color);

        if(checkRing(nearestRing)){

            error = nearestRing.x_middle_coord - VISION_CENTER;

            float motorPower = (VISION_TURN_KP * error);

            if(motorPower > maxSpeed){
                motorPower = maxSpeed;
            } else if (motorPower < -maxSpeed){
                motorPower = -maxSpeed;
            }

            drive(motorPower, -motorPower);

            printf("(%d, %d)\t Error: %d, motorPower: %f\n", nearestRing.x_middle_coord, nearestRing.y_middle_coord, error, motorPower);
            

            if(abs(error) < VISION_RANGE) {
                counter += 20;
                if(counter >= VISION_RANGE_TIMEOUT) {
                    reached = true;
                } 
            } else {
                counter = 0;
            }
        }

        pros::delay(20);
        timeout -= 20;
    }

    stopDrive();
}


void driveTowardsRing(int timeout, int maxSpeed, bool color){
    int hueLower = (COLOR) ? redLower : blueLower, hueUpper = (COLOR) ? redUpper : blueUpper;
    float motorPower;

    pros::vision_object_s_t ring = getMostRelevantObject(color);
    if(checkRing(ring) && ring.y_middle_coord < 100){
        while((getIntakeColor() < hueLower || getIntakeColor() > hueUpper) && timeout > 0){
            ring = getMostRelevantObject();

            if(checkRing(ring)){
                int error = 106 + ring.y_middle_coord;
                motorPower = VISION_LAT_KP * error;

                if(motorPower > maxSpeed){
                    motorPower = maxSpeed;
                }
            }

            driveStraight(motorPower);
            printf("(%d, %d)\n", ring.x_middle_coord, ring.y_middle_coord);

            pros::delay(30);
            timeout -= 30;

        }

        stopDrive();
    }
}

float limitSpeed(float speed, float maxSpeed){
    return (maxSpeed - 15) / (1 + pow(M_E, (-1./15) * (speed - ((maxSpeed + 30) / 2)))) + 15;
}

float distBetweenPts(float x1, float y1, float x2, float y2){
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

/** driveToRingParams
 *  @param maxSpeed maximum speed
 *  @param maxDist maximum distance allowed to travel
 *  @param xLimit X coordinate that cannot be crossed
 *  @param yLimit Y coordinate that cannot be crossed
 *  @param driveThrough use color sensor to stop (as opposed to distance)
 *  @param keepDriving continue seeking all available rings (if driveThrough is false, this is ignored)
 *  @param color which color to seek
 *  @param useLeftLine use left line tracker to avoid crossing line (not done)
 *  @param useRightLine use right line tracker to avoid crossing line (not done)
 */
void driveToRing(int timeout, driveToRingParams params) {

    autoDrive = true;
    float motorPower = params.maxSpeed, turnPower, ringPower;
    //float initLeft = getLeftMotorPositionInInches(), initRight = getRightMotorPositionInInches();
    printf("(%f, %f) - before reset\n", chassis.getPose().x, chassis.getPose().y);
    //resetDriveMotorPosition();
    printf("(%f, %f) - after reset\n", chassis.getPose().x, chassis.getPose().y);
    //printf("Left: %f\t Right: %f\n", initLeft, initRight);
    float distTravelled, drive_error = params.maxDist, derivative, prevError;
    float leftSpeed, rightSpeed;
    lemlib::Pose initialPose = chassis.getPose();
    bool startLessX = initialPose.x < params.xLimit, startLessY = initialPose.y < params.yLimit;

    printf("(%f, %f) - before loop\n", chassis.getPose().x, chassis.getPose().y);
    while(timeout > 0 && (auton || autoSkill || autoDrive)) {

        pros::vision_object_s_t nearestRing = getMostRelevantObject(params.color);

        if(params.driveThrough){
            if(!params.keepDriving || !checkRing(nearestRing)){
                if(detectOurColor(getIntakeColor())) { 
                    printf("Stop by intake");
                    break; 
                }
            }
        } else if(detectRingFront()) { 
            printf("Stop by intake");
            break; 
        }

        lemlib::Pose currentPose = chassis.getPose();
        //float leftPos = getLeftMotorPositionInInches(), rightPos = getRightMotorPositionInInches();
        float dist = distBetweenPts(currentPose.x, currentPose.y, initialPose.x, initialPose.y);
        distTravelled += dist;
        if(timeout % 30 == 0){
            printf("Current dist: %f\t Total dist: %f\t", dist, distTravelled);
        }

        if(distTravelled > params.maxDist || 
                startLessX != (currentPose.x < params.xLimit) || 
                startLessY != (currentPose.y < params.yLimit)) {
            printf("Stop by maximum\n");
            break;
        }
        drive_error = std::min({params.maxDist - distTravelled, 
            (float) fabs((1/sin(deg2rad(currentPose.theta))) * (params.xLimit - currentPose.x)), 
            (float) fabs((1/cos(deg2rad(currentPose.theta))) * (params.yLimit - currentPose.y))});

        derivative = drive_error - prevError;
        motorPower = (LAT_KP * drive_error) + (LAT_KD * derivative);

        if(checkRing(nearestRing)){
            if(timeout % 90 == 0){
                //printf("(%d, %d)\t", nearestRing.x_middle_coord, nearestRing.y_middle_coord);
            }
            int vision_error = nearestRing.x_middle_coord - VISION_CENTER;

            turnPower = VISION_TURN_KP * vision_error;

            if(nearestRing.y_middle_coord < 0 && !params.keepDriving){
                int error = 106 + nearestRing.y_middle_coord;
                ringPower = VISION_LAT_KP * error;
                if(ringPower < motorPower){
                    motorPower = ringPower;
                }
            }
        }
        if(timeout % 30 == 0){
            printf("(%f, %f)\t", currentPose.x, currentPose.y);
            printf("DistToMax: %f\t", drive_error);
        }

        if(params.useLeftLine && params.useRightLine && detectLeftLine() && detectRightLine()){
            driveStraight(-motorPower);
            break;
        } else if(params.useRightLine && detectRightLine()){
            printf("Detecting line\t Theta: %f\t", chassis.getPose().theta);
            if(COLOR){
                turnPower = TURN_KP * fmod(-90 - currentPose.theta, 360);
            } else {
                turnPower = TURN_KP * fmod(90 - currentPose.theta, 360);
            }
        } else if(params.useLeftLine && detectLeftLine()){
            printf("Detecting line\t Theta: %f\t", chassis.getPose().theta);
            if(COLOR){
                turnPower = TURN_KP * fmod(90 - currentPose.theta, 360);
            } else {
                turnPower = TURN_KP * fmod(-90 - currentPose.theta, 360);
            }
        }

        leftSpeed = limitSpeed(motorPower + turnPower, params.maxSpeed), rightSpeed = limitSpeed(motorPower - turnPower, params.maxSpeed);

        if(timeout % 30 == 0){
            printf("Left: %f\t Right %f\t", leftSpeed, rightSpeed);
            printf("\n");
        }
        drive(leftSpeed, rightSpeed);
        prevError = drive_error;
        initialPose = currentPose;

        pros::delay(15);
        timeout -= 15;
        
    }
    if(timeout <= 0){
        printf("Stop by timeout");
    }

    stopDrive();
    printf("\n");
}

/** driveToRingParams are same as driveToRing except for maxDist
 *  maxDist counts distance beyond distance to the original point
 */
void moveToPointWithVis(float x, float y, int timeout, driveToRingParams params, int delay){
    chassis.moveToPoint(x, y, timeout, {.maxSpeed = params.maxSpeed});
	int temp = pros::millis();
	while((!checkRing(getMostRelevantObject(params.color)) || pros::millis() - temp < delay) && chassis.isInMotion()){
		pros::delay(20);
	}
    lemlib::Pose currentPose = chassis.getPose();
    printf("(%f, %f)\tMax: %f\n", currentPose.x, currentPose.y, params.maxDist);
	if(chassis.isInMotion()){
		chassis.cancelAllMotions();
        pros::delay(10);
        currentPose = chassis.getPose();
        params.maxDist += sqrt(pow(x - currentPose.x, 2) + pow(y - currentPose.y, 2));
        printf("(%f, %f)\tMax: %f\n", currentPose.x, currentPose.y, params.maxDist);
		driveToRing(timeout - (pros::millis() - temp), params);
	}

}

bool basket_state;

float calcDistance(){

    pros::vision_object_s_t nearestRing = getMostRelevantObject();

    return (7 * 158.) / (nearestRing.width * tan(deg2rad(32.3)));
}

void driveFullVision(int timeout, int maxSpeed) {
    int hueLower = (COLOR) ? redLower : blueLower, hueUpper = (COLOR) ? redUpper : blueUpper;
    autoDrive = true;
    float turnPower, motorPower = maxSpeed;
    float distance, prevDistance = 0, derivative = 0;
    pros::delay(50);
    while((getIntakeColor() < hueLower || getIntakeColor() > hueUpper) && timeout > 0 && (auton || autoSkill) && autoDrive) {

        pros::vision_object_s_t nearestRing = getMostRelevantObject();

        if(nearestRing.signature != VISION_OBJECT_ERR_SIG){

            int vision_error = nearestRing.x_middle_coord - VISION_CENTER;

            turnPower = VISION_TURN_KP * vision_error;

            distance = calcDistance();

            derivative = distance - prevDistance;

            motorPower = LAT_KP * distance + LAT_KD * derivative;

            //printf("Drive: %f\t Turn: %f\n", motorPower, turnPower);

            prevDistance = distance;

            if(motorPower > maxSpeed){
                motorPower = maxSpeed;
            }
        }

        drive(motorPower + turnPower, motorPower - turnPower);
        //printf("Motor Power: %f\n", motorPower);

        pros::delay(15);
        timeout -= 15;
    }
    stopDrive();
}




/*void saveRingsAsTask(int speed)
{
    pros::Task intake_task(saveRings);
}*/

void saveOurRing(int timeout){
    int time = 0;
    while (true) {
        if (time >= timeout)
        {
            return;
        }
        int hue = get2ndIntakeColor();
        //printf("Color %d\n",hue);
        if(detectOurColor(hue))
        {
            //pros::delay(100);
            stopIntake();
            return;
        }
        pros::delay(20);
        time += 20;
    }
}

void saveRing(int timeout){
    int time = 0;
    while (true) {
        if (time >= timeout)
        {
            return;
        }
        int hue = get2ndIntakeColor();
        if(detectOurColor(hue))
        {
            pros::delay(100);
            stopIntake();
            return;
        }
        if(detectTheirColor(hue))
        {
            pros::delay(100);
            stopIntake();
            return;
        }
        pros::delay(20);
        time += 20;
    }
}

void saveOurRing1(int timeout){
    int time = 0;
    while (true) {
        if (time >= timeout)
        {
            return;
        }
        int hue = getIntakeColor();
        //printf("Color %d\n",hue);
        if(detectOurColor(hue))
        {
            pros::delay(100);
            stopIntake();
            return;
        }
        pros::delay(20);
        time += 20;
    }
}

void saveRing1(int timeout){
    int time = 0;
    while (true) {
        if (time >= timeout)
        {
            return;
        }
        int hue = getIntakeColor();
        if(detectOurColor(hue))
        {
            //pros::delay(100);
            stopIntake();
            return;
        }
        if(detectTheirColor(hue))
        {
            //pros::delay(100);
            stopIntake();
            return;
        }
        pros::delay(20);
        time += 20;
    }
}
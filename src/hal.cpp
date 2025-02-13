#include "hal.h"
#include "auton_basics.h"
#include "fmt/format.h"
#include "pros/device.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/vision.h"
#include "robot_config.h"
#include "controls.h"
#include "main.h"
#include <cstdio>
#include <queue>


bool COLOR = true; // true = red, false = blue

bool auton = false, autoSkill = false;
bool autoDrive = false, autoLift = false, autoIntake = false;

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
    intakeL.move(speed);
    intakeR.move(speed);
}

void stopIntake() {
    setIntakeBrake(pros::E_MOTOR_BRAKE_COAST);
    intakeL.brake();
    intakeR.brake();
}

void stopIntakeHold() {
    setIntakeBrake(pros::E_MOTOR_BRAKE_HOLD);
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
    mogo_clamp.retract();
}

void closeClamp() {
    mogo_clamp.extend();
}

void toggleClamp() {
    mogo_clamp.toggle();
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

bool getRedirect() {
    return redirect1.is_extended();
}

// Hood

void toggleHood(){
    if(getHood()){
        hoodBwd();
    } else {
        hoodFwd();
    }
}

void hoodFwd() {
    hood1.extend();
    stopSorting();
}

void hoodBwd() {
    hood1.retract();
}

bool getHood(){
    return hood1.is_extended();
}

// Intake Lift
void liftIntake() {
    intake_lift.extend();
}

void dropIntake() {
    intake_lift.retract();
}

// Sweeper
void extendSweep() {
    mogo_rush.extend();
}

void retractSweep() {
    mogo_rush.retract();
}

void extendRushClamp()
{
    mogo_rush_clamp.extend();
}
void retractRushClamp()
{
    mogo_rush_clamp.retract();
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

//Lift Helper
void liftPneumaticUp() {
    lift_helper1.extend();
    lift_helper2.extend();
}

void liftPneumaticDown() {
    lift_helper1.retract();
    lift_helper2.retract();
}

bool getLiftPneumatic() {
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
int getFrontDistance() {
    return distance_front.get();
}

int getBackDistance() {
    return distance_back.get();
}

int getLeftDistance() {
    return distance_left.get();
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
bool getLimitSwitch() {
    return limitSwitch.get_value();
}

// Vision Sensor
pros::vision_object_s_t getOurColorObject() {
    return vision_sensor.get_by_sig(0, COLOR_SIG);
}

pros::vision_object_s_t getMostRelevantObject() {
    pros::vision_object_s_t object_arr[5];
    int availableObjects = vision_sensor.read_by_sig(0, (COLOR) ? 2 : 1, 5, object_arr);

    int highestY = 0;
    int highestYIndex = 0;

    for(int i = 0; i < 5; i++){
        if(object_arr[i].signature != VISION_OBJECT_ERR_SIG){
            if(abs(object_arr->x_middle_coord - 158) > 120){
                object_arr[i].signature = VISION_OBJECT_ERR_SIG;
                availableObjects--;
            } else if (object_arr[i].y_middle_coord > highestY){
                highestY = object_arr[i].y_middle_coord;
                highestYIndex = i;
            }
        }
    }

    if(availableObjects <= 1){
        //printf("X: %d", object_arr[highestYIndex].x_middle_coord);
        return object_arr[highestYIndex];
    }

    int lowestXOffset = 158;
    int lowestXOffsetIndex = 0;

    for(int i = 0; i < 5; i++){
        if(object_arr[i].signature != VISION_OBJECT_ERR_SIG){
            if(abs(object_arr[i].y_middle_coord - highestY) < 10) {
                if(abs(object_arr[i].x_middle_coord - 158) < lowestXOffset){
                    lowestXOffset = abs(object_arr[i].x_middle_coord - 158);
                    lowestXOffsetIndex = i;
                }
            }
        }
    }

    //printf("X: %d", object_arr[lowestXOffsetIndex].x_middle_coord);
    return object_arr[lowestXOffsetIndex];
}

// Motor Encoder
void setDriveEncoder(pros::motor_encoder_units_e mode){
    lf.set_encoder_units(mode);
    lm.set_encoder_units(mode);
    lb.set_encoder_units(mode);
    rf.set_encoder_units(mode);
    rm.set_encoder_units(mode);
    rb.set_encoder_units(mode);
}

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
    return getLFPosition();
}

float getRightMotorPosition() {
    return getRFPosition();
}

float getLeftMotorPositionInInches() {
    return wheelRotToInches(getLeftMotorPosition());
}

float getRightMotorPositionInInches() {
    return wheelRotToInches(getLeftMotorPosition());
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
    return (PI * DRIVEBASE_WHEEL_DIAMETER) * (degrees / 360.0);
}

float wheelRotToInches(float rotations){
    return (PI * DRIVEBASE_WHEEL_DIAMETER) * rotations;
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
    moveLiftToPos(82);
}

void liftDown() {
    moveLiftToPos(14);
}

void moveLiftToPos(float pos,int timeout){
    int time = pros::millis();
    autoLift = true;

    if(pos <= 13){
        pos = 13.5;
    }
    else if(pos >= 102){
        pos = 102;
    }
    autoLift = true;

    if(getLiftPosition() > pos){
        spinIntake(-127);

        while(getLiftPosition() >= pos && (pros::millis() - time) < timeout && autoLift)
        {
            pros::delay(20);
            //printf("Lift %f\n",getLiftPosition());
        }

        if(getLiftPosition() > 25 && getLiftPosition() < 300){
            liftPneumaticDown();
            stopIntakeHold();
        }
        else{
            stopIntake();
        }
    }
    else if(getLiftPosition() < pos){
        spinIntake(127);

        if(getLiftPosition() < 25){
            liftPneumaticUp();
        }

        while(getLiftPosition() <= pos && (pros::millis() - time) < timeout && autoLift)
        {
            pros::delay(20);
            //printf("Lift %f\n",getLiftPosition());
        }
        liftPneumaticDown();
        stopIntakeHold();
    }

    autoLift = false;
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

int redLower = 0;
int redUpper = 25;

int blueLower = 190;
int blueUpper = 230;

bool detectRed(int hue){
    return hue >= 0 && hue <= 25;
}

bool detectBlue(int hue){
    return hue >= 195 && hue <= 240;
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
            }
            else if(detectBlue(hue)) //If we detect blue
            {
                printf("Enter blue %i\n", hue);
                ringQueue.push(false); //Add to queue as upcoming

                while(detectBlue(hue)){ //Wait for ring to continue through intake
                    pros::delay(20);
                    hue = getIntakeColor();
                }
            }
        }
        pros::delay(10);
    }
}

void checkQueue() {
    while(true){
        if(autoIntake){
            if(!ringQueue.empty()){
                if(ringQueue.front() == COLOR && getRedirect()){ //If next ring is ours and redirect is up
                    closeRedirect(); //close redirect
                    printf("Scoring rings\n");
                } else if(ringQueue.front() != COLOR && !getRedirect()) { //If next ring is opp.'s and redirect is closed
                    redirectRings(); //open redirect
                    printf("Sorting out\n");
                }
            }
        }
        pros::delay(5);
    }
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

void sort_color_queue(){
    printf("Sorting started\n");
    pros::Task add_ring_task(addCurrentRing);
    pros::Task count_ring_task(countRings);
    pros::Task check_queue_task(checkQueue);
}

void startSorting() {
    if(!autoIntake && !getHood()){
        master.print(1,0,"Sorting");
        autoIntake = true;
    }
}

void stopSorting() {
    if(autoIntake){
        master.clear_line(1);
        closeRedirect();
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

void turnToRing(int timeout, float maxSpeed){
    bool reached = false;
    int counter = 0;

    while(!reached && timeout > 0){

        pros::vision_object_s_t nearestRing = getMostRelevantObject();

        if(nearestRing.signature != VISION_OBJECT_ERR_SIG){

            int error = nearestRing.x_middle_coord - 158;

            float motorPower = VISION_KP * error;

            if(motorPower > maxSpeed){
                motorPower = maxSpeed;
            } else if (motorPower < -maxSpeed){
                motorPower = -maxSpeed;
            }

            drive(motorPower, -motorPower);

            if(abs(error) < VISION_RANGE) {
                if(counter >= VISION_RANGE_TIMEOUT) {
                    reached = true;
                } else {
                    counter += 30;
                }
            } else {
                counter = 0;
            }
        }

        pros::delay(30);
        timeout -= 30;
    }

    stopDrive();
}

void driveToRing(int timeout, int maxSpeed) {
    int hueLower = (COLOR) ? redLower : blueLower, hueUpper = (COLOR) ? redUpper : blueUpper;
    autoDrive = true;
    float turnPower;
    pros::delay(50);
    while((getIntakeColor() < hueLower || getIntakeColor() > hueUpper) && timeout > 0 && (auton || autoSkill || autoDrive)) {

        pros::vision_object_s_t nearestRing = getMostRelevantObject();

        if(nearestRing.signature != VISION_OBJECT_ERR_SIG){

            int vision_error = nearestRing.x_middle_coord - 158;

            turnPower = VISION_KP * vision_error;

            printf("Turn: %f\n", turnPower);
        } 

        drive(maxSpeed + turnPower, maxSpeed - turnPower);
        //printf("Motor Power: %f\n", motorPower);

        pros::delay(15);
        timeout -= 15;
    }

    stopDrive();
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

            int vision_error = nearestRing.x_middle_coord - 158;

            turnPower = VISION_KP * vision_error;

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
  
void liftIntakeWallStake()
{
    while (true) {
        int hue = getIntakeColor();
        if(detectOurColor(hue) && getLiftPosition() > 50)
        {
            //pros::delay(100);
            liftIntake();
        }
        else if(detectTheirColor(hue) && getLiftPosition() > 50)
        {
            //pros::delay(100);
            liftIntake();
        }
        else
        {
            dropIntake();

        }
    }
     
}
void openRedirectAfterOurRing(int timeout)
{
    int time = 0;
    while (true) {
        int hue = get2ndIntakeColor();
        if(detectOurColor(hue))
        {
            pros::delay(250);
            redirectRings();
            return;
        }
        if(timeout < time)
        {
            return;
        }
        time += 20;
        pros::delay(20);
    }
     
}


void climb_piston_extend()
{
    climb.extend();
}
void climb_piston_retract()
{
    climb.retract();
}
void climb_piston_toggle()
{
    climb.toggle();
}
void climb_up()
{
    climb.extend();
    pros::delay(200);
    liftDown(); 
}

/*
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
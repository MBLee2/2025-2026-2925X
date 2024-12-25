#include "hal.h"
#include "fmt/format.h"
#include "pros/motors.h"
#include "pros/vision.h"
#include "robot_config.h"
#include "controls.h"
#include "main.h"
#include <cstdio>
#include <queue>


bool COLOR = false; // true = red, false = blue
int COLOR_SIG = (COLOR) ? 2 : 1;

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
    hood1.toggle();
    hood2.toggle();
}

void hoodFwd() {
    hood1.retract();
    hood2.retract();
}

void hoodBwd() {
    hood1.extend();
    hood2.extend();
}

bool getHood(){
    return hood1.is_extended();
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

int getLeftDistance() {
    return distance_left.get();
}

float distToWallF() {
    return getFrontDistance() / 25.4 + F_DISTANCE_OFFSET;
}

float distToWallL() {
    return getLeftDistance() / 25.4 + L_DISTANCE_OFFSET;
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
    int availableObjects = vision_sensor.read_by_sig(0, 2, 5, object_arr);

    int highestY = 0;
    int highestYIndex = 0;

    for(int i = 0; i < 5; i++){
        if(object_arr[i].signature != VISION_OBJECT_ERR_SIG){
            if(abs(object_arr->x_middle_coord - 158) > 90){
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

    int lowestXOffset = 158;
    int lowestXOffsetIndex = 0;

    for(int i = 0; i < 5; i++){
        if(object_arr[i].signature != VISION_OBJECT_ERR_SIG){
            if(abs(object_arr[i].y_middle_coord - highestY) < 15) {
                if(abs(object_arr[i].x_middle_coord - 158) < lowestXOffset){
                    lowestXOffset = abs(object_arr[i].x_middle_coord - 158);
                    lowestXOffsetIndex = i;
                }
            }
        }
    }

    return object_arr[lowestXOffsetIndex];
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
void driveDistance(float distance, int timeout, int maxSpeed) {
    resetDriveMotorPosition(); //TEST TO MAKE SURE THIS DOES NOT AFFECT LEMLIB
    autoDrive = true;
    bool notReached = true;
    float error, prevError = 0, totalError = 0;
    float derivative;
    int counter = 0;
    pros::delay(50);
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

        if(motorPower > maxSpeed){
            motorPower = maxSpeed;
        } 
        else if(motorPower < -maxSpeed){
            motorPower = -maxSpeed;
        }

        driveStraight(motorPower);
        //printf("Motor Power: %f\n", motorPower);

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
    moveLiftToPos(80);
}

void liftDown() {
    moveLiftToPos(8);
}

void moveLiftToPos(float pos,int timeout){
    int time = pros::millis();

    if(pos <= 2){
        pos = 3;
    }
    else if(pos >= 84){
        pos = 83;
    }
    autoLift = true;

    if(getLiftPosition() > pos){
        spinIntake(-127);

        while(getLiftPosition() >= pos && (pros::millis() - time) < timeout && autoLift)
        {
            pros::delay(20);
            //printf("Lift %f\n",getLiftPosition());
        }

        if(getLiftPosition() > 15 && getLiftPosition() < 300){
            liftPneumaticDown();
            stopIntakeHold();
        }
        else{
            stopIntake();
        }
    }
    else if(getLiftPosition() < pos){
        spinIntake(127);

        if(getLiftPosition() < 15){
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
int redUpper = 35;

int blueLower = 140;
int blueUpper = 260;

bool detectRed(int hue){
    return hue >= 0 && hue <= 35;
}

bool detectBlue(int hue){
    return hue >= 140 && hue <= 260;
}

bool detectOurColor(int hue){
    if(COLOR){
        return detectRed(hue);
    } else {
        return detectBlue(hue);
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

            if(detectRed(hue))
            {
                printf("Detected red %i\n", hue);
                ringQueue.push(true);
                
                while(detectRed(hue)){
                    pros::delay(20);
                    hue = getIntakeColor();
                }
            }
            else if(detectBlue(hue))
            {
                printf("Detected blue %i\n", hue);
                ringQueue.push(false);

                while(detectBlue(hue)){
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
                if(ringQueue.front() == COLOR && getRedirect()){
                    closeRedirect();
                    printf("Scoring rings\n");
                } else if(ringQueue.front() != COLOR && !getRedirect()) {
                    redirectRings();
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

            if(detectRed(hue) && ringQueue.front() == true)
            {
                if(COLOR == true){
                    waitForExitRed();
                    ringQueue.pop();
                    printf("Exiting red %i\n", hue);
                } else {
                    pros::delay(30);
                    ringQueue.pop();
                    printf("Exiting red %i\n", hue);
                    waitForExitRed();
                }
            }

            if(detectBlue(hue) && ringQueue.front() == false)
            {
                if(COLOR == false){
                    waitForExitBlue();
                    ringQueue.pop();
                    printf("Exiting blue %i\n", hue);
                } else {
                    pros::delay(30);
                    ringQueue.pop();
                    printf("Exiting blue %i\n", hue);
                    waitForExitBlue();
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
    if(!autoIntake){
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
        } 

        drive(maxSpeed + turnPower, maxSpeed - turnPower);
        //printf("Motor Power: %f\n", motorPower);

        pros::delay(15);
        timeout -= 15;
    }

    stopDrive();
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
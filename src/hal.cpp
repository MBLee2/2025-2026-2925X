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
#include <climits>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>
#include <type_traits>


bool COLOR = false; // true = red, false = blue
int COLOR_SIG = (COLOR) ? 1 : 2;


bool auton = false, autoSkill = false;
bool autoDrive = false, autoLift = false, autoIntake = false, isintaking = false, LBPickup = false;
bool liftReset = false; //Do not modify

std::queue<bool> ringQueue;


//Basic Motor Movement

void stopAllMotors() {
    stopDrive();
    stopAllIntake();
}

// Drive Base Movement
void spinLeftMotors(int speed) {
    left_side_motors.move(speed);
}

void spinRightMotors(int speed) {
    right_side_motors.move(speed);
}

void stopLeftMotors() {
    left_side_motors.brake();
}

void stopRightMotors() {
    right_side_motors.brake();
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
    left_side_motors.set_brake_mode_all(mode);
    right_side_motors.set_brake_mode_all(mode);
}


// Intake Movement
intake_state current_intake = STOP;
reload_state current_reload = FROM_INTAKE;

void spinIntake(int speed) {
    intake.move(speed);
    intake2.move(speed);
}

void spinScoring(int speed) {
    /*if(hoodExtended()){
        setScoringBrake(pros::E_MOTOR_BRAKE_COAST);
        scoringR.move(speed);
        scoringL.move(speed);
    }*/
}

void spinStorage(int speed) {
    //storage.move(speed);
}

void spinReload(int speed) {
    //reload.move(speed);
}

void stopIntake() {
    setIntakeBrake(pros::E_MOTOR_BRAKE_COAST);
    intake.brake();
    intake2.brake();
}

void stopIntakeHold() {
    setIntakeBrake(pros::E_MOTOR_BRAKE_HOLD);
    intake.brake();
    intake2.brake();
}

void stopScoring() {
    /*setScoringBrake(pros::E_MOTOR_BRAKE_COAST);
    scoringR.brake();
    scoringL.brake();*/
}

void stopScoringHold() {
    /*setScoringBrake(pros::E_MOTOR_BRAKE_HOLD);
    scoringR.brake();
    scoringL.brake();*/
}

void stopStorage() {
    //storage.brake();
}

void stopReload() {
    //reload.brake();
}

void intakeAll(int speed) {
    printf("INTAKE\n");
    current_intake = INTAKE;
    current_reload = FROM_INTAKE;

    spinIntake(speed);

    master.clear();
    master.print(1, 0, "Intake");
}

void scoreTop(int speed) {
    goalUp();
    intakeAll(speed);

    /*if(current_reload == FROM_INTAKE){
        topFromIntake(speed);
    } else if(current_reload == FROM_STORAGE){
        topFromStorage(speed);
    }*/
}

void topFromIntake(int speed){
    /*printf("TOPSCORE\n");
    current_intake = TOPSCORE;
    current_reload = FROM_INTAKE;
    
    spinIntake(speed);
    spinStorage(speed);
    spinReload(speed);

    hoodUp();
    spinScoring(speed);

    master.clear();
    master.print(1, 0, "Top Score");
    master.print(2, 0, "From Intake");*/

}

void topFromStorage(int speed){
    /*printf("TOPSCORE\n");
    current_intake = TOPSCORE;
    current_reload = FROM_STORAGE;
    
    spinIntake(speed);
    spinStorage(speed);
    spinReload(-speed);

    hoodUp();
    spinScoring(speed);

    master.clear();
    master.print(1, 0, "Top Score");
    master.print(2, 0, "From Storage");*/

}

void scoreMiddle(int speed){
    goalDown();
    intakeAll(speed);

    /*if(current_reload == FROM_INTAKE){
        middleFromIntake(speed);
    } else if(current_reload == FROM_STORAGE){
        middleFromStorage(speed);
    }*/
}

void middleFromIntake(int speed){
    /*printf("MIDSCORE\n");
    current_intake = MIDSCORE;
    current_reload = FROM_INTAKE;

    spinIntake(0.5 * speed);
    spinStorage(speed);
    spinReload(speed);

    hoodUp();
    spinScoring(-0.3 * speed);

    master.clear();
    master.print(1, 0, "Middle Score");
    master.print(2, 0, "From Storage");*/

}

void middleFromStorage(int speed){
    /*printf("MIDSCORE\n");
    current_intake = MIDSCORE;
    current_reload = FROM_STORAGE;

    spinIntake(0.5 * speed);
    spinStorage(speed);
    spinReload(-speed);

    hoodUp();
    spinScoring(-0.3 * speed);

    master.clear();
    master.print(1, 0, "Middle Score");
    master.print(2, 0, "From Storage");*/
}

void outakeAll(int speed){
    printf("OUTAKE\n");
    current_intake = OUTAKE;
    current_reload = FROM_INTAKE;

    spinIntake(-speed);

    master.clear();
    master.print(1, 0, "Outake");
}


void stopAllIntake(){
    stopIntake();

    master.clear();
    master.print(1, 0, "Stopped");

    current_intake = STOP;
    current_reload = FROM_INTAKE;
}

void setIntakeBrake(pros::motor_brake_mode_e mode) {
    intake.set_brake_mode(mode);
    intake2.set_brake_mode(mode);
    //intakeR.set_brake_mode(mode);
}

void setScoringBrake(pros::motor_brake_mode_e mode) {
    //scoringR.set_brake_mode(mode);
}

void intakeAntiJam() {
    if(fabs(intake.get_actual_velocity()) < 5)
    {
        if(current_intake == INTAKE){
            spinIntake(-127);
            pros::delay(200);
            intakeAll(127);
        } else if(current_intake == OUTAKE){
            outakeAll(127);
            pros::delay(200);
            intakeAll(-127);
        } 
        
        /*else if (temp_intake == TOPSCORE) {
            reload_state temp_reload = current_reload;
            if(temp_reload == FROM_INTAKE){
                topFromIntake(127);
            } else if(temp_reload == FROM_STORAGE){
                topFromStorage(127);
            }
        } else if (temp_intake == MIDSCORE) {
            reload_state temp_reload = current_reload;
            if(temp_reload == FROM_INTAKE){
                middleFromIntake(127);
            } else if(temp_reload == FROM_STORAGE){
                middleFromStorage(127);
            }
        }*/
    }
}

bool antiJam = false;
void intakeAntiJamTaskFunc(){
    int counter = 0;
    while (true) {
        if(intake.get_actual_velocity() < 5 && antiJam)
        {
            counter++;
            if(counter >= 15)
            {
                intakeAntiJam();
                counter = 0;
            }
        } else if (counter > 0) {
            counter = 0;
        }
        pros::delay(20);
    }
}

void startAntiJam() {
    antiJam = true;
}

void stopAntiJam() {
    antiJam = false;
}

//Pneumatics
void extendLoader(){
    loader.extend();
}

void retractLoader(){
    loader.retract();
}

void toggleLoader(){
    if(loaderExtended()){
        retractLoader();
        
    } else {
        extendLoader();
    }
}

bool loaderExtended(){
    return loader.is_extended();
}

void hoodUp(){
    /*hood.extend();
    if(current_intake == OUTAKE){
        printf("OUTAKE\n");
        pros::delay(250);
        spinScoring(-127);
    } else if(current_intake == MIDSCORE){
        printf("MIDSCORE\n");
        pros::delay(250);
        spinScoring(-0.3 * 127);
    } else if(current_intake == TOPSCORE){
        printf("TOPSCORE\n");
        pros::delay(250);
        spinScoring(127);
    }*/
}

void hoodDown(){
    // stopScoring();
    // pros::delay(200);
    // hood.retract();
}

void toggleHood(){
    if(hoodExtended()){
        hoodDown();
    } else {
        hoodUp();
    }
}

bool hoodExtended(){
    //return hood.is_extended();
}

void goalDown(){
    goal_switch.retract();
    current_intake = MIDSCORE;
}

void goalUp(){
    goal_switch.extend();
    current_intake = TOPSCORE;
}

bool goalExtended(){
    return goal_switch.is_extended();
}

void toggleGoal(){
    if(goalExtended()){
        goalDown();
    } else {
        goalUp();
    }
}

void extendAligner(){
    aligner.extend();
}

void retractAligner(){
    aligner.retract();
}

void toggleAligner(){
    if(alignerExtended()){
        retractAligner();
    } else {
        extendAligner();
    }
}

bool alignerExtended(){
    return aligner.is_extended();
}

//IMU
void resetIMUHeading() {
    imu.tare_heading();
}

float getHeading() {
    return imu.get_heading();
}


// Motor Encoder
void setDriveEncoder(pros::motor_encoder_units_e_t mode){
    left_side_motors.set_encoder_units_all(mode);
    right_side_motors.set_encoder_units_all(mode);
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

// Reset Motor Positions
void resetLeftMotorPosition() {
    left_side_motors.tare_position_all();
}

void resetRightMotorPosition() {
    right_side_motors.tare_position_all();
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

//Sensors
int getIntakeColor() {
    return intake_color.get_hue();
}

int getIntakeColorDist() {
    return intake_color.get_proximity();
}

bool detectRed(int hue){
    return hue >= 0 && hue <= 35;
}
bool detectBlue(int hue){
    return hue >= 180 && hue <= 200;
}

bool detectBlock(int hue){
    return detectRed(hue) || detectBlue(hue);
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

void count_blocks(int num, int timeout){
    while(num > 0 && timeout > 0){
        if(detectBlock(getIntakeColor())){
            while(detectBlock(getIntakeColor())){
                pros::delay(20);
                timeout -= 20;
            }
            num--;
        }
        pros::delay(20);
        timeout -= 20;
    }
}

int num_blocks = 0;
void taskFn_count_blocks() {
	printf("%s(): Started\n", __func__);
    while(true){
        if(current_intake == INTAKE){
            if(detectBlock(getIntakeColor())){
                while(detectBlock(getIntakeColor())){
                    pros::delay(20);
                }
                num_blocks++;
                printf("Num blocks: %d\n", num_blocks);
                if(num_blocks > 4 && current_intake == INTAKE){
                    intakeAll(127);
                }
            }
        }
        else if(current_reload == FROM_STORAGE || current_intake == OUTAKE){
            if(detectBlock(getIntakeColor())){
                while(detectBlock(getIntakeColor())){
                    pros::delay(20);
                }
                num_blocks--;
                printf("Num blocks: %d\n", num_blocks);
            }
        }

        if(num_blocks < 0){
            num_blocks = 0;
        }
        pros::delay(20);
    }
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

int start_time;
void log_data(char* message){
    lemlib::Pose temp_pose = chassis.getPose();
    printf("%s, %d, %f, %f, %f", message, pros::millis() - start_time, temp_pose.x, temp_pose.y, temp_pose.theta);
    printf(";\n");
}

char* step_message;
void logStep(){
    if(strcmp(step_message, "") != 0){
        log_data(step_message);
        step_message = "";
    }
}

void messageStep(char* message){
    step_message = message;
}

bool prev_motion = false;
void logMove(){
    if(chassis.isInMotion() && !prev_motion){
        log_data((char*) "Move started");
        prev_motion = true;
    } else if(!chassis.isInMotion() && prev_motion){
        log_data((char*) "Move ended");
        prev_motion = false;
    }
}

const int tick_length = 20;
void logTick(){
    if(floor(((pros::millis() - start_time) % tick_length) / 10) == 0){
        log_data((char *) "N/A\t");
    }
}

bool logging_data = false;
void (*function_pointers[])() = {&logStep, &logMove, &logTick};
void logging(int range){
    printf("Message, Time, X, Y, Theta");
    printf(";\n");
    pros::Task log_task([=]{
        logging_data = true;
        start_time = pros::millis();
        while(logging_data){
            for(int i = 0; i < range; i++){
                function_pointers[i]();
            }
        }
    });
}

void end_log(){
    logging_data = false;
}

/*********************** END OF USED FUNCTIONS ***********************/


// Lift Movement
void spinLift(int speed) {
    //ladybrown.move(speed);
    // setLiftBrake((getLiftPosition() > 40) ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
}

void stopLift(){
    //ladybrown.brake();
}

void stopLiftCoast(){
    // setLiftBrake(pros::E_MOTOR_BRAKE_COAST);
    // stopLift();
}
void stopLiftHold() {
    // setLiftBrake(pros::E_MOTOR_BRAKE_HOLD);
    // stopLift();
}


void setLiftBrake(pros::motor_brake_mode_e mode) {
    //ladybrown.set_brake_mode_all(mode);
}


//Pneumatics

// Clamp
void openClamp() {
    // mogo_clamp.retract();
}

void closeClamp() {
    // mogo_clamp.extend();
}

void toggleClamp() {
    // mogo_clamp.toggle();
}

// leftsweeper
void extendLeftSweeper() {
    // left_sweeper.extend();
}

void retractLeftSweeper() {
    // left_sweeper.retract();
}

void toggleLeftSweeper() {
    // left_sweeper.toggle();
}

// right sweeper
void extendRightSweeper() {
    // right_sweeper.extend();
}

void retractRightSweeper() {
    // right_sweeper.retract();
}

void toggleRightSweeper() {
    // right_sweeper.toggle();
}
//IntakeLift
void liftIntake() {
    // intake_lift.extend();
}
void dropIntake()
{
    // intake_lift.retract();
}
//PTO
void closePTO() {
    // pto.extend();
}
void openPTO() {
    // pto.retract();
}
void togglePTO() {
    // pto.toggle();
}
//Climb balance
void retractClimbBalance() {
    // climb_balance.extend();
}



//Distance
int getFrontDistance() {
    // return distance_front.get();
    return 0;
}

int getBackDistance() {
    // return distance_back.get();
    return 0;
}

int getLeftDistance() {
    // return distance_left.get();
    return 0;
}

int getRightDistance() {
    // return distance_right.get();
    return 0;
}

int getProximity() {
    // return distance_proxi.get();
    return 0;
}

float distToWallF() {
    // return (getFrontDistance() / 25.4) + F_DISTANCE_OFFSET;
    return 0;
}

float distToWallB() {
    // return (getBackDistance() / 25.4) + B_DISTANCE_OFFSET;
    return 0;
}

float distToWallL() {
    // return (getLeftDistance() / 25.4) + L_DISTANCE_OFFSET;
    return 0;
}

float distToWallR() {
    // return (getRightDistance() / 25.4) + R_DISTANCE_OFSET;
    return 0;
}

float distToObject() {
    // return (getProximity() / 25.4) + PROXI_OFFSET;
    return 0;
}

int getIntakeDist(){
    // return intake_dist.get();
    return 0;
}

//Color


int get2ndIntakeColor() {
    // return intake_color2.get_hue();
    return 0;
}

void setIntakeColorLED(int value){
    // intake_color.set_led_pwm(value);
}

void setIntakeColor2LED(int value){
    // intake_color2.set_led_pwm(value);
}

// Limit Switch
bool getLBLimitSwitch() {
    // return LB_limit.get_value();
    return 0;
}

// Vision Sensor
pros::vision_object_s_t getRed() {
    // return vision_sensor.get_by_sig(0, 1);
}
pros::vision_object_s_t getBlue() {
    // return vision_sensor.get_by_sig(0, 2);
}

pros::vision_object_s_t getOurColorObject() {
    // return vision_sensor.get_by_sig(0, COLOR_SIG);
}

void swap_objects(pros::vision_object_s_t obj_arr[], int a, int b){
    // pros::vision_object_s_t temp = obj_arr[a];
    // obj_arr[a] = obj_arr[b];
    // obj_arr[b] = temp;
}

pros::vision_object_s_t getMostRelevantObject(bool color) {
    // pros::vision_object_s_t object_arr[5];
    // int availableObjects = vision_sensor.read_by_sig(0, (color) ? 1 : 2, 5, object_arr);

    // int highestY = 0;
    // int highestYIndex = 0;

    // for(int i = 0; i < 5; i++){
    //     if(object_arr[i].signature != VISION_OBJECT_ERR_SIG){
    //         if(abs(object_arr->x_middle_coord - VISION_CENTER) > 135){
    //             object_arr[i].signature = VISION_OBJECT_ERR_SIG;
    //             availableObjects--;
    //         } else if (object_arr[i].y_middle_coord > highestY){
    //             highestY = object_arr[i].y_middle_coord;
    //             highestYIndex = i;
    //         }
    //     }
    // }

    // if(availableObjects <= 1){
    //     return object_arr[highestYIndex];
    // }

    // int lowestXOffset = 160;
    // int lowestXOffsetIndex = 0;

    // for(int i = 0; i < 5; i++){
    //     if(object_arr[i].signature != VISION_OBJECT_ERR_SIG){
    //         if(abs(object_arr[i].y_middle_coord - highestY) < 10) {
    //             if(abs(object_arr[i].x_middle_coord - VISION_CENTER) < lowestXOffset){
    //                 lowestXOffset = abs(object_arr[i].x_middle_coord - VISION_CENTER);
    //                 lowestXOffsetIndex = i;
    //             }
    //         }
    //     }
    // }

    // return object_arr[lowestXOffsetIndex];
}


void setLiftZero(double pos){
    //ladybrownL.set_zero_position(pos);
    //ladybrownR.set_zero_position(pos);
}

void resetLiftPosition(){
    //ladybrownL.tare_position();
    //ladybrownR.tare_position();
}
void resetLiftPositionWithDistance(){
    // if(getLBLimitSwitch())
    // { 
    //     LBPickup = false;
    //     liftReset = true;
    //     resetLiftPosition();
    //     //printf("Lift reset\n");
    // }
}
void resetLiftWithDistTaskFunc(){
    // while (true) {
    //     resetLiftPositionWithDistance();
    //     pros::delay(20);
    // }
}

void setLiftEncoder(pros::motor_encoder_units_e mode) {
    //ladybrownL.set_encoder_units(mode);
    //ladybrownR.set_encoder_units(mode);
}


float getLiftPosition() {
    //float pos = (ladybrownL.get_position() + R.get_position()) / 2 ;
    // float pos = 0;
    // return (pos);
    return 0;
}

void setIntakeEncoder(pros::motor_encoder_units_e mode) {
    // intake.set_encoder_units(mode);
}

float getIntakePosition() {
    // return (intake.get_position());
    return 0;
}


int getRightLine() {
    // return lineRight.get_value(); 
    return 0;
}

int getLeftLine() {
    // return lineLeft.get_value();
    return 0;
}

bool detectLine(int value) {
    // return value < 2700;
    return 0;
}

bool detectRightLine() {
    // return detectLine(getRightLine());
    return 0;
}

bool detectLeftLine() {
    // return detectLine(getLeftLine());
    return 0;
}




// Lift
void liftUpWallStake() {
    // setLiftEncoder(pros::E_MOTOR_ENCODER_DEGREES);
    // moveLiftToPos(210,127,1200);
    // stopLiftHold();
    // printf("WALL STAKE");
}

int moveToReset(float speed) {
    // autoLift = false;
    // pros::delay(30);
    // int time = 0;
    // LBPickup = true;
    // autoLift = true;
    // setLiftEncoder(pros::E_MOTOR_ENCODER_DEGREES);
    // while (!getLBLimitSwitch() && time < 1200 && autoLift) {
    //     spinLift(-speed);
    //     pros::delay(20);
    //     time=+20;
    // }
    // resetLiftPositionWithDistance();
    // stopLift();
    // return time;
    return 0;
}

void liftPickup() {
    // int time = 0;
    // if(getLiftPosition() < 70 || !liftReset){
    //     time = moveToReset(100);
    //     moveLiftToPos(60, 100, 1200 - time);
    // } else {
    //     moveLiftToPos(60, 100, 1200);
    // }
    // stopLiftHold();
    // LBPickup = true;
}

void liftDown() {
    // moveLiftToPos(4);
}

void moveLiftToPos(float pos,int speed,int timeout){

    // autoLift = false;
    // pros::delay(30);

    // if(pos <= 0){
    //     pos = 0;
    // }
    // else if(pos >= 2000){
    //     pos = 2000;
    // }
    // autoLift = true;
    // float error, prevError, derivative;
    // int counter = 0;
    // while(counter < 150 && timeout > 0 && autoLift){
    //     error = pos - getLiftPosition();
    //     printf("Lift: %f\tError: %f\n", getLiftPosition(), error);

    //     derivative = error - prevError;

    //     float motorPower = 1.2 * error + derivative;
     
    //     if(motorPower > speed) motorPower = speed;
    //     else if(motorPower < -speed) motorPower = -speed;
    //     spinLift(motorPower);

    //     prevError = error;
    //     pros::delay(20);
    //     timeout -= 20;

    //     if(fabs(pos - getLiftPosition()) < 0.5){
    //         counter += 20;
    //     } else {
    //         counter = 0;
    //     }
    // }
    // stopLift();
    // autoLift = false;
}






// Color Sorting

bool sort_color(bool sort) {
    //false = red, true = blue 
    // int hue = getIntakeColor();
    // int hue2 = get2ndIntakeColor();
    // int timeout = 0;
    // if (sort == true){

    //     setIntakeColorLED(100);
    //     setIntakeColor2LED(100);

    //     if(COLOR == false) // Sorting out Red
    //     {
    //         //printf("Sorting out red \n"); // Log the function exit for debugging 
    //         if(hue >= 0 && hue <= 25)
    //         {
    //             printf("Detected Red \n"); // Log the function exit for debugging
    //             while(hue2 >= 155 && hue2 <= 185){
    //                 pros::delay(20);
    //                 hue2 = get2ndIntakeColor();
    //             }

    //             redirectRings();

    //             while((hue2 < 0 || hue2 > 25) && timeout < 500){
    //                 pros::delay(20);
    //                 hue2 = get2ndIntakeColor();
    //                 timeout += 20;
    //             }

    //             timeout = 0;
    //             while((hue2 >= 0 && hue2 <= 25) && timeout < 500){
    //                 pros::delay(20);
    //                 hue2 = get2ndIntakeColor();
    //                 timeout += 20;
    //             }

    //             pros::delay(50);
    //             closeRedirect();
    //             return true;
    //         }
    //     }
    //     if(COLOR == true)// Sorting out Blue
    //     {
    //         if(hue >= 135 && hue <= 185)
    //         {
    //             printf("Detected Blue \n"); // Log the function exit for debugging
    //             while(hue2 >= 0 && hue2 <= 25){
    //                 pros::delay(20);
    //                 hue2 = get2ndIntakeColor();
    //             } 

    //             redirectRings();

    //             while((hue2 < 155 || hue2 > 185) && timeout < 500){
    //                 pros::delay(20);
    //                 hue2 = get2ndIntakeColor();
    //                 timeout += 20;
    //             }

    //             timeout = 0;
    //             while((hue2 >= 155 && hue2 <= 185) && timeout < 500){
    //                 pros::delay(20);
    //                 hue2 = get2ndIntakeColor();
    //                 timeout += 20;
    //             }

    //             pros::delay(50);
    //             closeRedirect();
    //             return true;
    //         }
    //     }
    // }
    // return false;
}

bool detectRingFront(){
    // return distToObject() < 3.7;
    return 0;
}

int redLower = 0;
int redUpper = 20;

int blueLower = 210;
int blueUpper = 230;

bool detectRing() {
    // return getIntakeDist() < 30;
    return 0;
}

void printRingQueue(){
    // if(!ringQueue.empty()){
    //     bool first;
    //     for(int i = 0; i < ringQueue.size(); i++){
    //         first = ringQueue.front();
    //         if(first){
    //             printf("Red  ");
    //         } else {
    //             printf("Blue ");
    //         }
    //         ringQueue.pop();
    //         ringQueue.push(first);
    //     }
    //     printf("\n");
    // }
}


void clearRingQueue() {
    // while(!ringQueue.empty()){
    //     ringQueue.pop();
    // }
}

void addCurrentRing(){
    // while(true){
    //     if(autoIntake){
    //         int hue = getIntakeColor();

    //         if(detectRed(hue)) //If we detect red
    //         {
    //             printf("Enter red %i\n", hue);
    //             ringQueue.push(true); //Add to queue as upcoming
                
    //             while(detectRed(hue)){ //Wait for ring to continue through intake
    //                 pros::delay(20);
    //                 hue = getIntakeColor();
    //             }
    //             printRingQueue();
    //         }
    //         else if(detectBlue(hue)) //If we detect blue
    //         {
    //             printf("Enter blue %i\n", hue);
    //             ringQueue.push(false); //Add to queue as upcoming

    //             while(detectBlue(hue)){ //Wait for ring to continue through intake
    //                 pros::delay(20);
    //                 hue = getIntakeColor();
    //             }
    //             printRingQueue();
    //         }
    //     }
    //     pros::delay(10);
    // }
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
    // int hue = get2ndIntakeColor();

    // while(detectRed(hue)){
    //     pros::delay(20);
    //     hue = get2ndIntakeColor();
    // }
}

void waitForExitBlue(){
    // int hue = get2ndIntakeColor();

    // while(detectBlue(hue)){
    //     pros::delay(20);
    //     hue = get2ndIntakeColor();
    // }
}

void waitForExitRing() {
    // while(detectRing()){
    //     pros::delay(20);
    // }
    // if(ringQueue.front()){
    //     printf("Exiting Red \n");
    // } else {
    //     printf("Exiting Blue \n");
    // }
}

void countRings() {
    // while(true){
    //     if(autoIntake && !ringQueue.empty()){
    //         int hue = get2ndIntakeColor();

    //         if(detectRed(hue) && ringQueue.front() == true) //If we detect red
    //         {
    //             if(COLOR == true){ //If we are red 
    //                 waitForExitRed(); //wait for it to score to avoid opening early
    //                 ringQueue.pop(); //then remove from queue
    //                 printf("Exiting red %i\n", hue);
    //                 pros::delay(50);
    //             } else { //If we are not red
    //                 pros::delay(30);
    //                 ringQueue.pop(); //remove from queue as it's exiting to close as early as possible
    //                 printf("Exiting red %i\n", hue);
    //                 waitForExitRed(); //wait for it to exit
    //             }
    //         }

    //         if(detectBlue(hue) && ringQueue.front() == false) //If we detect blue
    //         {
    //             if(COLOR == false){ //If we are blue
    //                 waitForExitBlue(); //wait for it to score to avoid opening early
    //                 ringQueue.pop(); //then remove from queue
    //                 printf("Exiting blue %i\n", hue);
    //                 pros::delay(50);
    //             } else {
    //                 pros::delay(30);
    //                 ringQueue.pop(); //remove from queue as it's exiting to close as early as possible
    //                 printf("Exiting blue %i\n", hue);
    //                 waitForExitBlue(); //wait for it to exit
    //             }
    //         }
    //     }
    //     pros::delay(10);
    // }
}

void countRingsDist() {
    // while(true) {
    //     if(autoIntake && !ringQueue.empty()){
    //         if(detectRing())
    //         {
    //             printRingQueue();
    //             if(COLOR == ringQueue.front()){
    //                 waitForExitRing();
    //                 ringQueue.pop();
    //             } else {
    //                 printf("sorting out\n");
    //                 waitForExitRing();
    //                 //pros::delay(100);
    //                 spinIntake(-127);
    //                 ringQueue.pop();
    //                 pros::delay(70);
    //                 spinIntake(127);
    //             }
    //         }
    //     }
    //     pros::delay(20);
    // }
}

void sort_color_queue(){
    // printf("Sorting started\n");
    // pros::Task add_ring_task(addCurrentRing);
    // pros::Task count_ring_task(countRingsDist);
}

void startSorting() {
    // if(!autoIntake ){
    //     master.print(1,0,"Sorting");
    //     autoIntake = true;
    // }
}

void stopSorting() {
    // if(autoIntake){
    //     master.clear_line(1);
    //     autoIntake = false;
    //     clearRingQueue();
    // }
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
    // return (ring.x_middle_coord != 0 || ring.y_middle_coord != 0) && abs(ring.x_middle_coord) < 160 && abs(ring.y_middle_coord) < 110;
    return 0;
}

void turnToRing(int timeout, float maxSpeed, bool color){
//     bool reached = false;
//     int counter = 0;
//     int error;
//     float prevError = 0,derivative = 0;

//     while(!reached && timeout > 0){

//         pros::vision_object_s_t nearestRing = getMostRelevantObject(color);

//         if(checkRing(nearestRing)){

//             error = nearestRing.x_middle_coord - VISION_CENTER;

//             derivative = error - prevError;

//             float motorPower = (VISION_TURN_KP * error) + (VISION_TURN_KD * derivative);

//             if(motorPower > maxSpeed){
//                 motorPower = maxSpeed;
//             } else if (motorPower < -maxSpeed){
//                 motorPower = -maxSpeed;
//             }

//             drive(motorPower, -motorPower);
//             prevError = error;

//             printf("(%d, %d)\t Error: %d, motorPower: %f\n", nearestRing.x_middle_coord, nearestRing.y_middle_coord, error, motorPower);
            

//             if(abs(error) < VISION_RANGE) {
//                 counter += 20;
//                 if(counter >= VISION_RANGE_TIMEOUT) {
//                     reached = true;
//                 } 
//             } else {
//                 counter = 0;
//             }
//         }

//         pros::delay(20);
//         timeout -= 20;
//     }

//     stopDrive();
// }

// void turnToGoal(int timeout, float maxSpeed){
//     bool reached = false;
//     int counter = 0;
//     int error;
//     float prevError = 0,derivative = 0;

//     while(!reached && timeout > 0){

//         pros::vision_object_s_t nearestGoal = vision_sensor.get_by_sig(0, 3);

//         if(checkRing(nearestGoal)){

//             error = nearestGoal.x_middle_coord - VISION_CENTER;

//             derivative = error - prevError;

//             float motorPower = (VISION_TURN_KP * error) + (VISION_TURN_KD * derivative);

//             if(motorPower > maxSpeed){
//                 motorPower = maxSpeed;
//             } else if (motorPower < -maxSpeed){
//                 motorPower = -maxSpeed;
//             }

//             drive(motorPower, -motorPower);
//             prevError = error;

//             printf("(%d, %d)\t Error: %d, motorPower: %f\n", nearestGoal.x_middle_coord, nearestGoal.y_middle_coord, error, motorPower);
            

//             if(abs(error) < VISION_RANGE) {
//                 counter += 20;
//                 if(counter >= VISION_RANGE_TIMEOUT) {
//                     reached = true;
//                 } 
//             } else {
//                 counter = 0;
//             }
//         }

//         pros::delay(20);
//         timeout -= 20;
//     }

//     stopDrive();
}

void driveTowardsRing(int timeout, int maxSpeed, bool color){
    // int hueLower = (COLOR) ? redLower : blueLower, hueUpper = (COLOR) ? redUpper : blueUpper;
    // float motorPower;

    // pros::vision_object_s_t ring = getMostRelevantObject(color);
    // if(checkRing(ring) && ring.y_middle_coord < 100){
    //     while((getIntakeColor() < hueLower || getIntakeColor() > hueUpper) && timeout > 0){
    //         ring = getMostRelevantObject();

    //         if(checkRing(ring)){
    //             int error = 106 + ring.y_middle_coord;
    //             motorPower = VISION_LAT_KP * error;

    //             if(motorPower > maxSpeed){
    //                 motorPower = maxSpeed;
    //             }
    //         }

    //         driveStraight(motorPower);
    //         printf("(%d, %d)\n", ring.x_middle_coord, ring.y_middle_coord);

    //         pros::delay(30);
    //         timeout -= 30;

    //     }

    //     stopDrive();
    // }
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

    // autoDrive = true;
    // float motorPower = params.maxSpeed, turnPower, ringPower;
    // float distTravelled, drive_error = params.maxDist, derivative, prevError;
    // float leftSpeed, rightSpeed;
    // lemlib::Pose initialPose = chassis.getPose();
    // bool startLessX = initialPose.x < params.xLimit, startLessY = initialPose.y < params.yLimit;

    // printf("(%f, %f) - before loop\n", chassis.getPose().x, chassis.getPose().y);
    // while(timeout > 0 && (auton || autoSkill || autoDrive)) {

    //     pros::vision_object_s_t nearestRing = getMostRelevantObject(params.color);

    //     if(params.driveThrough){
    //         if(!params.keepDriving || !checkRing(nearestRing)){
    //             if(detectOurColor(getIntakeColor())) { 
    //                 printf("Stop by intake");
    //                 break; 
    //             }
    //         }
    //     } else if(detectRingFront()) { 
    //         printf("Stop by intake");
    //         break; 
    //     }

    //     lemlib::Pose currentPose = chassis.getPose();
    //     //float leftPos = getLeftMotorPositionInInches(), rightPos = getRightMotorPositionInInches();
    //     distTravelled = distBetweenPts(currentPose.x, currentPose.y, initialPose.x, initialPose.y);
    //     if(timeout % 20 == 0){
    //         printf("Total dist: %f\t", distTravelled);
    //     }

    //     if(distTravelled > params.maxDist || 
    //             startLessX != (currentPose.x < params.xLimit) || 
    //             startLessY != (currentPose.y < params.yLimit)) {
    //         printf("Stop by maximum\n");
    //         break;
    //     }
    //     drive_error = std::min({params.maxDist - distTravelled, 
    //         (float) fabs((1/sin(deg2rad(currentPose.theta))) * (params.xLimit - currentPose.x)), 
    //         (float) fabs((1/cos(deg2rad(currentPose.theta))) * (params.yLimit - currentPose.y))});

    //     derivative = drive_error - prevError;
    //     motorPower = (LAT_KP * drive_error) + (LAT_KD * derivative);

    //     if(checkRing(nearestRing)){
    //         if(timeout % 90 == 0){
    //             //printf("(%d, %d)\t", nearestRing.x_middle_coord, nearestRing.y_middle_coord);
    //         }
    //         int vision_error = nearestRing.x_middle_coord - VISION_CENTER;

    //         turnPower = VISION_TURN_KP * vision_error;

    //         if(nearestRing.y_middle_coord < 0 && !params.keepDriving){
    //             int error = 106 + nearestRing.y_middle_coord;
    //             ringPower = VISION_LAT_KP * error;
    //             if(ringPower < motorPower){
    //                 motorPower = ringPower;
    //             }
    //         }
    //     }
    //     if(timeout % 20 == 0){
    //         printf("(%f, %f)\t", currentPose.x, currentPose.y);
    //         printf("DistToMax: %f\t", drive_error);
    //     }

    //     if(params.useLeftLine && params.useRightLine && detectLeftLine() && detectRightLine()){
    //         driveStraight(-motorPower);
    //         break;
    //     } else if(params.useRightLine && detectRightLine()){
    //         printf("Detecting line\t Theta: %f\t", chassis.getPose().theta);
    //         if(COLOR){
    //             turnPower = TURN_KP * fmod(-90 - currentPose.theta, 360);
    //         } else {
    //             turnPower = TURN_KP * fmod(90 - currentPose.theta, 360);
    //         }
    //     } else if(params.useLeftLine && detectLeftLine()){
    //         printf("Detecting line\t Theta: %f\t", chassis.getPose().theta);
    //         if(COLOR){
    //             turnPower = TURN_KP * fmod(90 - currentPose.theta, 360);
    //         } else {
    //             turnPower = TURN_KP * fmod(-90 - currentPose.theta, 360);
    //         }
    //     }

    //     leftSpeed = limitSpeed(motorPower + turnPower, params.maxSpeed), rightSpeed = limitSpeed(motorPower - turnPower, params.maxSpeed);

    //     if(timeout % 20 == 0){
    //         printf("Left: %f\t Right %f\t", leftSpeed, rightSpeed);
    //         printf("\n");
    //     }
    //     drive(leftSpeed, rightSpeed);
    //     prevError = drive_error;

    //     pros::delay(15);
    //     timeout -= 15;
        
    // }
    // if(timeout <= 0){
    //     printf("Stop by timeout");
    // }

    // stopDriveHold();
    // printf("\n");
}

/** driveToRingParams are same as driveToRing except for maxDist
 *  maxDist counts distance beyond distance to the original point
 */
void moveToPointWithVis(float x, float y, int timeout, driveToRingParams params, int delay){
    // chassis.moveToPoint(x, y, timeout, {.maxSpeed = params.maxSpeed});
	// int temp = pros::millis();
	// while((!checkRing(getMostRelevantObject(params.color)) || pros::millis() - temp < delay) && chassis.isInMotion()){
	// 	pros::delay(20);
	// }
    // lemlib::Pose currentPose = chassis.getPose();
    // printf("(%f, %f)\tMax: %f\n", currentPose.x, currentPose.y, params.maxDist);
	// if(chassis.isInMotion()){
	// 	chassis.cancelAllMotions();
    //     pros::delay(10);
    //     currentPose = chassis.getPose();
    //     params.maxDist += sqrt(pow(x - currentPose.x, 2) + pow(y - currentPose.y, 2));
    //     printf("(%f, %f)\tMax: %f\n", currentPose.x, currentPose.y, params.maxDist);
	// 	driveToRing(timeout - (pros::millis() - temp), params);
	// }

}

void turnToHeadingWithVis(float angle, int timeout,int range, driveToRingParams params,int delay)
{
    // chassis.turnToHeading(angle, timeout, {.maxSpeed = (int)params.maxSpeed});
    // int temp = pros::millis();
	// while((!checkRing(getMostRelevantObject(params.color)) 
    //        || pros::millis() - temp < delay) && chassis.isInMotion() 
    //        || abs(int(chassis.getPose().y - angle)) > range){
    //         pros::delay(20);
    //         temp += 20;
	// }
    // lemlib::Pose currentPose = chassis.getPose();
    // printf("(%f, \n", currentPose.theta);
	// if(chassis.isInMotion()){
	// 	chassis.cancelAllMotions();
    //     pros::delay(10);
	// 	turnToRing(timeout - (pros::millis() - temp),params.maxSpeed, params.color);
	// }

}

void turnToHeadingWithVisGoal(float angle, int timeout,int range, int speed, int delay)
{

    // chassis.turnToHeading(angle, timeout, {.maxSpeed = speed});
    // int temp = pros::millis();
	// while((!checkRing(vision_sensor.get_by_sig(0, 3)) || pros::millis() - temp < delay) && chassis.isInMotion()){
    //     if(abs(int(chassis.getPose().y - angle)) > range)
    //     {
	// 	    pros::delay(20);
    //     }
	// }
    // lemlib::Pose currentPose = chassis.getPose();
    // printf("(%f, \n", currentPose.theta);
	// if(chassis.isInMotion()){
	// 	chassis.cancelAllMotions();
    //     pros::delay(10);
	// 	turnToGoal(timeout - (pros::millis() - temp),speed);
	// }

}


float calcDistance(){

    // pros::vision_object_s_t nearestRing = getMostRelevantObject();

    // return (7 * 158.) / (nearestRing.width * tan(deg2rad(32.3)));
    return 0;
}

float calcDistanceGoal(){

    // pros::vision_object_s_t nearestGoal = vision_sensor.get_by_sig(0, 3);
    // if(checkRing(nearestGoal)){
    //     return (11.6 * 158.) / (nearestGoal.width * tan(deg2rad(32.3)));
    // } else {
    //     return -1;
    // }
    return 0;
}

void driveFullVision(int timeout, int maxSpeed) {
    // int hueLower = (COLOR) ? redLower : blueLower, hueUpper = (COLOR) ? redUpper : blueUpper;
    // autoDrive = true;
    // float turnPower, motorPower = maxSpeed;
    // float distance, prevDistance = 0, derivative = 0, prev_vision_error = 0,turn_derivative = 0;
    // pros::delay(50);
    // while((getIntakeColor() < hueLower || getIntakeColor() > hueUpper) && timeout > 0 && (auton || autoSkill) && autoDrive) {

    //     pros::vision_object_s_t nearestRing = getMostRelevantObject();

    //     if(nearestRing.signature != VISION_OBJECT_ERR_SIG){

    //         int vision_error = nearestRing.x_middle_coord - VISION_CENTER;

    //         turnPower = VISION_TURN_KP * vision_error;

    //         distance = calcDistance();

    //         derivative = distance - prevDistance;

    //         turn_derivative = vision_error - prev_vision_error;

    //         motorPower = LAT_KP * distance + LAT_KD * derivative;

    //         //printf("Drive: %f\t Turn: %f\n", motorPower, turnPower);

    //         prevDistance = distance;
    
    //         prev_vision_error = vision_error;

    //         if(motorPower > maxSpeed){
    //             motorPower = maxSpeed;
    //         }
    //     }

    //     drive(motorPower + turnPower, motorPower - turnPower);
    //     //printf("Motor Power: %f\n", motorPower);

    //     pros::delay(15);
    //     timeout -= 15;
    // }
    // stopDrive();
}


void saveOurRing(int timeout){
    // int time = 0;
    // while (true) {
    //     if (time >= timeout)
    //     {
    //         return;
    //     }
    //     int hue = get2ndIntakeColor();
    //     //printf("Color %d\n",hue);
    //     if(detectOurColor(hue))
    //     {
    //         //pros::delay(100);
    //         stopIntake();
    //         return;
    //     }
    //     pros::delay(20);
    //     time += 20;
    // }
}

void saveRing(int timeout){
    // int time = 0;
    // while (true) {
    //     if (time >= timeout)
    //     {
    //         return;
    //     }
    //     int hue = get2ndIntakeColor();
    //     if(detectOurColor(hue))
    //     {
    //         pros::delay(100);
    //         stopIntake();
    //         return;
    //     }
    //     if(detectTheirColor(hue))
    //     {
    //         pros::delay(100);
    //         stopIntake();
    //         return;
    //     }
    //     pros::delay(20);
    //     time += 20;
    // }
}

void saveOurRing1(int timeout){
    // int time = 0;
    // while (true) {
    //     if (time >= timeout)
    //     {
    //         return;
    //     }
    //     int hue = getIntakeColor();
    //     //printf("Color %d\n",hue);
    //     if(detectOurColor(hue))
    //     {
    //         pros::delay(100);
    //         stopIntake();
    //         return;
    //     }
    //     pros::delay(20);
    //     time += 20;
    // }
}

void saveRing1(int timeout){
    // int time = 0;
    // while (true) {
    //     if (time >= timeout)
    //     {
    //         return;
    //     }
    //     int hue = getIntakeColor();
    //     if(detectOurColor(hue))
    //     {
    //         //pros::delay(100);
    //         stopIntake();
    //         return;
    //     }
    //     if(detectTheirColor(hue))
    //     {
    //         //pros::delay(100);
    //         stopIntake();
    //         return;
    //     }
    //     pros::delay(20);
    //     time += 20;
    // }
}

void saveRingDist(int timeout){
    // int time = 0;
    // while(time < timeout){
    //     if(detectRingFront()){
    //         stopIntake();
    //         return;
    //     }
    // }
}
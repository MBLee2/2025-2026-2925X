#include "controls.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#define TURN_CONST 1.4
//Drivebase control
double now_time;

void taskFn_drivebase_control(void)
{
    
    bool lift_status = false;
    printf("%s(): Entered \n", __func__);
    while (true) 
    {
        int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        int turnVelright = TURN_CONST*rightX;
        int turnVelleft = TURN_CONST*leftX;

        
       if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) 
        {
            if (lift_status == false)
            {
                lift_status = true;
                lift_pistons.set_value(true);

            }
            else if(lift_status == true)
            {
                lift_status = false;
                lift_pistons.set_value(false);
                intake_mtr.move(-127);
    
            }   
        }

        //chassis.arcade(leftY,turnVelleft);
        //chassis.arcade(leftY,turnVelright);
        left_side_motors.move(leftY + turnVelright);
		right_side_motors.move(leftY - turnVelright);
        //chassis.tank(leftY,rightY,0);


            


        //chassis.curvature(leftY,rightX,0);
        /*if(imu.get_roll()< -45){
           left_side_motors.move(-100);
           right_side_motors.move(-100);
        }//*/
        
        //C tier Climb
        pros::delay(20);

    } // end of while loop

    printf("%s(): Exiting \n", __func__);
    
} // end of taskFn_drivebase_control

//Flwheel control
void taskFn_flywheel_control(void)
{
    printf("%s(): Entered \n", __func__);
    cata_mtr1.tare_position();
    bool cata_status = false;
    bool cata_angle = 0;
    lemlib::Timer match (20000);

    while (true) 
    {   
        if(match.getTimeLeft() < 10000) {
            while(true)
            {
                master.rumble("");
                pros::delay(1000);
            }
        }
        pros::delay(20);
    } // end of while loop

    printf("%s(): Exiting \n", __func__);

} // end of taskFn_flywheel_control

//Intake control
void taskFn_intake_control(void)
{
    printf("%s(): Entered \n", __func__);
    while (true) 
    {
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            intake_mtr.move(-127);  
        }   
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            intake_mtr.move(127);
            while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            {
            intake_mtr.move(-127);  
            }     
        } 
        intake_mtr.move(0); 

    /*if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) 
        {
            lift_pistons.set_value(true);
            pros::delay(500);
            lift_pistons.set_value(false);       
        }
        pros::delay(10);*/
        //MODIFIED INTAKE

        
        // NON SHIFT
        /*while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
        {
            intake_mtr.move(127);
        }
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) 
        {
            intake_mtr.move(-127);         
        } 
		

        intake_mtr.move(0);//*/

    } // end of while loop

    printf("%s(): Exiting \n", __func__);

} // end of taskFn_intake_control

//Wings Control
void taskFn_wings_control(void)
{
    bool status_wings = false;
    while (true) 
    {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){ left_piston.set_value(true);} 
        else left_piston.set_value(status_wings);


        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){ right_piston.set_value(true);} 
        else right_piston.set_value(status_wings);

        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            status_wings = !status_wings;
            left_piston.set_value(status_wings);
            right_piston.set_value(status_wings);
            
        }
        pros::delay(20);
    }
}

void taskFn_auto_intake_control(void)
{
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        intake_mtr.move(-127);  
    }   
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        intake_mtr.move(100);
        /*if(distance_sensor.get()>= 100)
        {
            intake_mtr.brake();
            break;
        }*/
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Shiiii: %3d", distance_sensor.get());    
    } 
    intake_mtr.move(0); 

}
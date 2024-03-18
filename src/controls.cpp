#include "controls.h"
#include "robot_config.h"
#include "lemlib/api.hpp"
#define TURN_CONST 1.3
 

//Drivebase control
double now_time;

void taskFn_drivebase_control(void)
{
    printf("%s(): Entered \n", __func__);
    while (true) 
    {
        int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        int turnVelright;
        int turnVelleft;

        if(rightX > 0) //PERFECT QUADRATIC
        {
            turnVelright = (rightX * rightX)/(127/TURN_CONST);
            turnVelleft = (leftX * leftX)/(127/TURN_CONST);
        }
        else if (rightX < -0) //PERFECT QUADRATIC
        {
            turnVelright = -1*(rightX * rightX)/(127/TURN_CONST);
            turnVelleft = (leftX * leftX)/-1*(127/TURN_CONST);
        }
        else
        {
            turnVelright = 0;
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


    while (true) 
    {

       if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) 
        { //WHEN SHIFT KEY IS PRESSED
        if (cata_status == false)
            {
                cata_status = true;
                //cata_mtr1.move_voltage(12000); 
                //cata_motors.move_voltage(9100);
                cata_motors.move_voltage(8590);
                printf("Postition \n",cata_mtr1.get_position());

            }
            else if(cata_status == true)
            {
                cata_status = false;
                cata_motors.move_voltage(0);


            }//*/

        }
        //WHEN SHIFT KEY IS NOT PRESSED
        
        
        
        pros::delay(20);
        /* now_time = pros::millis();
        if (now_time - opcontrol_start_time == 105000)
        {
            lift_status = false;
            lift_pistonsF.set_value(false);
            lift_pistonsB.set_value(false);
        }//*/
        //printf("%s(): Exiting \n", __func__);    

    } // end of while loop

    printf("%s(): Exiting \n", __func__);

} // end of taskFn_flywheel_control

//Intake control
void taskFn_intake_control(void)
{
    bool lift_status = false;

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
            }   
        }
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
    while (true) 
    {
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            left_piston.set_value(true);
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            {
                right_piston.set_value(true);
            }
            else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == false)
            {
                right_piston.set_value(false);
            }
        }   
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            right_piston.set_value(true);
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            {
                left_piston.set_value(true);
            }
            else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == false)
            {
                left_piston.set_value(false);
            }
        } 
        right_piston.set_value(false);
        left_piston.set_value(false);
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
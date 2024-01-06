#include "controls.h"
#include "robot_config.h"
#include "lemlib/api.hpp"

//Drivebase conrol
void taskFn_drivebase_control(void)
{
    printf("%s(): Entered \n", __func__);

    while (true) 
    {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY,rightX,0);
 
        bool PTO_status = false;

        // A tier climb
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) 
        {
            if (PTO_status == false)
            {
                PTO_status = true;
                PTO_piston.set_value(true);
                lift_pistons.set_value(false);
                chassis.arcade(127,0);
                pros::delay(1000);
                chassis.arcade(0,0);            
            }
            else if (PTO_status == true)
            {
                PTO_status = false;
                PTO_piston.set_value(false);

            }
        }

        //B tier climb
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) 
        {
            if (PTO_status == false)
            {
                PTO_status = true;
                PTO_piston.set_value(true);
                lift_pistons.set_value(false);
                chassis.arcade(127,0);
                pros::delay(1400);
                chassis.arcade(0,0);            
            }
            else if (PTO_status == true)
            {
                PTO_status = false;
                PTO_piston.set_value(false);
            }          
        }

        //C tier Climb
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) 
        {
            if (PTO_status == false)
            {
                PTO_status = true;
                PTO_piston.set_value(true);
                lift_pistons.set_value(false);
                chassis.arcade(127,0);
                pros::delay(1000);
                chassis.arcade(0,0);
                back_wing_piston.set_value(true);
                pros::delay(200);
   
                chassis.arcade(127,0);
                pros::delay(1000);
                chassis.arcade(0,0);//*/
            }
            else if (PTO_status == true)
            {
                PTO_status = false;
                PTO_piston.set_value(false);

            }          
        }  
        
        //move the chassis with curvature drive
        //chassis.curvature(leftY, rightX);//*/

        pros::delay(20);

    } // end of while loop

    printf("%s(): Exiting \n", __func__);
    
} // end of taskFn_drivebase_control

//Flwheel control
void taskFn_flywheel_control(void)
{
    printf("%s(): Entered \n", __func__);
 
    bool cata_status = false;
    bool lift_status = false;

    while (true) 
    {
        //WHEN SHIFT KEY IS PRESSED
        while (master.get_digital (pros::E_CONTROLLER_DIGITAL_L2)) 
        {   
            if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_L1)) 
            {
                if (cata_status == false)
                {
                    cata_status = true;
                    lift_status == false;
                    lift_pistons.set_value(false);
                    //cata_mtr1.move_voltage(12000); 
                    //cata_motors.move_voltage(6000);
                    cata_motors.move(82.55);
                }
                else if(cata_status == true)
                {
                    cata_status = false;
                    lift_status == false;
                    lift_pistons.set_value(false);
                    //cata_mtr1.move_voltage(0);
                    cata_motors.move_voltage(0);
                }
            } 
        }

        //WHEN SHIFT KEY IS NOT PRESSED
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) 
        {
            if (cata_status == false && lift_status == false)
            {
                cata_status = true;
                lift_status = true;
                lift_pistons.set_value(true);
                //cata_mtr1.move_voltage(12000); 
                cata_motors.move_voltage(0);

            }
            else if(cata_status == true && lift_status == true)
            {
                cata_status = false;
                lift_status = false;
                lift_pistons.set_value(false);
                //cata_mtr1.move_voltage(0);
                cata_motors.move_voltage(0);
            }           
        }
        printf("%s(): Exiting \n", __func__);    

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
            while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            {
                intake_mtr.move(-127);         
            } 
		    pros::delay(20);

            intake_mtr.move(0);
        }   

        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) 
        {
            while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            {
                intake_mtr.move(127);         
            } 
            intake_mtr.move(0);
        }
        
        // NON SHIFT
        /*while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
        {
            intake_mtr.move(127);
        }
        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) 
        {
            intake_mtr.move(-127);         
        } 
		pros::delay(20);

        intake_mtr.move(0);//*/

    } // end of while loop

    printf("%s(): Exiting \n", __func__);

} // end of taskFn_intake   _control

//Wings Control
void taskFn_wings_control(void)
{
  printf("%s(): Entered \n", __func__);
    bool wings_front_status = false; 
    bool wings_back_status = false; 
    while (true)
    {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) 
        {
            if (wings_front_status == false)
            {
                wings_front_status = true;
                right_piston.set_value(true);
                left_piston.set_value(true);  
                intake_mtr.move(-127);
            }
            else if(wings_front_status == true)
            {
                wings_front_status = false;
                right_piston.set_value(false);
                left_piston.set_value(false);
                intake_mtr.move(0);  

            }           
        } 
        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) 
        {
            if (wings_back_status == false)
            {
                wings_back_status = true;
                back_wing_piston.set_value(true);  
            }
            else if(wings_back_status == true)
            {
                wings_back_status = false;
                back_wing_piston.set_value(false);
            }           
        } 





        /*while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            {
                back_wing_piston.set_value(true); 
            } 
		    pros::delay(20);
            back_wing_piston.set_value(false);
        }  

        while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
        {
            while (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            {
                right_wing_piston.set_value(true);      
                left_wing_piston.set_value(true);  
            } 
            pros::delay(20);
            right_wing_piston.set_value(false);      
            left_wing_piston.set_value(false);
            
        }//*/
    }
}


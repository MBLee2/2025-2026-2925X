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
 
    bool flywheel_status = false;
    bool lift_status = false;

    while (true) 
    {
        //WHEN SHIFT KEY IS PRESSED
        while (master.get_digital (pros::E_CONTROLLER_DIGITAL_L2)) 
        {   
            if (master.get_digital_new_press (pros::E_CONTROLLER_DIGITAL_L1)) 
            {
                if (flywheel_status == false)
                {
                    flywheel_status = true;
                    lift_status == false;
                    //flywheel_mtr.move_voltage(11964); 
                    lift_pistons.set_value(false);
                    flywheel_mtr = 98;

                }
                else if(flywheel_status == true)
                {
                    flywheel_status = false;
                    lift_status == false;
                    flywheel_mtr.move_voltage(0); 
                    lift_pistons.set_value(false);
                    //flywheel_mtr = 110;
                }
            } 
        }

        //WHEN SHIFT KEY IS NOT PRESSED
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) 
        {
            if (flywheel_status == false && lift_status == false)
            {
                flywheel_status = true;
                lift_status = true;
                flywheel_mtr.move_voltage(12000); 
                lift_pistons.set_value(true);
                //flywheel_mtr = 127;
            }
            else if(flywheel_status == true && lift_status == true)
            {
                flywheel_status = false;
                lift_status = false;
                flywheel_mtr.move_voltage(0);
                lift_pistons.set_value(false);
                //flywheel_mtr = 110;
            }           
        }
        printf("%s(): Exiting \n", __func__);    

        //NON SHIFT
        /* if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) 
        {
            if (flywheel_status == false)
            {
                flywheel_status = true;
                flywheel_mtr.move_voltage(11964); 
                //flywheel_mtr = 110;

            }
            else if(flywheel_status == true)
            {
                flywheel_status = false;
                flywheel_mtr.move_voltage(0);
                //flywheel_mtr = 110;
            }           
        }//*/
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

//PTO Control
void taskFn_PTO_control(void)
{
    printf("%s(): Entered \n", __func__);

    bool PTO_status = false;
    while (true) 
    {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) 
        {
            if (PTO_status == false)
            {
                PTO_status = true;
                PTO_piston.set_value(true);
                lift_pistons.set_value(false);  
                flywheel_mtr.move(0);
            }
            else if(PTO_status == true)
            {
                PTO_status = false;
                PTO_piston.set_value(false);

            }           
        } 
    } // end of while loop

    printf("%s(): Exiting \n", __func__);

}
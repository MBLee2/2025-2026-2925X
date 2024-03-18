#include "dashboard.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "robot_config.h"
#include "lemlib/api.hpp"


// TODO: Figure out why it gives a segmentation fault if this declaration is moved from here to "robot_config.cpp"
// Short term fix is to have this declaration in "robot_config.cpp" instead of here!! See "robot_config.cpp"
/*// define the auton menu buttons
std::vector <dashboard_motor_display> dashboard_motor_display_items = 
{
    dashboard_motor_display {  5, 135, "DB-LF", left_mtrF},
    dashboard_motor_display {105, 135, "DB-LB", left_mtrB},
    dashboard_motor_display {  5, 190, "DB-RF", right_mtrF},
    dashboard_motor_display {105, 190, "DB-RB", right_mtrB},
    dashboard_motor_display {205, 135, "Pnchr", puncher_mtr},
    dashboard_motor_display {205, 190, "Intk",  intake_mtr},
    dashboard_motor_display {305, 135, "Lft-1", right_lift_mtr},
    dashboard_motor_display {305, 190, "Lft-2", left_lift_mtr}
}; //*/


void draw_dashboard_motor_display(dashboard_motor_display d) 
/**
 * @brief Render a rectangular display on the opcontrol() or autonomous() screen
 * @param d for display. See definition of the struct dashboard_motor_display for details
 * @return void
*/
{
    // draw the rectangles representing the various parts of the display
    draw_rectangle_patch(d.motor_name);
    draw_rectangle_patch(d.port_number);
    draw_rectangle_patch(d.motor_temp);
    draw_rectangle_patch(d.over_current);

} // end draw_dashboard_motor_display() 


void render_dashboard()
/**
 * @brief Render all the displays for the dashboard 
 * @param None
 * @return void
*/
{
    // printf("%s(): Entered \n", __func__);

    // draw the outer container box
    pros::screen::set_pen(COLOR_DARK_GRAY);
    pros::screen::fill_rect(1, 100, 479, 239); 
    // Put the title box at the top
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 175, 110, "Robot Dashboard");

    // draw all the dashboard_motor_display_items 
    for (int i=0; i<dashboard_motor_display_items.size(); i++) 
    {
        // Check the motor temperatures, and update display accordingly
        // Mark RED if over temperature detected
        if (dashboard_motor_display_items[i].mtr.is_over_temp())
        {
            dashboard_motor_display_items[i].motor_temp.fill_color = COLOR_RED;
            dashboard_motor_display_items[i].motor_temp.text 
                = std::to_string(int(dashboard_motor_display_items[i].mtr.get_temperature())) + " OT";
        }
        // Mark YELLOW if over 55 degrees celcius
        else if (dashboard_motor_display_items[i].mtr.get_temperature() > 55)
        {
            dashboard_motor_display_items[i].motor_temp.fill_color = COLOR_YELLOW;
            dashboard_motor_display_items[i].motor_temp.text 
                = std::to_string(int(dashboard_motor_display_items[i].mtr.get_temperature()));
        }
        // Mark GREEN in all other cases
        else 
        {
            dashboard_motor_display_items[i].motor_temp.fill_color = COLOR_GREEN;
            dashboard_motor_display_items[i].motor_temp.text 
                = std::to_string(int(dashboard_motor_display_items[i].mtr.get_temperature()));
        }
        /*printf("%s(): %s Motor on port [%d], temperature [%f], Over Termperature [%d]\n", __func__, 
            dashboard_motor_display_items[i].mtr_name.data(),
            dashboard_motor_display_items[i].mtr.get_port(),
            dashboard_motor_display_items[i].mtr.get_temperature(),
            dashboard_motor_display_items[i].mtr.is_over_temp());//*/


        // Check the motor for over current, and update display accordingly
        // Mark RED if over current detected
        if (dashboard_motor_display_items[i].mtr.is_over_current())
        {
            dashboard_motor_display_items[i].over_current.fill_color = COLOR_RED;
            dashboard_motor_display_items[i].over_current.text = "OC";
        }
        // Mark GRAY in all other cases
        else 
        {
            dashboard_motor_display_items[i].over_current.fill_color = COLOR_GRAY;
            // current output is in milli Amps; convert to Amps
            dashboard_motor_display_items[i].over_current.text 
                = std::to_string(float(dashboard_motor_display_items[i].mtr.get_current_draw())/1000);
        }
        /*printf("%s(): %s Motor on port [%d], Current Draw [%s], Over Current [%d]\n", __func__, 
            dashboard_motor_display_items[i].mtr_name.data(),
            dashboard_motor_display_items[i].mtr.get_port(),
            std::to_string(float(dashboard_motor_display_items[i].mtr.get_current_draw())/1000),
            dashboard_motor_display_items[i].mtr.is_over_current());//*/

        // display the dashboard item 
        draw_dashboard_motor_display(dashboard_motor_display_items[i]);

    } // end for()

    // printf("%s(): Exiting\n", __func__);

} // end render_dashboard()


void render_compass_rose()
/**
 * @brief Render the compass rose display to show heading
 * @param None
 * @return void
*/
{
    // printf("%s(): Entered \n", __func__);

    // define the compass rose 
    int cc_x = 425; int cc_y = 50;  // compass center "cc" coordinates
    int rad = 40;                   // compass rose size (radius)

    // Note: An alphabet is assumed to be of 5x5 pixel size for E_TEXT_SMALL
    // and 10x10 pixel size for E_TEXT_MEDIUM. 

    // clear out the compass rose areas
    pros::screen::set_pen(COLOR_BLACK);
    pros::screen::fill_circle(cc_x, cc_y, rad+5);

    // re-draw the compass rose
    pros::screen::set_pen(COLOR_TEAL);
    pros::screen::draw_circle(cc_x, cc_y, rad);
    // draw the axes
    pros::screen::draw_line(cc_x-rad, cc_y,     cc_x+rad, cc_y);
    pros::screen::draw_line(cc_x,     cc_y-rad, cc_x,     cc_y+rad);
    // print the 4 cardinal directions
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL,  cc_x-5,      cc_y-rad-5, "0"  );
    pros::screen::print(pros::E_TEXT_SMALL,  cc_x+rad-5,  cc_y-5,     "90" );
    pros::screen::print(pros::E_TEXT_SMALL,  cc_x-10,     cc_y+rad-5, "180");
    pros::screen::print(pros::E_TEXT_SMALL,  cc_x-rad-20, cc_y-5,     "270");


    // get the current heading
    double current_heading = gps.get_heading();
    // display it if the values are in a valid range
    if (current_heading >= 0 && current_heading < 360)
    {
        // draw a "bug" on the compass rose depicting the current heading
        int bug_x = cc_x + rad * sin(deg2rad(current_heading));
        int bug_y = cc_y - rad * cos(deg2rad(current_heading));
        pros::screen::set_pen(COLOR_YELLOW);
        pros::screen::fill_circle(bug_x, bug_y, 3); 
        pros::screen::draw_line(cc_x, cc_y, bug_x,bug_y);
        // Also display the current heading in numbers in the middle of the compass rose
        pros::screen::print(pros::E_TEXT_MEDIUM, cc_x-15, cc_y-7, "%03.0f", current_heading);
    }
    
    // printf("%s(): Exiting\n", __func__);

} // end render_compass_rose())


void taskFn_display_gps_coordinates(void)
/**
 * @brief A task function to independently display the basic GPS coordinates on the V5 Brain 
 * sceenat all times. 
 * @param none
 * @return none 
*/
{
    printf("%s(): Started\n", __func__);

    while (true)
    {
        // Dsiplay the (x, y) coordinates
        pros::screen::set_pen(COLOR_YELLOW);
        pros::screen::print(pros::E_TEXT_MEDIUM, 250, 13, "GPS Readout");
        //pros::screen::print(pros::E_TEXT_MEDIUM, 250, 35, "  x: %+1.3f", gps.get_status().x);
        //pros::screen::print(pros::E_TEXT_MEDIUM, 250, 55, "  y: %+1.3f", gps.get_status().y);
        
        //lemlib status x and y and theta
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(pros::E_TEXT_MEDIUM, 250, 35, "x: %f", pose.x); // print the x position
        pros::screen::print(pros::E_TEXT_MEDIUM, 250, 55, "y: %f", pose.y); // print the y position
        pros::screen::print(pros::E_TEXT_MEDIUM, 250, 75, "heading: %f", pose.theta); // print the heading

        // Render the compass rose (dial) to show heading
        render_compass_rose();

        // display the RMS error rate
        //pros::screen::print(pros::E_TEXT_MEDIUM, 250, 75, "err: %+.3f", gps.get_error());

        // once every 100 milliseconds (10 times a second) is enough for this task 
        // which just refreshes the display of the latest GPS coordinates and heading
        pros::delay(100);
    }

    printf("%s(): Stopping\n", __func__);
}


void taskFn_dashboard_display(void)
/**
 * @brief The task function to be triggered from opcontrol() as well as autonomous() 
 * to run as an independent thread and display all critical parameters on the 
 * lower half of the V5 Brain screen
 * @param none
 * @return none 
*/
{
    printf("%s(): Started\n", __func__);

    while (true) 
    {
        // Render the dashboard screen
        render_dashboard();  
       
        // once every 200 milliseconds (5 times a second) is enough for this task 
        // which just refreshes the dashboard display
        pros::delay(200);

    } // end of while loop

    printf("%s(): Stopping\n", __func__);

} // end of taskFn_dashboard_display


#ifndef _DASHBOARD_H_
#define _DASHBOARD_H_

#include "api.h"
#include "auton_menu.h"

/**
 * A structure that uses s_rectangle_patch to create a 5 box "display"
 * to show all the critical parameters of a V5 motor
 *      ---------------------
 *      | Motor Name | Port |
 *      ---------------------
 *      | Temp  OT   |  OC  | 
 *      ---------------------
*/
typedef struct s_dashboard_motor_display
{
    int x1, y1; // top left (starting) coordinates of the overall display
    std::string mtr_name;
    pros::Motor &mtr;
    // Hard coded based on the above
    rectangle_patch motor_name   {x1,    y1,    x1+60,  y1+20, pros::c::COLOR_DARK_SLATE_GRAY, pros::c::COLOR_WHITE_SMOKE, mtr_name.data(),                pros::E_TEXT_SMALL};
    rectangle_patch port_number  {x1+60, y1,    x1+100, y1+20, pros::c::COLOR_DARK_SLATE_GRAY, pros::c::COLOR_WHITE_SMOKE, std::to_string(mtr.get_port()), pros::E_TEXT_SMALL};
    rectangle_patch motor_temp   {x1,    y1+20, x1+60,  y1+45, pros::c::COLOR_DARK_SLATE_GRAY, pros::c::COLOR_WHITE_SMOKE, "-- --",                        pros::E_TEXT_SMALL};
    rectangle_patch over_current {x1+60, y1+20, x1+100, y1+45, pros::c::COLOR_DARK_SLATE_GRAY, pros::c::COLOR_WHITE_SMOKE, "--",                           pros::E_TEXT_SMALL};
    
} dashboard_motor_display; 


extern std::vector <dashboard_motor_display> dashboard_motor_display_items;

// functions needed for the dashboard task
void draw_dashboard_motor_display(dashboard_motor_display d);
void render_dashboard();
void render_compass_rose();

// task fuction for a displaying the dashboard
extern void taskFn_dashboard_display(void);
// task function for displaying GPS coordinates on the brain, when using the GPS sensor
extern void taskFn_display_gps_coordinates(void);


#endif // _DASHBOARD_H_
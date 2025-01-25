#include "auton_basics.h"
#include "auton_menu.h"
#include "auton_routines.h"
#include "robot_config.h"
#include "controls.h"
#include "lemlib/api.hpp"


// define the auton menu buttons
using pros::c::COLOR_WHITE;

std::vector<auton_menu_button> button_list = {
    auton_menu_button{{15, 50, 115, 100, pros::c::COLOR_DIM_GRAY,
                       pros::c::COLOR_WHITE_SMOKE, "PosSafe", pros::E_TEXT_MEDIUM},
                      safe_positive},
    auton_menu_button{{130, 50, 230, 100, pros::c::COLOR_DIM_GRAY,
                       pros::c::COLOR_WHITE_SMOKE, "PosWP", pros::E_TEXT_MEDIUM},
                      safe_positive},
    auton_menu_button{{245, 50, 345, 100, pros::c::COLOR_DIM_GRAY,
                       pros::c::COLOR_WHITE_SMOKE, "GoalRush", pros::E_TEXT_MEDIUM},
                      goal_rush},
    auton_menu_button{{360, 50, 460, 100, pros::c::COLOR_DIM_GRAY,
                       pros::c::COLOR_WHITE_SMOKE, "Rush+Stake", pros::E_TEXT_MEDIUM},
                       goal_rush},
     auton_menu_button{{15, 115, 115, 165, pros::c::COLOR_DIM_GRAY,
                       pros::c::COLOR_WHITE_SMOKE, "NegSix", pros::E_TEXT_MEDIUM},
                      neg_six_ring_red},
    auton_menu_button{{130, 115, 230, 165, pros::c::COLOR_DIM_GRAY,
                       pros::c::COLOR_WHITE_SMOKE, "NegSix+Stake", pros::E_TEXT_MEDIUM},
                      blue_neg_awp},
    auton_menu_button{{245, 115, 345, 165, pros::c::COLOR_DIM_GRAY,
                       pros::c::COLOR_WHITE_SMOKE, "NegSafe", pros::E_TEXT_MEDIUM},
                      safe_negative},
    auton_menu_button{{360, 115, 460, 165, pros::c::COLOR_DIM_GRAY,
                       pros::c::COLOR_WHITE_SMOKE, "StackBreak", pros::E_TEXT_MEDIUM},
                       blue_ring_rush},
    auton_menu_button{{15, 180, 230, 230, pros::c::COLOR_BEIGE,
                       pros::c::COLOR_WHITE_SMOKE, "Skills", pros::E_TEXT_MEDIUM},
                      skills_1},
    auton_menu_button{{245, 180, 460, 230, pros::c::COLOR_BEIGE,
                       pros::c::COLOR_WHITE_SMOKE, "solo_wp", pros::E_TEXT_MEDIUM},
                      solo_WP}};

void draw_rectangle_patch(rectangle_patch p) 
/**
 * @brief Render a rectangule_patch on the screen
 * @param p for patch. See definition of the struct auton_menu_button for details
 * @return void
*/
{
    // draw a rectanglular outline representing the button in WHITE
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::draw_rect(p.x1, p.y1, p.x2, p.y2);
    // draw a filled rectangle representing the button
    pros::screen::set_pen(p.fill_color);
    pros::screen::fill_rect(p.x1+1, p.y1+1, p.x2-1, p.y2-1);
    
    // assuming each character is 8 pixel wide, truncate the button name
    int max_name_length = round((p.x2 - p.x1)/8);
    std::string button_name = p.text.substr(0, max_name_length - 1);
    
    // calculate the print position in the center of the button rectangle
    int name_start_x = p.x1 + round(((p.x2 - p.x1) - button_name.length()*8)/2);
    int name_start_y = p.y1 + round((p.y2 - p.y1 - 8)/2);
    
    // Print the button name
    pros::screen::set_pen(p.text_color);
    pros::screen::print(p.text_size, name_start_x, name_start_y, button_name.data());

} // end draw_rectangle_patch() 


void highlight_rectangle_patch(rectangle_patch p) 
/**
 * @brief Highlight a rectangule_patch on the screen
 * @param p for patch. See definition of the struct auton_menu_button for details
 * @return void
*/
{
    // draw a rectangle representing the button
    pros::screen::set_pen(pros::c::COLOR_CRIMSON);
    pros::screen::fill_rect(p.x1, p.y1, p.x2, p.y2);
    pros::delay(400);

} // end highlight_rectangle_patch()


void draw_auton_menu_screen()
/**
 * @brief Render the screen for the auton menu screen
 * @param None
 * @return void
*/
{
    // printf("%s(): Entered \n", __func__);

    // clear the screen      
    pros::screen::set_eraser(pros::c::COLOR_BLACK);
    pros::screen::erase();

    // draw the outer container box
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::draw_rect(1, 1, 479, 239); 

    // Put the title box at the top
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 150, 10, "Auton Menu");

    // draw all the rectangle_patch representing the auton menu buttons
    for (int i=0; i<button_list.size(); i++) 
    {
        draw_rectangle_patch(button_list[i].button);
    }

    // printf("%s(): Exiting\n", __func__);

} // end draw_auton_menu_screen()


auton_routine select_auton_routine()
/**
* @brief Draw out the menu for selecting the right Auton on the VEX V5 Brain screen and then use 
*   touch to get the selected choice.
* @note Useable screen space on the V5 Brain screen is 479 x 239
* @param None 
* @return auton_type - returns the auton selected
*/
{
    printf("%s(): Entered \n", __func__);

    // start with an invalid option
    auton_routine selected_routine;

    bool selection_made = false;

    while(selection_made == false) 
    {
        // Render the auton menu screen
        draw_auton_menu_screen();
    
        // check if the screen was touched (pressed)
        pros::screen_touch_status_s_t touch_status = pros::screen::touch_status();
        if (touch_status.touch_status == pros::E_TOUCH_PRESSED) 
        {
            // printf("%s(): Screen Pressed at (%d, %d) \n", __func__, touch_status.x, touch_status.y);
            
            // check if any of the buttons in the menu were pressed
            for (int i=0; i<button_list.size(); i++) 
            {
                if (touch_status.x > button_list[i].button.x1 && touch_status.x < button_list[i].button.x2 &&
                    touch_status.y > button_list[i].button.y1 && touch_status.y < button_list[i].button.y2) 
                {
                    highlight_rectangle_patch(button_list[i].button);
                    selected_routine = button_list[i].associated_routine;
                    selection_made = true;

                    printf("%s(): Selected: [%s] \n", __func__, button_list[i].button.text.data());

                    // clear the screen      
                    pros::screen::set_eraser(pros::c::COLOR_BLACK);
                    pros::screen::erase();
                    // Print the selection as a confirmation
                    pros::screen::set_pen(pros::c::COLOR_ANTIQUE_WHITE);
                    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 3, "Selected: %s", 
                        button_list[i].button.text.data());
                    master.print(0, 0, "Auton: %s", button_list[i].button.text.data());
                    pros::delay(1000);
                    break; // selection made, end the for() loop
                }
            } // end for()
        }

        // A 250 millisecond (4 times a second) should be enough refresh rate to get user inputs
        pros::delay(250);

    } // end while(true)

    printf("%s(): Exiting - Selected: %s\n", __func__, selected_routine.routine_name.data());
    return selected_routine;

} // end select_auton_routine()


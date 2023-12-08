#ifndef _AUTON_MENU_H_
#define _AUTON_MENU_H_

#include "api.h"
#include "auton_basics.h"
#include "auton_routines.h"

/**
 * A structure to capture all the attributes of a rectangular "patch" on the V5 Brain 
 * Screen. This patch may be used to create interactuve "buttons" that can be "touched"
 * or "pressed" - e.g. in a auton selection menu button. Alternatively, the patch may 
 * be used to to create display boxes - e.g. a space on a dashboard to display data.
 * such as motor temperatue or over current warning, etc.
*/
typedef struct s_rectangle_patch
{
  int x1, y1, x2, y2; // top left & and bottom right coordinates of a rectangle
  int fill_color;
  int text_color;
  std::string text;
  pros::text_format_e_t text_size = pros::E_TEXT_MEDIUM;
} rectangle_patch;

/**
 * A structure that takes the rectangle_patch and creates a auton menu "button"
 * by associating an auton routine with the patch
*/
typedef struct s_auton_menu_button
{
  rectangle_patch button;
  auton_routine associated_routine; 
  
} auton_menu_button;


// functions to render the menu and read inputs from it
void draw_rectangle_patch(rectangle_patch p);
void highlight_rectangle_patch(rectangle_patch p);
void draw_auton_menu_screen();
auton_routine select_auton_routine(); 

#endif // _AUTON_MENU_H_

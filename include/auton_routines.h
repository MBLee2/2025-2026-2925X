#ifndef _AUTON_ROUTINES_H_
#define _AUTON_ROUTINES_H_

#include "api.h"
#include "auton_basics.h"

typedef void (*auton_routine_fPtr)();

// Auton routine start positions. This is a triad of values that define the starting
// position of the robot in terms of the GPS system's (x, y) coordinates and heading angle
// See: https://kb.vex.com/hc/en-us/articles/360061932711-Using-the-V5-GPS-Sensor 
typedef struct s_auton_routine
{
    float start_x, start_y;          // Valid range: -1.800 to 1.800 metres
    int start_heading;               // Valid range: 0 to 359.99 degrees
    std::string routine_name;        // Name of the routine
    auton_routine_fPtr routine_func; // callback function defining the actual routine
} auton_routine;


//change all of these
extern auton_routine null_routine; 
extern auton_routine goal_rush, blue_ring_rush, blue_neg_awp, neg_six_ring_red, rush_wp_a;
extern auton_routine safe_positive, negative_four,solo_WP;
extern auton_routine skills_1, skills_2;

// functions to render the menu and read inputs from it
void neg_6_ring_red();
void goal_rush_pull_back();
void ring_rush();
void blue_negative_four();
void DescoreRushElim();
void rushelim();
void rushWP();
void safePos();
void negativeFour();
void solo_wp();
void positive_half_wp();
void auton_60s_skills_1();
void auton_60s_skills_2();
#endif // _AUTON_ROUTINES_H_
#include "hal.h"
#include "fmt/format.h"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "robot_config.h"
#include "controls.h"
#include "main.h"
#include <cmath>
#include <cstddef>
#include <cstdio>
#include "controls.h"
#include "auton_basics.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "main.h"

class wall_sensor{       // The class
  public:             
    double X_offset; // Distance out from the center of the robot
    double Y_offset; // Distance from center perimeter of robot
    std::uint8_t dist_port;
    pros::Distance mydist;
    double theta = 0; //radians


    wall_sensor(double X_offset, double Y_offset, std::uint8_t dist_port): mydist(dist_port) { // Constructor with parameters
      this->X_offset = X_offset;
      this->Y_offset = Y_offset;
      this->dist_port = dist_port;
    }

    float get_distance() {  // Gives distance away from wall does not convert to pure pursuit GPS system
        double dist = mydist.get_distance() / 25.4; // distance from wall converted to inches
        theta = lemlib::degToRad(chassis.getPose().theta);
        double pos =  cos(theta)*(dist + X_offset) - Y_offset*sin(theta);
        return pos;
    }
    
    float get_distance_raw() {  // Gives distance away from wall does not convert to pure pursuit GPS system
        return mydist.get_distance();
    }
    
    float get_distance_parrlel() {  // Gives distance away from wall does not convert to pure pursuit GPS system
        return mydist.get_distance() + X_offset;
    }
    
  private:
    int max_sensing_distance = 2000;  

    // Attribute (string variable)
};

wall_sensor left_sensor(7.25,0,15);
void reset()
{
    double x;
    double y;
    double wall_distance = 72.0;//inches

    bool xsign = std::signbit(chassis.getPose().x);
    double ysign = std::signbit(chassis.getPose().y);

    x = wall_distance - left_sensor.get_distance();
    printf("Distance: %f \n", left_sensor.get_distance()); 
    printf("Position: %f \n", x);


    if(xsign == false && ysign == false){
        chassis.setPose(x,y,chassis.getPose().theta);
    }
    else if(xsign == false && ysign == true){
        chassis.setPose(x,-y,chassis.getPose().theta);
    }
    else if(xsign == true && ysign == false){
        chassis.setPose(-x,y,chassis.getPose().theta);
    }
    else{
        chassis.setPose(-x,-y,chassis.getPose().theta);     
    }
}
float get_distance()
{
    return left_sensor.get_distance();
}
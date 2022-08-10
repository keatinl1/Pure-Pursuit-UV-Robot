// File:my_controller_cpp.cpp
// Date:09/08/22
// Author:Luke

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>

#include <traj.hpp>
#include <math.h>

#define PI 3.14159265

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = 16 ; //(int)robot->getBasicTimeStep();
  double max_speed = 6.28 ;
  double L = 5 ;
  double x_dist[80], y_dist[80], dist[80] ;


  // Motor instances
  Motor *motor_1 = robot->getMotor("motor_1");
  motor_1->setPosition(INFINITY);
  motor_1->setVelocity(0);
  
  Motor *motor_2 = robot->getMotor("motor_2");  
  motor_2->setPosition(INFINITY);
  motor_2->setVelocity(0);
  

  // GPS instances
  GPS *front_gps = robot->getGPS("front_gps");
  front_gps->enable(timeStep);
  
  GPS *center_gps = robot->getGPS("gps");
  center_gps->enable(timeStep);
  

  // Main loop:
  while (robot->step(timeStep) != -1) {

    // 0 - Read the sensors and reinitialise dist used in loop
    int next_wp_index = 0;
    
    double next_wp_dist = 100 ;
    
    const double *front_gps_value = front_gps->getValues();
    const double *center_gps_value = center_gps->getValues();
        
    
    // 1 - Find next waypoint
    for (int i = 0; i < 79; i++) {
      x_dist[i] =  (x_pos[i] - center_gps_value[0])*(x_pos[i] - center_gps_value[0]) ;
      y_dist[i] =  (y_pos[i] - center_gps_value[1])*(y_pos[i] - center_gps_value[1]) ;
      
      dist[i] = x_dist[i] + y_dist[i] ;
      
      if (dist[i] < next_wp_dist){
        next_wp_index = i ;
        next_wp_dist = dist[i] ;
      
      }
      
      }
      
    double next_x = x_pos[next_wp_index + 5] ;
    double next_y = y_pos[next_wp_index + 5] ;    
    
    
    // 2 - Find goal on lookahead circle
    double t = L / next_wp_dist ;
    
    double goal_x = (1 - t)*center_gps_value[0] + t* next_x ;
    double goal_y = (1 - t)*center_gps_value[1] + t* next_y ;
    
    
    // 3 - Find required and current heading
    double x = goal_x - center_gps_value[0] ;
    double y = goal_y - center_gps_value[1] ;
    double f_x = front_gps_value[0] - center_gps_value[0] ;
    double f_y = front_gps_value[1] - center_gps_value[1] ;

    double required_heading = atan2(y, x) ;
    double current_heading = atan2(f_y, f_x ) ;
    
    if (required_heading < 0) {
        required_heading = PI + (PI - abs(required_heading));
    }
    
    if (current_heading < 0 ) {
        current_heading = PI + (PI - abs(current_heading));
    }    
    
    
    // 4 - Align with required heading if not almost already
    if ((current_heading - required_heading) > 0.1) {
        while (abs(current_heading - required_heading) > 0.1) {
            robot->step(timeStep) ;
            const double left_speed = -0.25 * max_speed ;
            const double right_speed = 0.25 * max_speed ;

            motor_1->setVelocity(left_speed);
            motor_2->setVelocity(right_speed);
            
            const double *front_gps_value = front_gps->getValues();
            const double *center_gps_value = center_gps->getValues();
            
            x = goal_x - center_gps_value[0] ;
            y = goal_y - center_gps_value[1] ;
            f_x = front_gps_value[0] - center_gps_value[0] ;
            f_y = front_gps_value[1] - center_gps_value[1] ;
        
            required_heading = atan2(y, x) ;
            current_heading = atan2(f_y, f_x ) ;
            
            if (required_heading < 0) {
                required_heading = PI + (PI - abs(required_heading));
            }
            
            if (current_heading < 0 ) {
                current_heading = PI + (PI - abs(current_heading));
            }  
            
            std::cout << required_heading << std::endl;
            std::cout << current_heading << std::endl;
            std::cout << "A" << std::endl;
                        
        }
    }
    
    
    
    else if ((required_heading - current_heading) > 0.1 ) {
      while (abs(current_heading - required_heading) > 0.1 ){
        robot->step(timeStep) ;
        const double left_speed = 0.25 * max_speed ;
        const double right_speed = -0.25 * max_speed ;  
        
        motor_1->setVelocity(left_speed) ;
        motor_2->setVelocity(right_speed) ;
        
        const double *front_gps_value = front_gps->getValues();
        const double *center_gps_value = center_gps->getValues();
            
        x = goal_x - center_gps_value[0] ;
        y = goal_y - center_gps_value[1] ;
        f_x = front_gps_value[0] - center_gps_value[0] ;
        f_y = front_gps_value[1] - center_gps_value[1] ;
        
        required_heading = atan2(y, x) ;
        current_heading = atan2(f_y, f_x ) ;
            
        if (required_heading < 0) {
          required_heading = PI + (PI - abs(required_heading));
        }
            
        if (current_heading < 0 ) {
          current_heading = PI + (PI - abs(current_heading));
        }
        std::cout << required_heading << std::endl;
        std::cout << current_heading << std::endl;
        std::cout << "B" << std::endl;          
        
      }
    }        
    
    // 5 - Proceed forward
    const double left_speed = 0.25 * max_speed ;
    const double right_speed = 0.25 * max_speed ;  
        
    motor_1->setVelocity(left_speed) ;
    motor_2->setVelocity(right_speed) ;
    
    //std::cout << "forward" << std::endl;
    
  
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

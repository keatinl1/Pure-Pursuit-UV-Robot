#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#include <traj.h>
#include <motor_instructions.h>

using namespace webots;

int main(int argc, char **argv) {
  // Setup 
  Class Obj ;
  
  int timeStep = 16 ;
  float max_speed = 6.28 ;
  
  Robot *robot = new Robot();
  Motor *motor_1 = robot->getMotor("motor_1");
  Motor *motor_2 = robot->getMotor("motor_2");  
  
  motor_1->setPosition(INFINITY);
  motor_2->setPosition(INFINITY);  
  
  motor_1->setVelocity(0);
  motor_2->setVelocity(0);
  
  
  while (robot->step(timeStep) != -1) {
    // Loop

    const double speed = Obj.speed(max_speed) ;

    motor_1->setVelocity(speed) ;
    motor_2->setVelocity(speed) ;
    
  };

  delete robot;
  return 0;

}

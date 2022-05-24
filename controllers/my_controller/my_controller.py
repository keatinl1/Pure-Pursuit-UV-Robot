"""my_controller controller."""

from controller import Robot

if __name__ == "__main__":
    
    # Robot instance.
    robot = Robot()
    
    # Set the time step of the current world.
    timestep = 16
    max_speed = 6.28
    
    # Motor instances (defining where to send motor instructions)
    left_motor = robot.getDevice('motor_1')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor = robot.getDevice('motor_2')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    # LIDAR instance 
    lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    # GPS instance
    gps = robot.getDevice('gps')
    gps.enable(timestep)



# Main loop:
while robot.step(timestep) != -1:
    robot.step(timestep)
    left_speed = 0.25 * max_speed
    right_speed = 0.25 * max_speed
                                       
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

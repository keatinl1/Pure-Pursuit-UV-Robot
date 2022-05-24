"""drive_my_robot controller."""

from controller import Robot
import time

# Initialising stuff
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
    l_gps = robot.getDevice('left_gps')
    l_gps.enable(timestep)
    
    r_gps = robot.getDevice('right_gps')
    r_gps.enable(timestep)

    lap_complete = 0
    
    start_time = robot.getTime()
    elapsed_time = 0

    # Main loop:
    while robot.step(timestep) != -1:
        # this is where you read sensors
        # process data
        # issue commands to motors    
        
        l_gps_value = l_gps.getValues()
        r_gps_value = r_gps.getValues()
        range_image = lidar.getRangeImage()

        l_gps_x = l_gps_value[0]
        r_gps_x = r_gps_value[0]
        l_gps_y = l_gps_value[1]
        r_gps_y = r_gps_value[1]
        
        x_start = (l_gps_x + r_gps_x) / 2        
        y_start = (l_gps_y + r_gps_y) / 2  
        

#################################################################################
# make robot face left 

        while abs(r_gps_x - l_gps_x) > 0.01 or (r_gps_y < l_gps_y):
        # make robot face left wall
            robot.step(timestep)
            print("turning to face wall... \n")
            
            left_speed = -0.25 * max_speed
            right_speed = 0.25 * max_speed
                                       
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            
            if (l_gps_x > r_gps_x) and (l_gps_y < r_gps_y):
                left_speed = 0.25 * max_speed
                right_speed = -0.25 * max_speed
                                       
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)

            
            if (l_gps_x < r_gps_x) and (l_gps_y < r_gps_y):
                left_speed = -0.25 * max_speed
                right_speed = 0.25 * max_speed
                                       
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
                
            if (l_gps_x < r_gps_x) and (l_gps_y > r_gps_y):
                left_speed = -0.25 * max_speed
                right_speed = 0.25 * max_speed
                                       
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
                
            if (l_gps_x > r_gps_x) and (l_gps_y > r_gps_y):
                left_speed = 0.25 * max_speed
                right_speed = -0.25 * max_speed
                                       
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)

            l_gps_value = l_gps.getValues()
            r_gps_value = r_gps.getValues()
            
            l_gps_x = l_gps_value[0]
            r_gps_x = r_gps_value[0]
            l_gps_y = l_gps_value[1]
            r_gps_y = r_gps_value[1]



#################################################################################
# while you are more than 1.1m from left wall and time is < 30s
# progress forward
#
# could see this needing bailout

        x_curr = (l_gps_x + r_gps_x) / 2
        y_curr = (l_gps_y + r_gps_y) / 2

        while (x_curr - 1) > 0.2 and elapsed_time < 30 : 
        # get to 1m away from wall
            robot.step(timestep)
            current_time = robot.getTime()
            elapsed_time =  current_time - start_time
            print(elapsed_time)
            
            robot.step(timestep)
            print("moving to wall... \n")
            
            robot.step(timestep)
            left_speed = 0.50 * max_speed
            right_speed = 0.50 * max_speed

            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)  

            # bailout - if you get stuck on a wall
            # reverse for 2s 
            # veer right for 1 sec
            range_image = lidar.getRangeImage()
            
            if any(lidar < 0.25 for lidar in range_image):
                
                
                t_end = robot.getTime() + 1
                while robot.getTime() < t_end and elapsed_time < 30:
                    robot.step(timestep)
                    current_time = robot.getTime()
                    elapsed_time =  current_time - start_time
                
                    robot.step(timestep)
                    left_speed = -0.50 * max_speed
                    right_speed = -0.50 * max_speed
                    
                    left_motor.setVelocity(left_speed)
                    right_motor.setVelocity(right_speed)
                    
                    t_end = robot.getTime() + 1
                    while robot.getTime() < t_end:
                        robot.step(timestep)
                        left_speed = 0.50 * max_speed
                        right_speed = -0.25 * max_speed
                                
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(right_speed)    
    
                    
                
#--------------------------------------------------------------------------------
# if you encounter an obstacle while trying to reach wall
# turn 90° clockwise so you're facing top of room
# go forward for 0.15s
# turn 90° anti-clockwise so you're facing the left wall again
# back to going 0.5 towards wall
#
# could see this needing bailout

            if ((min(range_image[50:70])) < 0.5):
                print("reorient... \n")
                
                l_gps_value = l_gps.getValues()
                r_gps_value = r_gps.getValues()
                range_image = lidar.getRangeImage()
                
                l_gps_x = l_gps_value[0]
                r_gps_x = r_gps_value[0]
                l_gps_y = l_gps_value[1]
                r_gps_y = r_gps_value[1]
                
                while (l_gps_y < r_gps_y) and elapsed_time < 30:
                    print("reorient... \n")
                    print(elapsed_time)
                    robot.step(timestep)
                    current_time = robot.getTime()
                    elapsed_time =  current_time - start_time
                    
                    left_speed =  0.5 * max_speed
                    right_speed = - 0.5 * max_speed
                    
                    left_motor.setVelocity(left_speed)
                    right_motor.setVelocity(right_speed)
                    
                    l_gps_value = l_gps.getValues()
                    r_gps_value = r_gps.getValues()
                    
                    l_gps_y = l_gps_value[1]
                    r_gps_y = r_gps_value[1]
                
                

                t_end = time.time() + 0.5
                while time.time() < t_end:
                    robot.step(timestep)
                    
                    left_speed = 0.50 * max_speed
                    right_speed = 0.50 * max_speed

                    left_motor.setVelocity(left_speed)
                    right_motor.setVelocity(right_speed)  

                # bailout - if you get stuck on a wall
                # reverse for 2s 
                # veer right for 1 sec
                range_image = lidar.getRangeImage()
                
                if any(lidar < 0.25 for lidar in range_image) :
                    t_end = time.time() + 1
                    while time.time() < t_end:
                        robot.step(timestep)
                        left_speed = -0.50 * max_speed
                        right_speed = -0.50 * max_speed
                        
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(right_speed)
                        
                    t_end = time.time() + 1
                    while time.time() < t_end:
                        robot.step(timestep)
                        left_speed = 0.50 * max_speed
                        right_speed = 0.25 * max_speed
                                        
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(right_speed) 
                        
                
                while (l_gps_x < r_gps_x) and (elapsed_time < 30):
                    robot.step(timestep)
                    
                    left_speed =  -0.5 * max_speed
                    right_speed =  0.5 * max_speed
                    
                    left_motor.setVelocity(left_speed)
                    right_motor.setVelocity(right_speed)
                    
                    l_gps_value = l_gps.getValues()
                    r_gps_value = r_gps.getValues()
                    
                    l_gps_x = l_gps_value[0]
                    r_gps_x = r_gps_value[0]
                    
                    
            l_gps_value = l_gps.getValues()
            r_gps_value = r_gps.getValues()
            
            l_gps_x = l_gps_value[0]
            r_gps_x = r_gps_value[0]
            l_gps_y = l_gps_value[1]
            r_gps_y = r_gps_value[1]
            
            x_curr = (l_gps_x + r_gps_x) / 2
            y_curr = (l_gps_y + r_gps_y) / 2


#################################################################################
# align robot w wall

        while abs(r_gps_y - l_gps_y) > 0.01 and elapsed_time < 30: 
        # align robot w wall
            current_time = robot.getTime()
            elapsed_time =  current_time - start_time
            robot.step(timestep)
            print(elapsed_time)
            robot.step(timestep)
            print("getting in parallel with wall... \n")
            
            if r_gps_y > l_gps_y:
                left_speed = 0.25 * max_speed
                right_speed = -0.25 * max_speed
                                       
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
            if r_gps_y < l_gps_y:
                left_speed = -0.25 * max_speed
                right_speed = 0.25 * max_speed

                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
            l_gps_value = l_gps.getValues()
            r_gps_value = r_gps.getValues()
            
            l_gps_y = l_gps_value[1]
            r_gps_y = r_gps_value[1]
            

#################################################################################
# wall follower
# follow 1 meter from wall whole way round room       
# when it has completed a lap, exit loop
#
# don't think this needs a bailout
                
        x_1 = (l_gps_x + r_gps_x) / 2
        y_1 = (l_gps_y + r_gps_y) / 2 
                
        while lap_complete != 1:
        # wall follower
        
            current_time = robot.getTime()
            elapsed_time =  current_time - start_time
            print(elapsed_time)
            
            robot.step(timestep)
            print("following the wall... \n")

            left_speed = 0.5 * max_speed
            right_speed = 0.5 * max_speed
                
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            
            range_image = lidar.getRangeImage()
            
            if max(range_image[0:20]) > 0.6:
                print("turning left... \n")
                left_speed = 0.25 * max_speed
                right_speed = 0.5 * max_speed
                
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
            range_image = lidar.getRangeImage()
                
            if min(range_image[0:20]) < 0.4:
                print("turning right... \n")
                left_speed = 0.5 * max_speed
                right_speed = -0.25 * max_speed   
                
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
            range_image = lidar.getRangeImage()
            
            if range_image[64] < 1:
                print("wall ahead, turn... \n")
                while range_image[64] < 1.0:
                    robot.step(timestep)
                    left_speed = 0.5 * max_speed
                    right_speed = -0.5 * max_speed
                                               
                    left_motor.setVelocity(left_speed)
                    right_motor.setVelocity(right_speed)
                    
                    range_image = lidar.getRangeImage()
            
            l_gps_value = l_gps.getValues()
            r_gps_value = r_gps.getValues()
            
            l_gps_x = l_gps_value[0]
            r_gps_x = r_gps_value[0]
            l_gps_y = l_gps_value[1]
            r_gps_y = r_gps_value[1]
                    
            x_curr = (l_gps_x + r_gps_x) / 2
            y_curr = (l_gps_y + r_gps_y) / 2 
            
            #if abs(x_curr - x_1) < 1. and (y_1 - 1 > y_curr):
            #    lap_complete = 1
            
            if elapsed_time > 90:
                lap_complete = 1


#################################################################################
# wander around randomly after the lap

            if lap_complete == 1:
                print("wandering now... \n")
            
                while l_gps_y < r_gps_y :
                # make robot face left wall
                    robot.step(timestep)
                    
                    left_speed = 0.50 * max_speed
                    right_speed = - 0.50 * max_speed
                       
                    left_motor.setVelocity(left_speed)
                    right_motor.setVelocity(right_speed) 
                    
                    l_gps_value = l_gps.getValues()
                    r_gps_value = r_gps.getValues()
                    
                    l_gps_y = l_gps_value[1]
                    r_gps_y = r_gps_value[1]


                while lap_complete == 1:
                    robot.step(timestep)

                    left_speed = 0.50 * max_speed
                    right_speed = 0.50 * max_speed

                    left_motor.setVelocity(left_speed)
                    right_motor.setVelocity(right_speed)   
                    
                    range_image = lidar.getRangeImage() 
                    
                    if min(range_image[0:64]) < 0.5 :
                        left_speed = 0.50 * max_speed
                        right_speed = - 0.50 * max_speed
                        
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(right_speed) 
                        
                        
                    if min(range_image[65:128]) < 0.5 :
                        left_speed = - 0.50 * max_speed
                        right_speed =  0.50 * max_speed
                       
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(right_speed) 

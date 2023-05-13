"""my_controller controller."""
from controller import Robot
import math
import pandas as pd
import numpy as np

class StateEstimator:
    '''Return the heading from GPS data'''
    def __init__(self, center_gps, front_gps):
        self.center = center_gps
        self.front = front_gps
        self.current_heading = 0
        self.current_gps_center = [0, 0]
        self.current_gps_front = [0, 0]
        self.get_gps()

    def get_gps(self):
        '''get positional data from "GPS" on robot'''
        center_gps_vals = self.center.getValues()
        self.current_gps_center[0] = center_gps_vals[0]
        self.current_gps_center[1] = center_gps_vals[1]

        f_gps_vals = self.front.getValues()
        self.current_gps_front[0] = f_gps_vals[0]
        self.current_gps_front[1] = f_gps_vals[1]

        self.get_current_heading(self.current_gps_center[0], self.current_gps_center[1], self.current_gps_front[0], self.current_gps_front[1])

    def get_current_heading(self, center_x, center_y, front_x, front_y):
        '''get required and current headings from gps and goal data'''
        d_f_x = front_x - center_x
        d_f_y = front_y - center_y

        self.current_heading = np.arctan2(d_f_y, d_f_x)

        if self.current_heading < 0 :
            self.current_heading = np.pi + (np.pi - abs(self.current_heading))

class PathTracker:
    '''Find what goal waypoint should be, this assumes no obstacles'''
    def __init__(self, gps_list):
        self.gps_x = gps_list[0]
        self.gps_y = gps_list[1]
        self.next_x = 0
        self.next_y = 0
        waypoints = pd.read_excel('trajectory.xlsx')
        self.wp_array = np.array(waypoints)
        self.x_wp = self.wp_array[:, 0]
        self.y_wp = self.wp_array[:, 1]

        self.plan()

    def plan(self):
        '''Find next waypoint'''
        dist = ( self.x_wp - self.gps_x ) ** 2 + ( self.y_wp - self.gps_y ) ** 2

        i_closest_wp = np.argmin(dist)

        next_wp = self.wp_array[i_closest_wp + 1]

        self.next_x, self.next_y = next_wp[0], next_wp[1]

        return self.next_x, self.next_y

class PurePursuit:
    '''Controller Class'''
    def __init__(self, curr_heading, nxt_x_wypt, nxt_y_wypt, gps_center):
        self.lookahead_radius = 1.0
        self.current_heading = curr_heading
        self.required_heading = 0
        self.left_velocity = 0
        self.right_velocity = 0
        self.next_x = nxt_x_wypt
        self.next_y = nxt_y_wypt
        self.gps = gps_center
        self.find_goal_point()

    def find_goal_point(self):
        '''Find goal on lookahead circle'''
        dist_to_next = np.sqrt((self.next_x - self.gps[0])**2 + (self.next_y - self.gps[1])**2)

        t = self.lookahead_radius / dist_to_next

        goal_x, goal_y = (1 - t)*self.gps[0] + t* self.next_x, (1 - t)*self.gps[1] + t* self.next_y

        self.desired_heading(goal_x, goal_y)

    def desired_heading(self, goal_x, goal_y):
        '''find what heading we should be going'''
        x = goal_x - self.gps[0]
        y = goal_y - self.gps[1]

        self.required_heading = np.arctan2(y, x)

        if self.required_heading < 0 :
            self.required_heading = np.pi + (np.pi - abs(self.required_heading))

        self.calculate_wheel_velocities()

    def calculate_wheel_velocities(self, wheelbase=0.0, robot_width=0.5):
        '''function to calculate individual wheel vels based on headings and const vel'''

        velocity = 1.57

        # Calculate angular distance and direction
        angular_distance = self.required_heading - self.current_heading
        if angular_distance > math.pi:
            angular_distance -= 2 * math.pi
        elif angular_distance < -math.pi:
            angular_distance += 2 * math.pi

        # Calculate desired angular velocity
        angular_direction = 1.0 if angular_distance > 0 else -1.0
        desired_angular_velocity = abs(angular_distance) * angular_direction

        # Calculate time and linear distance
        time = abs(angular_distance) / desired_angular_velocity
        linear_distance = velocity * time

        # Calculate radius of curvature
        radius_of_curvature = linear_distance / abs(angular_distance)

        # Calculate left and right wheel velocities
        self.left_velocity = (velocity - (desired_angular_velocity * robot_width / 2)) * (1 - (wheelbase / 2) / radius_of_curvature)
        self.right_velocity = (velocity + (desired_angular_velocity * robot_width / 2)) * (1 + (wheelbase / 2) / radius_of_curvature)

def main():
    '''main function for simulation'''
    # Robot instance.
    robot = Robot()

    # Set the time step of the current world.
    timestep = 16

    # Motor instances (defining where to send motor instructions)
    left_motor = robot.getDevice('motor_2')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor = robot.getDevice('motor_1')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    # LiDAR instance
    lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    # GPS instance
    center_gps = robot.getDevice('gps')
    center_gps.enable(timestep)

    front_gps = robot.getDevice('front_gps')
    front_gps.enable(timestep)

    while True:

        # Timestep
        robot.step(timestep)

        # Process data from GPS to get headings
        state = StateEstimator(center_gps, front_gps)
        current_heading, gps_center = state.current_heading, state.current_gps_center

        print(gps_center)

        # Find where we want to go based on the state estimation and the path
        tracker = PathTracker(gps_center)
        next_x_waypoint, next_y_waypoint = tracker.next_x, tracker.next_y

        # print(next_x_waypoint, next_y_waypoint)

        # Find control actuations based on where we want to go
        controller = PurePursuit(current_heading,  next_x_waypoint, next_y_waypoint, gps_center)
        left_speed = controller.left_velocity
        right_speed = controller.right_velocity

        # Send the actuation
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":

    main()

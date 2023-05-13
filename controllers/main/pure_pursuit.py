'''Controller class'''

import math
import numpy as np

class PurePursuit:
    '''Controller Class'''
    def __init__(self, curr_heading, nxt_x_wypt, nxt_y_wypt, gps_center):
        self.lookahead_radius = 0.1
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

        velocity = 1.57 / 2

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

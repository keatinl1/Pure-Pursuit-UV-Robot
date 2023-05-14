'''Controller class'''
import numpy as np

class PurePursuit:
    '''Controller class'''
    def __init__(self, current_heading, next_x_wpt, next_y_wpt, gps_center):
        self.lookahead_radius = 0.5
        self.current_heading = current_heading
        self.next_x = next_x_wpt
        self.next_y = next_y_wpt
        self.gps_center = gps_center
        self.left_velocity = 0
        self.right_velocity = 0
        self.find_goal_point()

    def find_goal_point(self):
        '''find goal point on lookahead circle'''
        dist_to_next = np.linalg.norm([self.next_x - self.gps_center[0], self.next_y - self.gps_center[1]])
        t = self.lookahead_radius / dist_to_next
        goal_x, goal_y = (1 - t) * self.gps_center[0] + t * self.next_x, (1 - t) * self.gps_center[1] + t * self.next_y
        self.calculate_wheel_velocities(goal_x, goal_y)

    def calculate_wheel_velocities(self, goal_x, goal_y, velocity=0.39, wheelbase=0.33, robot_width=0.4):
        '''calculate the individual velocities to follow trajectory'''
        x, y = goal_x - self.gps_center[0], goal_y - self.gps_center[1]
        desired_heading = np.arctan2(y, x)
        if desired_heading < 0:
            desired_heading += 2 * np.pi

        # Calculate angular distance and direction
        angular_distance = desired_heading - self.current_heading
        if angular_distance > np.pi:
            angular_distance -= 2 * np.pi
        elif angular_distance < -np.pi:
            angular_distance += 2 * np.pi

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

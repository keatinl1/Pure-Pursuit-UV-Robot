'''Path tracking class'''
import numpy as np

class PathTracker:
    '''Find what goal waypoint should be, this assumes no obstacles'''
    def __init__(self, wp_array, gps_coords):
        self.x_wp = wp_array[:, 0]
        self.y_wp = wp_array[:, 1]
        self.gps_x, self.gps_y = gps_coords

    def get_next_waypoint(self, wp_array):
        '''Find next waypoint'''
        dist = (self.x_wp - self.gps_x)**2 + (self.y_wp - self.gps_y)**2
        i_closest_wp = np.argmin(dist)
        next_wp = wp_array[i_closest_wp + 3]
        next_x, next_y = next_wp[0], next_wp[1]
        return next_x, next_y

'''Path tracking class'''
import numpy as np
import pandas as pd

class PathTracker:
    '''Find what goal waypoint should be, this assumes no obstacles'''
    def __init__(self, gps_list):
        waypoints = pd.read_excel('trajectory.xlsx')
        self.wp_array = np.array(waypoints)
        self.x_wp = self.wp_array[:, 0]
        self.y_wp = self.wp_array[:, 1]
        self.gps_x = gps_list[0]
        self.gps_y = gps_list[1]
        self.next_x = 0
        self.next_y = 0
        self.plan()

    def plan(self):
        '''Find next waypoint'''
        dist = (self.x_wp - self.gps_x)**2 + (self.y_wp - self.gps_y)**2
        i_closest_wp = np.argmin(dist)
        next_wp = self.wp_array[i_closest_wp + 3]
        self.next_x, self.next_y = next_wp[0], next_wp[1]

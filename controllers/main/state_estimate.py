'''State estimator class'''
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

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
        self.get_state()

    def get_state(self):
        '''get positional data from "GPS" on robot'''
        self.current_gps_center = self.center.getValues()[:2]
        self.current_gps_front = self.front.getValues()[:2]

        d_f_x = self.current_gps_front[0] - self.current_gps_center[0]
        d_f_y = self.current_gps_front[1] - self.current_gps_center[1]

        self.current_heading = np.arctan2(d_f_y, d_f_x)

        if self.current_heading < 0 :
            self.current_heading = np.pi + (np.pi - abs(self.current_heading))

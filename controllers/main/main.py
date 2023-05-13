"""my_controller controller."""
from controller import Robot

from state_estimate import StateEstimator
from path_track import PathTracker
from pure_pursuit import PurePursuit

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

        # Find where we want to go based on the state estimation and the path
        tracker = PathTracker(gps_center)
        next_x_waypoint, next_y_waypoint = tracker.next_x, tracker.next_y

        # Find control actuations based on where we want to go
        controller = PurePursuit(current_heading,  next_x_waypoint, next_y_waypoint, gps_center)
        left_speed = controller.left_velocity
        right_speed = controller.right_velocity

        # Send the actuation
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    main()

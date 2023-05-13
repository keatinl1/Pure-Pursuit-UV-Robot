# Pure-Pursuit-UV-Robot 

Pure pursuit control of a UV sanitising robot made in WeBots, an open source robot simulator (https://cyberbotics.com/)

Watch the video below and scroll further for an explanation of the algorithm :)

## Demo

https://user-images.githubusercontent.com/87669488/236628705-70e88df3-e2e5-44bc-a3dc-9408b00b6bcc.mp4

$~~~~~~~~~~$

## Explanation

Pure pursuit is a geometric controller meaning it uses geometric properties like position, velocity etc. to decide actuations. In our case here, we use position as our property.

The robot has a lookahead circle around it with a user defined radius. We also have a set of postional waypoints which we want our robot to follow. See figure 1

<p align="center">
  <kbd>
    <img src="https://raw.githubusercontent.com/keatinl1/Pure-Pursuit-UV-Robot/main/figures/1.png">
  </kbd>
</p>
<p align="center">
Figure 1
</p>

We get the current position of the robot, and find which waypoint from the set is closest. We then set the following waypoint in the sequence as our next waypoint of interest, see figure 2.

<p align="center">
  <kbd>
    <img width=256 height=256 src="https://raw.githubusercontent.com/keatinl1/Pure-Pursuit-UV-Robot/main/figures/2.png">
  </kbd>
</p>
<p align="center">
Figure 2
</p>

Then we draw a line from our closest waypoint to the next waypoint of interest. Wherever this line intersects our lookahead circle is our goal point see figure 3.

<p align="center">
  <kbd>
    <img width=256 height=256 src="https://raw.githubusercontent.com/keatinl1/Pure-Pursuit-UV-Robot/main/figures/3.png">
  </kbd>
</p>
<p align="center">
Figure 3
</p>

In figure 4, the desired heading is the angle between our position and the goal point it is denoted as θ, the current heading is θ + φ (these angles are defined wrt a global coordinate system). The heading is corrected by adjusting the velocities of the wheels.

<p align="center">
  <kbd>
    <img width=256 height=256 src="https://raw.githubusercontent.com/keatinl1/Pure-Pursuit-UV-Robot/main/figures/4.png">
  </kbd>
</p>
<p align="center">
Figure 4
</p>

When this is done iteratively the controller can follow a set of waypoints.

$~~~~~~~~~~$

#### "Why not just find the angle between our position and the next waypoint?"

As the distance to the next waypoint would not always be consistent our steering would also be inconsistent. Having the lookahead circle allows you to "tune" how tightly your controller turns so your robot would behave predictably.

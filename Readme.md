
# Project Title :

Astar algorithm implemented for a differential drive robot.

# Authors:
1. Badrinarayanan Raghunathan Srikumar - braghuna (119215418)
2. Vyshnav Achuthan - vyachu07 (119304815)


# Dependencies Used:

rospy

math

numpy

pygame

nav_msgs

geometyry_msgs

# Steps to run the code for part1:
1. For part 1, unzip the package and navigate to the part 1 folder
2. Open a terminal in that location and run "python3 Astar_diffdrive.py"
3. Make sure that RPM values don't exceed 4 or 5, since, the size of the robot is huge and the wheel radius is also big, in comparison to the scale of the 2d map.
4. Example input set : 
  
  start - 50,100
  
  goal - 550,100
  
  RPM - 2,3 

# Steps to run the code for part 2:
1. For part 1, unzip the package and navigate to the part 2 folder
2. Make sure to comment out line number 274 in Astar_diffdrive.py file to avoid 2d visualization of the same map.
3. Navigate to the launch folder insode the part 2 folder and run "roslaunch turtlebot3_world.launch"

# Default values used for Part 2:
1. Goal : 550,100
2. Start : 50,100
3. RPM : 2,3

# Video links:
1. Part -1 : https://youtu.be/OeerggyOWTU
2. Part -2 : https://youtu.be/PJQnQ-o_vMg

# Github link :
https://github.com/Irdab2000/Astar-diffdrive-robot






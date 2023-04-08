Dependencies used :
    pygame
    numpy
    heapq
    math

- There are 5 python files in the src folder. Astar_diffdrive.py is for part 1 of Phase 2. It uses obstacle_gen.py to get the map.
- Astar_diffdrive_gazebo.py is for part 2 of Phase 2. It uses obstacle_gen_gazebo.py to get the map for part 2.
- control.py has the code to publish velocity to gazebo for turtlebot3.

- To run part 1 : Naviaget to src folder in the package, and run "python3 Astar_diffdrive.py"
- To run part 2 : In a terminal, run "roslaunch Astar-diffdrive-robot turtlebot3_world.launch"


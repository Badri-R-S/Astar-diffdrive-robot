#!/usr/bin/python3
"""
#########################################################################################
Authors : Vyshnav Achuthan (119304815)
          Badrinarayan (119215418)
#########################################################################################
"""
import math
import heapq
from obstacle_gen import obs_coord
import pygame
import numpy as np
import rospy
import tf
import time
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point

# rospy.init_node("astar_rigid")
# pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
# start_time = time.time()
# Euclidean distance
# euc = 0.5
# Theta threshold
# theta_thresh = 30
# Define the robot radius, clearance, tolerance, and goal tolerance
ROBOT_RADIUS = 0.105
ROBOT_WHEEL_RADIUS = 0.033
ROBOT_WHEEL_DIST = 0.160
OBSTACLE_CLEARANCE = 5
VISITED_TOLERANCE = 0.5
ORIENTATION_TOLERANCE = 30
GOAL_TOLERANCE = 1.5
MAP_WIDTH = 599
MAP_HEIGHT = 249
heap = []   #Open list
visited = np.zeros((1200,500,12)) #Closed list
# velocity_msg = Twist()
# tf_listener = tf.TransformListener()
# parent_frame = 'odom'
# child_frame = 'base_footprint'
# rate = rospy.Rate(100)
# global RPM1, RPM2

class Node:
    def __init__(self):
        self.state = None
        self.parent = None
        self.cost_to_come = float('inf')
        self.cost_to_goal = float('inf')
       
    def __lt__(self, other):
        return (self.cost_to_goal + self.cost_to_come) < (other.cost_to_goal + other.cost_to_come)

#Checking if given coordinates is in any obstacle
def is_obstacle(x,y):
    ind=obs_coord(x,y)
    if(ind == 1):
        #print("On obstacle")
        return True
    else:
        ind1 = obs_coord(x+ROBOT_RADIUS,y)
        ind2 = obs_coord(x,y+ROBOT_RADIUS)
        ind3 = obs_coord(x-ROBOT_RADIUS,y)
        ind4 = obs_coord(x,y-ROBOT_RADIUS)
        if(ind1 or ind2 or ind3 or ind4 == 1):
            return True
        else:
            return False

# Define a function to check if a point has been visited
def is_visited(node, visited):
    
    if visited[int(node.state[0]*2)][int(node.state[1]*2)][int(node.state[2]/30)]==1:
        return True
    return False

# Define a function to check if the goal has been reached
def reached_goal(node, goal):
    dx = node.state[0] - goal[0]
    dy = node.state[1] - goal[1]
    dist_to_goal = math.sqrt(dx**2 + dy**2)
    if(node.state[2] == goal[2]):
        return (dist_to_goal <= GOAL_TOLERANCE)
    else:
        return False
# Define get_cost function
def get_cost(current_node, neighbor_node):
    return math.sqrt((current_node.x - neighbor_node.x)**2 + (current_node.y - neighbor_node.y)**2)

# Define is_valid_node function
def is_valid_node(node):
    if(node.state[0]> MAP_WIDTH or node.state[1]>MAP_HEIGHT):
        return False
    else:
        if(is_obstacle(node.state[0],node.state[1])):
            return False
        return True

#Normalizing each coordiate
def normalize(val):
    return (val*2)/2

def next_node(node,Ul,Ur):
    t = 0
    dt = 0.1
    X=node.state[0]
    Y=node.state[1]
    Theta = math.pi * node.state[2] / 180
    Dist=0
    while t<1:
        t = t+dt
        dx = 0.5*ROBOT_RADIUS * (Ul + Ur) * math.cos(Theta) * dt
        dy = 0.5*ROBOT_RADIUS * (Ul + Ur) * math.sin(Theta) * dt
        Theta += (ROBOT_RADIUS/ROBOT_WHEEL_DIST)*(Ur-Ul) * dt
        X += dx
        Y += dy
        Dist += math.sqrt(math.pow((0.5*ROBOT_RADIUS * (Ul + Ur) * math.cos(Theta) *
                dt),2)+math.pow((0.5*ROBOT_RADIUS * (Ul + Ur) * math.sin(Theta) * dt),2))
    Thetafin = 180 * (Theta)/math.pi
    return X,Y,Thetafin,Dist

#Function to move forward
def move_forward(node,goal,actions):
    action = actions[0]
    new_x,new_y,new_theta,dist = next_node(node,action[0],action[1])
    if new_theta<0:
        new_theta +=360
    new_theta %= 360
    new_x = normalize(new_x)
    new_y = normalize(new_y)
    new_cost_to_come = node.cost_to_come + dist
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move forward")

#Fuction to move 30 degrees to the left
def turn_left_30(node,goal,step):
    new_x = normalize(node.state[0] + step*math.cos(math.radians(node.state[2])+math.pi/6))
    new_y = normalize(node.state[1] + step*math.sin(math.radians(node.state[2])+math.pi/6))
    new_theta  = node.state[2] + 30
    if new_theta<0:
        new_theta +=360
    new_theta %= 360
    new_cost_to_come = node.cost_to_come + step
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move 30 degrees to the left")

#Function to move 30 degrees to the right
def turn_right_30(node,goal,step):
    new_x = normalize(node.state[0] + step*math.cos(math.radians(node.state[2])+math.pi/6))
    new_y = normalize(node.state[1] - step*math.sin(math.radians(node.state[2])+math.pi/6))
    new_theta  = node.state[2] - 30
    if new_theta<0:
        new_theta +=360
    new_theta %= 360
    new_cost_to_come = node.cost_to_come + step
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move 30 degrees to the right")

#Function to move 60 degrees to the left
def turn_left_60(node,goal,step):
    new_x = normalize(node.state[0] + step*math.cos(math.radians(node.state[2])+math.pi/3))
    new_y = normalize(node.state[1] + step*math.sin(math.radians(node.state[2])+math.pi/3))
    new_theta  = node.state[2] + 60
    if new_theta<0:
        new_theta +=360
    new_theta %= 360
    new_cost_to_come = node.cost_to_come + step
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move 60 degrees to the left")

#Function to move 60 degrees to the right
def turn_right_60(node,goal,step):
    new_x = normalize(node.state[0] + step*math.cos(math.radians(node.state[2])+math.pi/3))
    new_y = normalize(node.state[1] - step*math.sin(math.radians(node.state[2])+math.pi/3))
    new_theta  = node.state[2] - 60
    if new_theta<0:
        new_theta +=360
    new_theta %= 360
    new_cost_to_come = node.cost_to_come + step
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move 60 degrees to the right")

#Backtracking
def backtrack(node):
    path = []
    path.append(node.state)
    print(node.state)
    while node.parent is not None:
        node = node.parent
        path.append(node.state)
        print(node.state)
    print("Done backtracking")
    # visualize(path)
    return path

#Astra logic using priority queue
def astar(start, goal,actions):
    heapq.heappush(heap, (start))
    while heap:
        curr_node = heapq.heappop(heap)
        print("Searching :",curr_node.state)
        if is_valid_node(curr_node):
        
            if reached_goal(curr_node, goal):
                print("Goal reached, Backtracking")
                backtrack(curr_node)
                break                           #Backtracking
        
            if is_visited(curr_node, visited):
                continue                            #Checking if node is already visited
        
            visited[int(curr_node.state[0]*2)][int(curr_node.state[1]*2)][int(curr_node.state[2]/30)] = 1
        
            move_forward(curr_node,goal,actions)
            turn_left_30(curr_node,goal,actions)
            turn_right_30(curr_node,goal,actions)
            turn_left_60(curr_node,goal,actions)
            turn_right_60(curr_node,goal,actions)      
    return None

# def go_straight(distance_to_drive, linear_velocity):
#     # get distance and linear velocity from command line
#     global velocity_msg
#     # update linear.x from the command line
#     velocity_msg.angular.z = 0.0
#     velocity_msg.linear.x = linear_velocity
#     # get the current time (s)
#     t_0 = rospy.Time.now().to_sec()
#     # keep track of the distance
#     distance_moved = 0.0

#     # while the amount of distance has not been reached
#     while distance_moved <= distance_to_drive:
#         # rospy.loginfo("TurtleBot is moving")
#         pub.publish(velocity_msg)
#         rate.sleep()
#         # time in sec in the loop
#         t_1 = rospy.Time.now().to_sec()
#         distance_moved = (t_1 - t_0) * abs(linear_velocity)
#         # rospy.loginfo("distance moved: {d}".format(d=distance_moved))

#     # rospy.logwarn("Distance reached")
#     # finally, stop the robot when the distance is moved
#     velocity_msg.linear.x = 0.0
#     velocity_msg.angular.z = 0.0
#     pub.publish(velocity_msg)


# def rotate(angle, angular_velocity):
#     """Make the robot rotate in place

#     The angular velocity is modified before publishing the message on the topic /cmd_vel.
#     """
#     print("Rotating")
#     # angular_velocity = math.radians(angular_velocity)
#     velocity_msg.linear.x = 0.0
#     velocity_msg.angular.z = angular_velocity
#     t0 = rospy.Time.now().to_sec()
#     while True:
#         # rospy.loginfo("TurtleBot is rotating")
#         pub.publish(velocity_msg)
#         rate.sleep()
#         t1 = rospy.Time.now().to_sec()
#         # rospy.loginfo("t0: {t}".format(t=t0))
#         # rospy.loginfo("t1: {t}".format(t=t1))
#         current_angle_degree = (t1 - t0) * angular_velocity
#         # current_angle_degree = current_angle_degree*180/math.pi
#         # rospy.loginfo("current angle: {a}".format(a=current_angle_degree))
#         # rospy.loginfo("angle to reach: {a}".format(a=angle))
#         if abs(current_angle_degree) >= math.radians(abs(angle)):
#             # rospy.loginfo("reached")
#             break
#     # finally, stop the robot when the distance is moved
#     velocity_msg.angular.z = 0
#     pub.publish(velocity_msg)


# def get_odom_data():
#     """Get the current pose of the robot from the /odom topic

#     Return
#     ----------
#     The position (x, y, z) and the yaw of the robot.
#     """
#     try:
#         (trans, rot) = tf_listener.lookupTransform(
#             parent_frame, child_frame, rospy.Time(0))
#         # rotation is a list [r, p, y]
#         rotation = euler_from_quaternion(rot)
#     except (tf.Exception, tf.ConnectivityException, tf.LookupException):
#         rospy.loginfo("TF Exception")
#         return
#     # return the position (x, y, z) and the yaw
#     return Point(*trans), rotation[2]


# def go_to_goal(current_node, next_node):
#     (pos, rot) = get_odom_data()
#     print(rot)
#     x_start = current_node[0]
#     y_start = current_node[1]
#     rotation = current_node[2]
#     goal_x = next_node[0]
#     goal_y = next_node[1]
#     goal_theta = next_node[2]
#     distance_to_drive_x = abs((goal_x - x_start) / 100)
#     print("x :", distance_to_drive_x)
#     distance_to_drive_y = abs((goal_y - y_start) / 100)
#     print("y :", distance_to_drive_y)
#     # angle_to_goal = goal_theta - rotation
#     angle_to_goal = 0
#     if x_start == goal_x and y_start < goal_y and (1.0 < rot < 2.5):
#         angle_to_goal = 0
#         go_straight(distance_to_drive_y, 0.2)
#     elif x_start == goal_x and y_start == goal_y:
#         pass
#     elif x_start == goal_x and y_start > goal_y and (-1.8 < rot < -1.4):
#         angle_to_goal = 0
#         go_straight(distance_to_drive_y, 0.2)
#     elif x_start == goal_x and y_start < goal_y and (-0.1 < rot < 1.4):
#         angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
#         angle_to_goal = angle_to_goal*180/math.pi
#         rotate(angle_to_goal, 0.2)
#         go_straight(distance_to_drive_y, 0.2)
#     elif x_start == goal_x and y_start > goal_y and (-0.45 < rot < 1.4):
#         angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
#         angle_to_goal = angle_to_goal*180/math.pi
#         print("hello")
#         rotate(angle_to_goal, -0.2)
#         go_straight(distance_to_drive_y, 0.2)
#     elif x_start == goal_x and y_start < goal_y and (rot > 2.5 or rot < -2.5):
#         angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
#         angle_to_goal = angle_to_goal*180/math.pi
#         print("world")
#         rotate(angle_to_goal, -0.2)
#         go_straight(distance_to_drive_y, 0.2)
#     elif x_start == goal_x and y_start > goal_y and (rot > 2.5 or rot < -2.5):
#         angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
#         angle_to_goal = angle_to_goal*180/math.pi
#         rotate(angle_to_goal, 0.2)
#         go_straight(distance_to_drive_y, 0.2)
#     elif x_start < goal_x and y_start == goal_y and (-0.1 < rot < 0.1):
#         angle_to_goal = 0
#         go_straight(distance_to_drive_x, 0.2)
#     elif x_start > goal_x and y_start == goal_y and (1.0 < rot < 2.5):
#         # angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
#         # angle_to_goal = angle_to_goal*180/math.pi
#         angle_to_goal = 90
#         rotate(angle_to_goal, 0.2)
#         go_straight(distance_to_drive_x, 0.2)
#     elif x_start < goal_x and y_start == goal_y and (1.0 < rot < 2.5):
#         # angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
#         # angle_to_goal = angle_to_goal*180/math.pi
#         angle_to_goal = 90
#         print("nice")
#         rotate(angle_to_goal, -0.2)
#         go_straight(distance_to_drive_x, 0.2)
#     elif x_start < goal_x and y_start == goal_y and (-1.8 < rot < -1.4):
#         # angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
#         # angle_to_goal = angle_to_goal*180/math.pi
#         angle_to_goal = 90
#         rotate(angle_to_goal, 0.2)
#         go_straight(distance_to_drive_x, 0.2)
#     elif x_start > goal_x and y_start == goal_y and (-1.8 < rot < -1.0):
#         # angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
#         # angle_to_goal = angle_to_goal*180/math.pi
#         angle_to_goal = 90
#         print("to")
#         rotate(angle_to_goal, -0.2)
#         go_straight(distance_to_drive_x, 0.2)
#     elif x_start < goal_x and y_start == goal_y and (-2.0 < rot < -1.0):
#         # angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
#         # angle_to_goal = angle_to_goal*180/math.pi
#         angle_to_goal = 90
#         rotate(angle_to_goal, 0.2)
#         go_straight(distance_to_drive_x, 0.2)
#     elif x_start < goal_x and y_start < goal_y and (-0.1 < rot < 0.1):
#         angle_to_goal = 45
#         rotate(angle_to_goal, 0.2)
#         go_straight(distance_to_drive_x, 0.2)
#     elif x_start < goal_x and y_start < goal_y and (1.4 < rot < 1.8):
#         angle_to_goal = 45
#         print("meet")
#         rotate(angle_to_goal, -0.2)
#         distance_to_drive = np.sqrt(distance_to_drive_y ** 2 + distance_to_drive_x **2)
#         go_straight(distance_to_drive, 0.2)
#     elif x_start < goal_x and y_start < goal_y and (0.35 < rot < 1.3):
#         distance_to_drive = np.sqrt(distance_to_drive_y ** 2 + distance_to_drive_x **2)
#         go_straight(distance_to_drive, 0.2)
#     elif x_start < goal_x and y_start == goal_y and (0.35 < rot < 1.3):
#         angle_to_goal = 45
#         rotate(angle_to_goal, -0.2)
#         go_straight(distance_to_drive_x, 0.2)
#         time.sleep(1)
def visualize(path_gen): #Function to visualize the graph
    pygame.init()

    # Set the window dimensions
    WINDOW_WIDTH = 600
    WINDOW_HEIGHT = 250

    # Create the Pygame window
    window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Obstacle Course")

    # Define the colors
    BACKGROUND_COLOR = pygame.Color("red")
    OBSTACLE_COLOR = pygame.Color("black")
    CLEARANCE_COLOR = pygame.Color("white")
    VISITED_COLOR = pygame.Color("green")
    PATH_COLOR = pygame.Color("blue")

    # Create the surface for the obstacle course
    surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))
    # pygame.display.flip()

    # Fill the surface with the background color
    surface.fill(BACKGROUND_COLOR)
    pygame.draw.rect(surface, CLEARANCE_COLOR, (0,0,5,250))
    pygame.draw.rect(surface, CLEARANCE_COLOR, (595,0,5,250))
    pygame.draw.rect(surface, CLEARANCE_COLOR, (0,0,600,5))
    pygame.draw.rect(surface, CLEARANCE_COLOR, (0,245,600,5))

    pygame.draw.rect(surface, CLEARANCE_COLOR, (0,0,5,250))
    pygame.draw.rect(surface, CLEARANCE_COLOR, (100-5, 145-5,50+10 ,100+10))
    pygame.draw.polygon(surface, CLEARANCE_COLOR, ((300,200+5),(365+4,162),(365+4,87),(300,50-5),(235-4,87),(235-4,162))) 
    pygame.draw.rect(surface, CLEARANCE_COLOR, (100-5,5-5,50+10,100+10))                                                                #Printing directly using the coordinates for visualization.
    pygame.draw.polygon(surface, CLEARANCE_COLOR, ((460-3,225+12),(460-3,25-12),(510+5,125)))
    # Draw the obstacles on the surface
    pygame.draw.rect(surface, OBSTACLE_COLOR, (100, 145,50 ,100))  
    pygame.draw.polygon(surface, OBSTACLE_COLOR, ((300,200),(365,162),(365,87),(300,50),(235,87),(235,162)))
    pygame.draw.rect(surface, OBSTACLE_COLOR, (100,5,50,100))
    pygame.draw.polygon(surface, OBSTACLE_COLOR, ((460,225),(460,25),(510,125)))   

    for idx,any in enumerate(heap):
        pygame.draw.rect(surface,VISITED_COLOR,(any.state[0],any.state[1],1,1))
        window.blit(surface,(0,0))
        # pygame.display.flip()
        pygame.display.update()
    
    for idx, every in enumerate(path_gen):
        pygame.draw.rect(surface,PATH_COLOR,(every[0],every[1],5,5))
        # pygame.display.flip()
    # Blit the updated surface onto the Pygame window
        window.blit(surface, (0, 0))
    # Update the Pygame window display
        pygame.display.update()
    # Wait for a short time to show the current coordinate
        pygame.time.wait(2)

    
    # Blit the surface onto the Pygame window
        window.blit(surface, (0, 0))

    # Run the game loop
    pygame.display.flip()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        pygame.display.update()


    # Quit Pygame
    pygame.quit()

def get_startcoord_input(): #function to get start coordinates from user
    flag = False
    x = int(input("Enter x-ccordinate of start position:"))
    y = int(input("Enter y coordinate of start position:"))
    theta = int(input("Enter start orientation in multiples of 30:"))
    y = 249-y
    if(theta%30 !=0):
        print("orientation should be in multiples of 30")
        return [flag]
    if(x<0 or x>=600 or y<0 or y>=250):
        print("Start coordinates out of bounds, Please enter x and y coordinates again")
        return [flag]
    elif (is_obstacle(x,y)):
        print("Start coordinates on an obstacle, Please enter x and y coordinates again")
        return [flag]
    else:
        return [True,x,y,theta]
def get_goalcoord_input(): #Function to get goal coordinates from user
    flag = False
    xg = int(input("Enter x-ccordinate of goal position:"))
    yg = int(input("Enter y coordinate of goal position:"))
    theta_g =int(input("Enter goal orientation in multiples of 30"))
    yg = 249-yg
    if(theta_g%30 !=0):
        print("orientation should be in multiples of 30")
        return [flag]
    if(xg<0 or xg>=600 or yg<0 or yg>=250):
        print("Goal coordinates out of bounds, Please enter x and y coordinates again")
        return [flag]
    elif(is_obstacle(xg,yg)):
        print("goal coordinates on an obstacle, Please enter x and y coordinates again")
        return [flag]
    else:
        return [True,xg,yg,theta_g]

if __name__ == "__main__":
    node = Node()
    a = True
    start_input = []
    while(a not in start_input):
        start_input = get_startcoord_input()    
    node.state = [start_input[1],start_input[2],start_input[3]]
    goal_input = []
    while(a not in goal_input):
        goal_input = get_goalcoord_input()
    goal = [goal_input[1],goal_input[2],goal_input[3]]
    node.parent = None
    node.cost_to_come = 0
    node.cost_to_goal = 100000
    step = int(input("Enter step between 1 and 10"))
    RPM1 = int(input("Enter lower limit RPM"))
    RPM2 = int(input("Enter higher limit RPM"))
    actions = [[0,RPM1],[RPM1,0],[RPM1,RPM2],[0,RPM2],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]
    astar(node,goal,actions)
    # shortest_path=backtrack(node)
    # for i in range(len(shortest_path) - 1):
    #     if i == 0:
    #         continue
    #     go_to_goal(shortest_path[i], shortest_path[i + 1])
    #     # time.sleep(1)
    # rospy.logwarn("Reached Goal!!")
    # vel = Twist()
    # vel.linear.x = 0.0
    # vel.angular.z = 0.0
    # pub.publish(vel)
    
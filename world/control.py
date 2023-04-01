
import rospy
import tf
from reimp import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


rospy.init_node('controller')

listener = tf.TransformListener()
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('posex', Point, queue_size=10)

def pause():
    listener.waitForTransform('/odom', '/base_footprint',
                                            rospy.Time(), rospy.Duration(500))

def cmd_vel(linear_vel_x,linear_vel_y, angular_vel,x,y):
    vel = Twist()
#     while(x)
    vel.linear.x = linear_vel_x
#     rospy.sleep(1)

    vel.linear.y = linear_vel_y
#     rospy.sleep(1)

    vel.angular.z = angular_vel
#     rospy.sleep(1)
    velocity_publisher.publish(vel)
    

def main():
    x_s, y_s, theta_start = 50, 100, 0

    x_g, y_g = 150, 200
    start_pos = Node()
    start_pos.state = [x_s,y_s,theta_start]
    start_pos.parent = None
    start_pos.cost_to_come = 0
    start_pos.cost_to_goal = 1000000
    goal_pos = (x_g, y_g)
    rpm1, rpm2 = 20, 30

    all_actions = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), (rpm2, 0), (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1)]

    path, explore = astar(start_pos, goal_pos,all_actions)


    print('\n path found')
    print('\n running')
    rospy.sleep(10)

    pause()
    rate = rospy.Rate(1)

    r = 0.038 #in metres
    L = 0.354 #in metres
    dt = 10
    pi = math.pi

    for node in path[1:]:
        if(node.actions=='LPR'):
                UL = 0
                UR = rpm1
        elif(node.actions=='RPR'):
                UL = rpm1
                UR = 0
        elif(node.actions=='MF'):
                UL = rpm1
                UR = rpm1
        elif(node.actions=='LPR2'):
                UL = 0
                UR = rpm2
        elif(node.actions=='RPR2'):
                UL = rpm2
                UR = 0
        elif(node.actions=='MF2'):
                UL = rpm2
                UR = rpm2
        elif(node.actions=='MRP1P2'):
                UL = rpm1
                UR = rpm2
        elif(node.actions=='MRP1P2'):
                UL = rpm2
                UR = rpm1
            
        x, y, theta = node.state[0],node.state[1],math.pi * node.state[2] / 180               
        theta_dot = (r / L) * (UR - UL) 
        velocity_value_x = (r / 2) * (UL + UR)*math.cos(theta)
        velocity_value_y = (r / 2) * (UL + UR)*math.sin(theta)

        cmd_vel(velocity_value_x,velocity_value_y, theta_dot,x,y)
        rate.sleep(1)

    cmd_vel(0, 0)

main()

if __name__ == '_main_':
    try:
       
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task terminated.")
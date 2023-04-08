
import rospy
import tf
from Astar_diffdrive import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SubscriberOdom:
        def __init__(self):
            self.x = None
            self.y = None
            self.z = None
            self.subscriber = rospy.Subscriber("odom",Odometry,self.odom_callback)
        def odom_callback(self,data):
            self.x = data.pose.pose.position.x
            self.y = data.pose.pose.position.y
            self.z = data.pose.pose.orientation.z


def pause(listener):
    listener.waitForTransform('/odom', '/base_footprint',
                                            rospy.Time(), rospy.Duration(500))

def cmd_vel(linear_vel,angular_vel,target_pos_x,target_pos_y,target_theta,odom_obj,velocity_publisher):
      vel = Twist()
#     while(x)
#     rospy.sleep(1)
      x = odom_obj.x
      y = odom_obj.y
      ang_z = odom_obj.z
      print("x = ",x)
      print("y = ",y)
      print("ori = ",ang_z)
      while(ang_z - target_theta<0.1):
        vel.angular.z = angular_vel
        velocity_publisher.publish(vel)
      vel.angular.z = 0
      velocity_publisher.publish(vel)
      while True:
        x = odom_obj.x
        y = odom_obj.y       
        dist = math.sqrt((target_pos_x - x)**2 + (target_pos_y - y)**2)
        if dist < 0.1 :
              vel.linear.x = 0
              vel.linear.y = 0
              velocity_publisher.publish(vel)
              return
        else:
             vel.linear.x = linear_vel
             velocity_publisher.publish(vel)

def main():
    rospy.init_node('controller')
    listener = tf.TransformListener()
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
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

    pause(listener)
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
        odom_obj = SubscriberOdom()
        rospy.spin()   
        x, y, theta = node.state[0],node.state[1],math.pi * node.state[2] / 180               
        theta_dot = (r / L) * (UR - UL) 
        velocity_value_linear = (r / 2) * (UL + UR)
        cmd_vel(velocity_value_linear, theta_dot,x,y,theta,odom_obj,velocity_publisher)
        rate.sleep(1)

    cmd_vel(0, 0)

main()

if __name__ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Task terminated.")
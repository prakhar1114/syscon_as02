#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy
import math
from math import cos, sin, atan2
# #!/usr/bin/env python
# import rospy
from nav_msgs.msg import Odometry
from quat2euler import quat2euler


## Define global variables
pose = []


## This function will give access to robots current position in `pose` variable
def callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]


def Waypoints(t):
    x  = 3*cos(t)-3
    y  = 3*sin(2*t)
    return [x,y] 


## This is where we will calculate error and apply the proportional control to 
##  compute linear velocity and angular velocity for the turtlebot 
def control_loop():
    rospy.init_node('turtlebot_trajectory_tracker')

    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, callback)

        ####
    r=Waypoints(rospy.get_time())
    xDes=r[0]
    yDes=r[1]

    curX=pose[0]
    curY=pose[1]

    E_theta=atan2(yDes,xDes)-pose[2]
    E_pos=((xDes-curX)**2+(yDes-curY)**2)**0.5
    kp_theta=10
    kp_pos=1
    # ####    
    
    rate = rospy.Rate(2) 
    velocity_msg = Twist()
    # velocity_msg.linear.x = 0.2
    # velocity_msg.angular.z = 0.2
    velocity_msg.linear.x = kp_pos*E_pos
    velocity_msg.angular.z = kp_theta*E_theta
    while not rospy.is_shutdown():
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))

        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

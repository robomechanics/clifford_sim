#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
import numpy

class controller():
    def __init__(self):
        # Create a node for the controller
        rospy.init_node('avg_vel',anonymous=True)
        # Publisher to drive the robot
        self.vx_arr = [] # array of velocity in x
        self.vy_arr = [] # array of velocity in x
        self.counter = 0
        # Subscriber to read the current pose (states) of the robot
        self.pose_sub = rospy.Subscriber('/odom',Odometry, self.control_update)
        rospy.spin()

    def control_update(self,odom_data):
        # read current vehicle states from mocap
        twist_odom = Twist()
        twist_odom = odom_data.twist.twist
        xdot = twist_odom.linear.x
        ydot = twist_odom.linear.y
        self.vx_arr.append(xdot)
        self.vy_arr.append(ydot)
        vx_arr2 = np.asarray(self.vx_arr)
        vy_arr2 = np.asarray(self.vy_arr)
        vy_avg = np.average(vy_arr2)
        vx_avg = np.average(vx_arr2)
        print('Average vx: ' + str(vx_avg))
        print('Average vy: ' + str(vy_avg))
        np.savetxt("/home/akshit/clifford_sim_ws/src/clifford_sim/data/vy_PI_ground_025.csv",vy_arr2,delimiter=",")
def main():
    controller()
    

if __name__== "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
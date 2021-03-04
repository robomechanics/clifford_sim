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
        rospy.init_node('stop_clifford',anonymous=True)
        # Publisher to drive the robot
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub_mod = rospy.Publisher('cmd_mode', Int8, queue_size=10)
        self.prev_xdot_error = 0.0
        self.prev_ydot_error = 0.0
        self.intgeral_xdot_error = 0.0
        self.intgeral_ydot_error = 0.0
        self.xdot_des = 0.0
        self.ydot_des = 0.0
        self.mode = 0
        # Subscriber to read the current pose (states) of the robot
        #rospy.on_shutdown(self.control_end)
        self.pose_sub = rospy.Subscriber('/odom',Odometry, self.control_update)
        rospy.spin()

    def control_end(self):
        print("Shutdown Procedure Initiated")
        #self.xdot_des = 0.0
        #self.ydot_des = 0.0
        # Publish commands to stop the robot
        #linear = Vector3(0, 0.0,0.0)
        #angular = Vector3(0.0,0.0,0)
        #twist = Twist(linear,angular)
        #self.pub_cmd.publish(twist)


    def control_update(self,odom_data):
        # read current vehicle states from mocap
        twist_odom = Twist()
        twist_odom = odom_data.twist.twist
        xdot = twist_odom.linear.x
        ydot = twist_odom.linear.y
        #print("Previous Error " + str(self.prev_xdot_error))
        # Desired lineat velocities (m/s)
        delT = 0.05 # time in seconds
        # ---------------|Longitudnal Controller|-------------------------
        kp_x = 1#3.60
        ki_x = 0*0.69
        kd_x = 0*-0.75
        xdot_error = self.xdot_des - xdot
        self.intgeral_xdot_error += xdot_error
        derivative_error_x = xdot_error - self.prev_xdot_error
        u_x = kp_x*xdot_error + ki_x*self.intgeral_xdot_error*delT + kd_x*(derivative_error_x/delT) # Input for x direction

        # ---------------|Lateral Controller|-------------------------
        kp_y = 1#3.60
        ki_y = 0*0.69
        kd_y = 0*-0.75
        ydot_error = self.ydot_des - ydot
        self.intgeral_ydot_error += ydot_error
        derivative_error_y = ydot_error - self.prev_ydot_error
        u_y = kp_y*ydot_error + ki_y*self.intgeral_ydot_error*delT + kd_y*(derivative_error_y/delT) # Input for y direction

        # Map inputs to vehicle parameters
        F_in = np.sqrt(u_x**2 + u_y**2) # throttle
        steer_in = np.arctan2(u_y, u_x) # steering input in radians

        # Storing the parameters for the next epoch
        self.prev_xdot_error = xdot_error
        self.prev_ydot_error = ydot_error

        # Publish commands to the robot
        linear = Vector3(0, 0.0,0.0)
        angular = Vector3(0.0,0.0,0)
        twist = Twist(linear,angular)
        self.pub_cmd.publish(twist)
        self.pub_mod.publish(self.mode)

def main():
    controller()

if __name__== "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
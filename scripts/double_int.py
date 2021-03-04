#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class controller():
    def __init__(self):
        # Create a node for the controller
        rospy.init_node('double_int',anonymous=True)
        # Publisher to drive the robot
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        self.pub_mod = rospy.Publisher('cmd_mode', Int8, queue_size=100)
        #self.pub_des = rospy.Publisher('des_vel', Float8, queue_size=100)
        self.prev_xdot_error = 0.0
        self.prev_ydot_error = 0.0
        self.intgeral_xdot_error = 0.0
        self.intgeral_ydot_error = 0.0
        self.xdot_des = 0.0
        self.ydot_des = 0.30
        self.mode = 2
        self.vx_arr = [] # array of velocity in x
        self.vy_arr = [] # array of velocity in x
        self.counter = 0
        # Subscriber to read the current pose (states) of the robot
        #rospy.on_shutdown(self.control_end)
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
        #print("Iteration: " + str(self.counter))
        self.counter +=1
        Array = np.array(self.vy_arr)
        
        # Displaying the array
        np.savetxt("/home/akshit/clifford_sim_ws/src/clifford_sim/data/PI_ramp_030.csv",Array,delimiter=",")
        #print("Previous Error " + str(self.prev_xdot_error))
        # Desired lineat velocities (m/s)
        delT = 0.05 # time in seconds
        # ---------------|Longitudnal Controller|-------------------------
        kp_x = 0.0#3.60
        ki_x = 0*0.69
        kd_x = 0*-0.75
        xdot_error = self.xdot_des - xdot
        self.intgeral_xdot_error += xdot_error
        derivative_error_x = xdot_error - self.prev_xdot_error
        u_x = 0.0 + kp_x*xdot_error + ki_x*self.intgeral_xdot_error*delT + kd_x*(derivative_error_x/delT) # Input for x direction

        # ---------------|Lateral Controller|-------------------------
        kp_y = 3.75#1.950
        ki_y = 0.25
        kd_y = 0*-0.75
        ydot_error = self.ydot_des - ydot
        self.intgeral_ydot_error += ydot_error
        derivative_error_y = ydot_error - self.prev_ydot_error
        u_y = kp_y*ydot_error + ki_y*self.intgeral_ydot_error*delT + kd_y*(derivative_error_y/delT) # Input for y direction

        # Map inputs to vehicle parameters
        F_in = np.sqrt(u_x**2 + u_y**2) # throttle
        steer_in = 1.0#np.arctan2(u_y, u_x) # steering input in radians

        # Storing the parameters for the next epoch
        self.prev_xdot_error = xdot_error
        self.prev_ydot_error = ydot_error

        # Publish commands to the robot
        linear = Vector3(F_in, 0.0,0.0)
        angular = Vector3(0.0,0.0,steer_in)

        # Compute roll, pitch, yaw from quaternion
        orientation = odom_data.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        #print("Throttle Input: " + str(F_in))
        #print("Steering Input: " + str(steer_in))
        #print("vy: "+str(ydot))
        print("Yaw: "+str(yaw))
        twist = Twist(linear,angular)
        dur = rospy.Duration(0.050)
        start = rospy.Time.now()
        while rospy.Time.now() - start < dur:
            self.pub_cmd.publish(twist)
            self.pub_mod.publish(self.mode)

def main():
    controller()
    

if __name__== "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
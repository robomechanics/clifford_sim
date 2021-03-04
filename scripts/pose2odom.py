#!/usr/bin/env python

import numpy as np
import rospy
import tf
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class pose2odom():
    def __init__(self):
        # Initialize the node
        rospy.init_node('pose2odom',anonymous=True)
        # Create a pulisher for the odom msg
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.10
        self.grad = 0 * (math.pi / 180.0) # gradient of the hill in radians
        self.prev_time = rospy.Time.now()
        self.pose_sub = rospy.Subscriber('/pose',PoseStamped, self.updateOdom)
        #self.pose_sub = rospy.Subscriber('/mocap_node/Robot_6/pose',PoseStamped, self.updateOdom)
        #self.joy_sub = rospy.Subscriber('/joy',Joy, self.JoyCallback)
        rospy.spin()
    
    def JoyCallback(self,msg):
        print("ji")
        # print(msg.axes[5])
    def updateOdom(self,pose_data):
        curr_time = rospy.Time.now()
        # Compute velocities
        # x is the longitudnal direction
        y = pose_data.pose.position.x # correcting for the frame of reference
        x = pose_data.pose.position.y
        dx =  (x - self.prev_x) / math.cos(self.grad)
        dy =  y - self.prev_y 
        yaw = math.atan2(y,x)
        dt = (curr_time - self.prev_time).to_sec() # convert time elapsed to seconds

        odom = Odometry()
        odom.header.stamp = curr_time
        odom.header.frame_id = "odom"
        # set the position and velocity
        vx = -dx/dt # negative sign to align with the ramp direction
        vy = -dy/dt
        yaw_vel = math.atan2(vy,vx)
        orientation = pose_data.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        print("Yaw: " + str(yaw))
        odom_quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        odom.pose.pose = Pose(Point(x,y,0), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        linear = Vector3(vx, vy, 0.0)
        angular = Vector3(0.0, 0.0, yaw_vel)
        odom.twist.twist = Twist(linear, angular)
        # Update values for next iteration
        self.prev_time = curr_time
        self.prev_x = x
        self.prev_y = y

        # Publish the msg
        self.odom_pub.publish(odom)
        #print ('vx: ' + str(vx))
        #print ('vy: ' + str(vy))
        print('Gradient Cosine: '+str(math.cos(self.grad)))
def main():
    pose2odom()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
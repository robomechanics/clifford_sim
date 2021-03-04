#!/usr/bin/env python

import numpy as np
import rospy
import tf
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point, Pose, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import numpy

class tf2odom():
    def __init__(self):
        # Initialize the node
        rospy.init_node('tf2odom',anonymous=True)
        # Create a pulisher for the odom msg
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.prev_xdot = 0.0
        self.prev_ydot = 0.0
        self.prev_yawd = 0.0 # angular rate
        self.grad = 0.0 * (math.pi / 180.0) # gradient of the hill in radians
        self.prev_time = rospy.Time.now()
        self.tf_sub = rospy.Subscriber('/imu/data', Imu, self.updateOdom)
        rospy.spin()
    
    def updateOdom(self,data):
        curr_time = rospy.Time.now()
        # Compute velocities
        acc_x = data.linear_acceleration.x
        acc_y = data.linear_acceleration.y
        dt = (curr_time - self.prev_time).to_sec() # convert time elapsed to seconds

        odom = Odometry()
        odom.header.stamp = curr_time
        odom.header.frame_id = "odom"
        # set the position and velocity
        vx = self.prev_xdot + acc_x*dt
        vy = self.prev_ydot + acc_y*dt
        yaw_vel = data.angular_velocity.z
        '''odom_quat = data.orientation
        point_odom = tf.transformations.euler_from_quaternion(data.orientation)
        odom.pose.pose = Pose(Point(point_odom), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        linear = Vector3(-vx, vy, 0.0)
        angular = Vector3(0.0, 0.0, yaw_vel)
        odom.twist.twist = Twist(linear, angular)'''

        # Update values for next iteration
        self.prev_time = curr_time
        self.prev_xdot = vx
        self.prev_ydot = vy

        # Publish the msg
        #self.odom_pub.publish(odom)
        print ('vx: ' + str(vx))
        print ('vy: ' + str(vy))
def main():
    tf2odom()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
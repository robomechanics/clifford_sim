#!/usr/bin/env python

import numpy as np
import rospy
import tf
import math
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point, Pose, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import numpy

class tf2odom():
    def __init__(self):
        # Initialize the node
        rospy.init_node('tf2odom',anonymous=True)
        # Create a pulisher for the odom msg
        self.odom_pub = rospy.Publisher("tf_odom", Odometry, queue_size=10)
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.10
        #self.grad = 0.0 * (math.pi / 180.0) # gradient of the hill in radians
        self.prev_time = rospy.Time.now()
        self.tf_sub = rospy.Subscriber('/tf_tag', TFMessage, self.updateOdom)
        #self.pose_sub = rospy.Subscriber('/pose',PoseStamped, self.updateOdom)
        rospy.spin()
    
    def updateOdom(self,tf_data):
        curr_time = rospy.Time.now()
        # Compute velocities
        x = tf_data.transforms[0].transform.translation.x
        y = tf_data.transforms[0].transform.translation.y
        #self.grad = 38.0 * (math.pi / 180.0)
        dx =  (x - self.prev_x) #/ math.cos(self.grad)
        dy =  y - self.prev_y 
        yaw = math.atan2(y,x)
        dt = (curr_time - self.prev_time).to_sec() # convert time elapsed to seconds

        odom = Odometry()
        odom.header.stamp = curr_time
        odom.header.frame_id = "odom"
        # set the position and velocity
        vx = dx/dt
        vy = dy/dt
        yaw_vel = math.atan2(vy,vx)
        odom_quat = tf.transformations.quaternion_from_euler(0,0,yaw)
        odom.pose.pose = Pose(Point(x,y,0), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        linear = Vector3(-vx, vy, 0.0)
        angular = Vector3(0.0, 0.0, yaw_vel)
        odom.twist.twist = Twist(linear, angular)
        # Update values for next iteration
        self.prev_time = curr_time
        self.prev_x = x
        self.prev_y = y

        # Publish the msg
        self.odom_pub.publish(odom)
        print ('vx: ' + str(vx))
        print ('vy: ' + str(vy))
def main():
    tf2odom()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
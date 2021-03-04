#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def publisher():
    pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(2) # Hz
    p = PoseStamped()
    p.pose.position.x = 0.5
    p.pose.position.y = -0.1
    p.pose.position.z = 0.0
    while not rospy.is_shutdown():
        p.pose.position.x += 0.01
        p.pose.position.y += 0.01
        p.pose.position.z *= 1.0
        
        # Make sure the quaternion is valid and normalized
        p.pose.orientation.x = 0.05
        p.pose.orientation.y = 0.05
        p.pose.orientation.z = 0.20
        p.pose.orientation.w = 1.0
        p.header.stamp = rospy.Time.now()
        print(p.pose.position.x)
        pub.publish(p)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass
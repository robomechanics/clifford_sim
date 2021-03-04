#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

import numpy

class Drive():
    def __init__(self):
        # Twist Publisher
        self.pub_cmd = rospy.Publisher('/cliffordDrive', Twist, queue_size=100)
        plan_file = "/home/akshit/clifford_sim_ws/src/clifford_sim/scripts/robotPath.txt"
        with open(plan_file) as f:
            plan = f.readlines()

        self.v_x = 0.00
        self.steer_angle = 0.00
        #print('Params Initialized \n')
        self.sendCommand(plan)

    def sendCommand(self, plan):
        for c in plan:
            cmd = c.split(",")
            assert len(cmd) == 2
            self.v_x, self.steer_angle = float(cmd[0]), float(cmd[1])

            linear = Vector3(self.v_x, 0.0, 0.0)
            angular = Vector3(0.0, 0.0, self.steer_angle)
            dur = rospy.Duration(0.30)
            rate = rospy.Rate(100)
            start = rospy.Time.now()
            twist = Twist(linear, angular)
            while rospy.Time.now() - start < dur: 
                #print('Publishing \n')
                self.pub_cmd.publish(twist)
                rate.sleep()

def main():
    rospy.init_node('cliffDriver')
    Drive()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

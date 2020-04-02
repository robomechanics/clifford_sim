import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties, SetPhysicsProperties
from std_srvs.srv import Empty
from geometry_msgs.msg import *
#from generateTerrain import GenerateSTL
import numpy as np
import time

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/get_world_properties")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/pause_physics")
    rospy.wait_for_service("gazebo/unpause_physics")
    print("Got it.")
    get_world_prop = rospy.ServiceProxy("gazebo/get_world_properties",GetWorldProperties)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model_prox = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    pause_world = rospy.ServiceProxy("gazebo/pause_physics",Empty)
    unpause_world = rospy.ServiceProxy("gazebo/unpause_physics",Empty)
    #pause world
    pause_world()
    #delete all models
    for modelName in get_world_prop().model_names:
        print(modelName)
        delete_model(modelName)
        rospy.wait_for_service("gazebo/delete_model")
    f = open('../models/ground/ground.sdf','r')
    c = open('../models/cliffordClosedChain/cliffordClosedChain.sdf','r')
    sdff = f.read()
    sdfc = c.read()

    cliffordPose = Pose()
    groundCount = 0
    for x in range(-5,6):
        for y in range(-5,5):
            print(groundCount)
            initial_pose = Pose()
            initial_pose.position.x = x*2
            initial_pose.position.y = y*2
            initial_pose.position.z = np.random.normal(0,0.1)
            if groundCount == 0:
                cliffordPose = initial_pose
            spawn_model_prox("ground" + str(groundCount), sdff, "", initial_pose, "world")
            groundCount = groundCount+1
    cliffordPose.position.z = cliffordPose.position.z + 6
    spawn_model_prox("clifford",sdfc,"", cliffordPose, "world")

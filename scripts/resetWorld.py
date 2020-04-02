import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties, SetPhysicsProperties, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState 
from std_srvs.srv import Empty
from geometry_msgs.msg import *
import numpy as np
import time

if __name__ == '__main__':
    # set up services
    print("Waiting for gazebo services...")
    rospy.init_node("mr_world_wide_resetter")
    rospy.wait_for_service("gazebo/reset_simulation")
    print("Got it.")
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    resetWorld = rospy.ServiceProxy("gazebo/reset_simulation",Empty)
    pauseWorld = rospy.ServiceProxy("gazebo/pause_physics",Empty)
    unpause_world = rospy.ServiceProxy("gazebo/unpause_physics",Empty)
    get_world_prop = rospy.ServiceProxy("gazebo/get_world_properties",GetWorldProperties)

    # find ground blocks
    groundNames = []
    for modelName in get_world_prop().model_names:
        if "ground" in modelName:
            groundNames.append(modelName)

    # reset world and pause
    pauseWorld()
    resetWorld()

    #move ground blocks
    for groundName in groundNames:
        print(groundName)
        groundState = get_state(groundName,"world")
        state_msg = ModelState()
        state_msg.model_name = groundName
        state_msg.pose.position.x = groundState.pose.position.x
        state_msg.pose.position.y = groundState.pose.position.y
        state_msg.pose.position.z = np.random.normal(0,0.1)
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        set_state(state_msg)
        while get_state(groundName,"world").pose.position.z!=state_msg.pose.position.z:
            print("here")
            set_state(state_msg)



    #state_msg = ModelState()
    #state_msg.model_name = 'clifford'
    #state_msg.pose.position.x = 0
    #state_msg.pose.position.y = 0
    #state_msg.pose.position.z = 10
    #state_msg.pose.orientation.x = 0
    #state_msg.pose.orientation.y = 0
    #state_msg.pose.orientation.z = 0
    #state_msg.pose.orientation.w = 0
    #set_state(state_msg)
    #time.sleep(0.5)
    #pause_world()
    #time.sleep(5)
    #unpause_world()
    #print(get_state("clifford","world").pose.position.x)
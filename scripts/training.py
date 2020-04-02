import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties, SetPhysicsProperties, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty
from geometry_msgs.msg import *
import numpy as np
import time

class worldModifier:
    """ example calls
    self.reset()
    self.pause()
    self.unpause()
    self.getProp()
    self.getModelNames
    state_msg = ModelState()
    state_msg.model_name = 
    state_msg.pose.position.x = 
    state_msg.pose.position.y = 
    state_msg.pose.position.z = 
    state_msg.pose.orientation.x = 
    state_msg.pose.orientation.y = 
    state_msg.pose.orientation.z = 
    state_msg.pose.orientation.w = 
    self.setModelState(state_msg)
    self.getModelState(modelName)

    """
    def __init__(self):
        print("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/reset_simulation")
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service("gazebo/pause_physics")
        rospy.wait_for_service("gazebo/unpause_physics")
        rospy.wait_for_service("gazebo/get_world_properties")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/delete_model")
        print("Got it.")
        self.setModelState = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.getModelStateRelative = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.reset = rospy.ServiceProxy("gazebo/reset_simulation",Empty)
        self.pause = rospy.ServiceProxy("gazebo/pause_physics",Empty)
        self.unpause = rospy.ServiceProxy("gazebo/unpause_physics",Empty)
        self.getProp = rospy.ServiceProxy("gazebo/get_world_properties",GetWorldProperties)
        self.delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.worldInitialized = False
    def getModelNames(self):
        return self.getProp().model_names
    def getModelState(self,modelName):
        return self.getModelStateRelative(modelName,"world")
    def setModelPose(self,modelName,posx,posy,posz,orienx,orieny,orienz,orienw):
        state_msg = ModelState()
        state_msg.model_name = modelName
        state_msg.pose.position.x = posx
        state_msg.pose.position.y = posy
        state_msg.pose.position.z = posz
        state_msg.pose.orientation.x = orienx
        state_msg.pose.orientation.y = orieny
        state_msg.pose.orientation.z = orienz
        state_msg.pose.orientation.w = orienw
        self.setModelState(state_msg)
    def setUpWorld(self):
        self.pause()
        self.reset()
        if self.worldInitialized:
            # world already initialized, just reset world and move ground blocks
            for i in range(len(self.groundBlocks)):
                name = self.groundBlocks[i]
                x = self.groundBlockXs[i]
                y = self.groundBlockYs[i]
                z = np.random.normal(0,0.1)
                self.setModelPose(name,x,y,z,0,0,0,0)
        else:
            # set up world from scratch
            # delete all models
            cliffordExits = False
            for modelName in self.getModelNames():
                if modelName == "clifford":
                    cliffordExits = True
                else:
                    self.delete(modelName)
            # open sdfs
            g = open('../models/ground/ground.sdf','r')
            c = open('../models/cliffordClosedChain/cliffordClosedChain.sdf','r')
            sdfg = g.read()
            sdfc = c.read()
            # spawn
            self.groundBlocks = []
            self.groundBlockXs = []
            self.groundBlockYs = []
            cliffordPose = Pose()
            groundCount = 0
            for x in range(-5,6):
                for y in range(-5,6):
                    groundName = "ground"+str(groundCount)
                    initial_pose = Pose()
                    initial_pose.position.x = x*2
                    initial_pose.position.y = y*2
                    initial_pose.position.z = np.random.normal(0,0.1)
                    if groundCount == 0:
                        cliffordPose = initial_pose
                    self.spawn(groundName,sdfg,"",initial_pose,"world")
                    self.groundBlocks.append(groundName)
                    self.groundBlockXs.append(initial_pose.position.x)
                    self.groundBlockYs.append(initial_pose.position.y)
                    groundCount = groundCount+1
            cliffordPose.position.z = cliffordPose.position.z + 6
            if cliffordExits:
                self.setModelPose("clifford",cliffordPose.position.x,cliffordPose.position.y,cliffordPose.position.z,0,0,0,0)
            else:
                self.spawn("clifford",sdfc,"", cliffordPose, "world")
            self.worldInitialized = True
        self.unpause()


if __name__ == '__main__':
    rospy.init_node("trainer")
    world = worldModifier()
    world.setUpWorld()
    cliffordCmd = rospy.Publisher('/cliffordDrive', Twist, queue_size=100)
    linear  = Vector3(1, 1, 0.0)
    angular = Vector3(0.0, 0.0, 1)
    twist = Twist(linear, angular)
    cliffordCmd.publish(twist)
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
    def __init__(self,numGroundBlocksX,numGroundBlocksY,zVar,GuiWaitTime):
        self.GuiWaitTime = GuiWaitTime
        self.groundBlockNames = []
        self.groundBlockMatrixXs = []
        self.groundBlockMatrixYs = []
        self.groundBlockXs = np.zeros((2*numGroundBlocksX+1,2*numGroundBlocksY+1))
        self.groundBlockYs = np.zeros((2*numGroundBlocksX+1,2*numGroundBlocksY+1))
        self.groundBlockZs = np.zeros((2*numGroundBlocksX+1,2*numGroundBlocksY+1))
        self.zVar = zVar
        groundCount = 0
        for i in range(-numGroundBlocksX,numGroundBlocksX+1):
            for j in range(-numGroundBlocksY,numGroundBlocksY+1):
                self.groundBlockNames.append("ground"+str(groundCount))
                MatrixX = numGroundBlocksX+i
                MatrixY = numGroundBlocksY+j
                self.groundBlockMatrixXs.append(MatrixX)
                self.groundBlockMatrixYs.append(MatrixY)
                self.groundBlockXs[MatrixX,MatrixY] = (2*i)
                self.groundBlockYs[MatrixX,MatrixY] = (2*j)
                groundCount = groundCount+1
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
        time.sleep(self.GuiWaitTime)
        self.setModelState(state_msg)
    def setUpWorld(self):
        self.pause()
        self.reset()
        # set random block heights
        self.groundBlockZs = np.random.normal(0,zVar,self.groundBlockZs.shape)
        if not self.worldInitialized:
            # world not initialized yet
            modelsInWorld = self.getModelNames()
            # delete models not supposed to be in sim
            for modelName in modelsInWorld:
                if (modelName!="clifford") and (not modelName in self.groundBlockNames):
                    self.delete(modelName)
            # import clifford if not in world already
            if not "clifford" in modelsInWorld:
                c = open('../models/cliffordClosedChain/cliffordClosedChain.sdf','r')
                sdfc = c.read()
                print("import clifford")
                self.spawn("clifford",sdfc,"", Pose(), "world")
            #import ground not in world already
            g = open('../models/ground/ground.sdf','r')
            sdfg = g.read()
            for i in range(len(self.groundBlockNames)):
                if not self.groundBlockNames[i] in modelsInWorld:
                    print("import " + self.groundBlockNames[i])
                    self.spawn(self.groundBlockNames[i],sdfg,"",Pose(),"world")
            self.worldInitialized = True
        # move blocks and clifford to desired position
        for i in range(len(self.groundBlockNames)):
            x = self.groundBlockXs[self.groundBlockMatrixXs[i],self.groundBlockMatrixYs[i]]
            y = self.groundBlockYs[self.groundBlockMatrixXs[i],self.groundBlockMatrixYs[i]]
            z = self.groundBlockZs[self.groundBlockMatrixXs[i],self.groundBlockMatrixYs[i]]
            while not self.groundBlockPositionCheck(i):
                self.setModelPose(self.groundBlockNames[i],x,y,z,0,0,0,0)
            if i == 60:
                z = z+6
                self.setModelPose("clifford",x,y,z,0,0,0,0)
        self.unpause()
    def groundBlockPositionCheck(self,blockNum):
        check = True
        tol = 0.0001;
        state = self.getModelState(self.groundBlockNames[blockNum])
        if np.abs(state.pose.position.x-self.groundBlockXs[self.groundBlockMatrixXs[blockNum],self.groundBlockMatrixYs[blockNum]])>tol:
            check = False
        elif np.abs(state.pose.position.y-self.groundBlockYs[self.groundBlockMatrixXs[blockNum],self.groundBlockMatrixYs[blockNum]])>tol:
            check = False
        elif np.abs(state.pose.position.z-self.groundBlockZs[self.groundBlockMatrixXs[blockNum],self.groundBlockMatrixYs[blockNum]])>tol:
            check = False
        return check


if __name__ == '__main__':
    rospy.init_node("trainer")
    cliffordCmd = rospy.Publisher('/cliffordDrive', Twist, queue_size=100)
    linear  = Vector3(0, 0, 0.0)
    angular = Vector3(0.0, 0.0, 0)
    twist = Twist(linear, angular)
    cliffordCmd.publish(twist)
    numGroundBlocksX = 5 #num ground blocks in +x and -x direction (not including one at origin)
    numGroundBlocksY = 5 #num ground blocks in +y and -y direction (not including one at origin)
    zVar = 0.1;
    world = worldModifier(numGroundBlocksX,numGroundBlocksY,zVar,0.015)
    world.setUpWorld()
    time.sleep(3)
    linear  = Vector3(1, 0, 0.0)
    angular = Vector3(0.0, 0.0, 1)
    twist = Twist(linear, angular)
    cliffordCmd.publish(twist)
    for i in range(0,100):
        state = world.getModelState("clifford")
        print(state.twist.angular.z)
        time.sleep(0.1)
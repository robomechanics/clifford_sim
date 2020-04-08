#!/usr/bin/env python2

import rospy, tf
from os.path import dirname, join, abspath
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties, SetPhysicsProperties, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import OccupancyGrid, Path
from std_srvs.srv import Empty
from geometry_msgs.msg import *
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


class worldModifier:
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
            parentDir = dirname(dirname(abspath(__file__)))
            for modelName in modelsInWorld:
                if (modelName!="clifford") and (not modelName in self.groundBlockNames):
                    self.delete(modelName)
            # import clifford if not in world already
            if not "clifford" in modelsInWorld:
                sdfDir = join(parentDir, "models/cliffordClosedChain/cliffordClosedChain.sdf")
                c = open(sdfDir,'r')
                sdfc = c.read()
                print("import clifford")
                self.spawn("clifford",sdfc,"", Pose(), "world")
            #import ground not in world already
            sdfDir = join(parentDir, "models/ground/ground.sdf")
            g = open(sdfDir,'r')
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
            if i == 0:
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

def publishMap(world,mapOut):
    numGridsPerBlock = 1;20;
    map2publish = OccupancyGrid()
    map2publish.info.width = numGridsPerBlock*world.groundBlockZs.shape[0]
    map2publish.info.height = numGridsPerBlock*world.groundBlockZs.shape[1]
    map2publish.info.resolution = 2.0/numGridsPerBlock
    map2publish.info.origin.position.x = world.groundBlockXs[0,0]#-1
    map2publish.info.origin.position.y = world.groundBlockYs[0,0]#-1
    data = np.array([])
    for j in range (world.groundBlockZs.shape[1]):
        for k in range(numGridsPerBlock):
            for i in range(world.groundBlockZs.shape[1]):
                rowData = (np.ones(numGridsPerBlock))#*round(world.groundBlockZs[i,j]+1))
                data = np.append(data,rowData)
                #map2publish.data.append(np.ones(numGridsPerBlock)*round(world.groundBlockZs[i,j]))
    map2publish.data = data
    mapOut.publish(map2publish)
    print("published")

def publishCliffordSearchStart(world,startOut):
    cliffordState = world.getModelState("clifford").pose
    cliffordOrientation = cliffordState.orientation
    cliffordPosition = cliffordState.position
    r = R.from_quat([cliffordOrientation.x,cliffordOrientation.y,cliffordOrientation.z,cliffordOrientation.w])
    v = [1,0,0]
    v = r.apply(v)
    theta = np.arctan2(v[1],v[0])
    start2publish = Point()
    start2publish.x = cliffordPosition.x
    start2publish.y = cliffordPosition.y
    start2publish.z = theta
    startOut.publish(start2publish)

def getPath(data):
    print('got path')
    global pathX
    global pathY
    global pathTheta
    pathX = np.array([])
    pathY = np.array([])
    pathTheta = np.array([])
    vectorX = np.array([])
    vectorY = np.array([])
    vectorTheta = np.array([])
    for pose in data.poses:
        nextX = pose.pose.position.x
        nextY = pose.pose.position.y
        nextTheta = pose.pose.position.z
        vectorX = np.append(vectorX,nextX)
        vectorY = np.append(vectorY,nextY)
        vectorTheta = np.append(vectorTheta,nextTheta)
        maxStep = 0.001
        if (pathX.shape[0]>0):
            robotTheta = pathTheta[-1]
            rotationMatrix_rw = np.matrix([[np.cos(robotTheta),np.sin(robotTheta)],[-np.sin(robotTheta),np.cos(robotTheta)]])
            rotationMatrix_wr = np.matrix([[np.cos(robotTheta),-np.sin(robotTheta)],[np.sin(robotTheta),np.cos(robotTheta)]])
            stepX_w = nextX - pathX[-1]
            stepY_w = nextY - pathY[-1]
            stepTheta = nextTheta - pathTheta[-1]
            stepRobot = np.matmul(rotationMatrix_rw,np.matrix([[stepX_w],[stepY_w]]))
            #print(stepTheta)
            stepTheta = 3.14 - 2*np.arctan(abs(stepRobot[0,0]/stepRobot[1,0]))
            if (stepRobot[1,0]*stepRobot[0,0] < 0):
                stepTheta = -stepTheta
            #print(stepTheta)
            if stepTheta == 0:
                stepSize = stepRobot[0,0]
                numSteps = int(np.ceil(stepSize/maxStep))
                newStepSize = stepSize/numSteps
                for i in range(numSteps):
                    pathX = np.append(pathX,pathX[-1]+stepX_w*newStepSize/stepSize)
                    pathY = np.append(pathY,pathY[-1]+stepY_w*newStepSize/stepSize)
                    pathTheta = np.append(pathTheta,pathTheta[-1])
            else:
                turnRadius = abs(stepRobot[0,0])/np.sin(abs(stepTheta))
                stepSize = turnRadius*abs(stepTheta)
                numSteps = int(np.ceil(stepSize/maxStep))
                newStepSize = stepSize/numSteps
                startX = pathX[-1]
                startY = pathY[-1]
                startTheta = pathTheta[-1]
                for i in range(numSteps):
                    totalStepTheta = (i+1)*stepTheta*newStepSize/stepSize
                    totalStepX = np.sign(stepRobot[0])*turnRadius*abs(np.sin(totalStepTheta))
                    totalStepY = np.sign(stepRobot[1])*turnRadius*(1-np.cos(totalStepTheta))
                    totalStepRobot = np.matrix([[totalStepX[0,0]],[totalStepY[0,0]]])
                    totalStepWorld = np.matmul(rotationMatrix_wr,totalStepRobot)
                    pathX = np.append(pathX,startX + totalStepWorld[0,0])
                    pathY = np.append(pathY,startY + totalStepWorld[1,0])
                    pathTheta = np.append(pathTheta,startTheta+totalStepTheta)
        else:
            pathX = np.append(pathX,nextX)
            pathY = np.append(pathY,nextY)
            pathTheta = np.append(pathTheta,nextTheta)

        #print("x = " + str(pose.pose.position.x) + "y = " + str(pose.pose.position.y) + "theta = " + str(pose.pose.position.z))
    pathIndex = range(pathX.shape[0])
    vectorLength = 0.01;
    U = vectorLength*np.cos(vectorTheta)
    V = vectorLength*np.sin(vectorTheta)
    #plt.quiver(vectorX,vectorY,U,V,linewidths=1)
    plt.plot(pathX,pathY)
    plt.xlim(-11,11)
    plt.ylim(-11,11)
    plt.show()
    driveClifford()

def driveClifford():
    global world
    global cliffordCmd
    global pathX
    global pathY
    global pathZ
    actualX = np.array([])
    actualY = np.array([])
    kp = np.matrix([[30],[50]])
    kd = np.matrix([[0],[10]])
    lastError = np.matrix([[0],[0]])
    for i in range(pathX.shape[0]):
        cliffordState = world.getModelState("clifford").pose
        cliffordOrientation = cliffordState.orientation
        cliffordPosition = cliffordState.position
        r = R.from_quat([cliffordOrientation.x,cliffordOrientation.y,cliffordOrientation.z,cliffordOrientation.w])
        v = [1,0,0]
        v = r.apply(v)
        theta = np.arctan2(v[1],v[0])
        worldError = np.matrix([[pathX[i] - cliffordPosition.x],[pathY[i] - cliffordPosition.y]])
        rotationMatrix_rw = np.matrix([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]])
        robotError = np.matmul(rotationMatrix_rw,worldError)
        drive = np.multiply(kp,robotError) - np.multiply(kd,(robotError-lastError))
        drive = np.matrix([[np.clip(drive[0,0],-1,1)],[np.clip(drive[1,0],-1,1)]])
        lastError = robotError;
        linear  = Vector3(drive[0,0], 0, 0.0)
        angular = Vector3(0.0, 0.0, drive[1,0])
        twist = Twist(linear, angular)
        cliffordCmd.publish(twist)
        actualX = np.append(actualX,cliffordPosition.x)
        actualY= np.append(actualY,cliffordPosition.y)
        print("drive " + str(drive[0,0]) + " steer" + str(drive[1,0]))
        rospy.sleep(0.00025)
    linear  = Vector3(0, 0, 0.0)
    angular = Vector3(0.0, 0.0, 0)
    twist = Twist(linear, angular)
    cliffordCmd.publish(twist)
    plt.plot(pathX,pathY)
    plt.plot(actualX,actualY)
    plt.xlim(-11,11)
    plt.ylim(-11,11)
    plt.show()


if __name__ == '__main__':
    global world
    global cliffordCmd
    rospy.init_node("trainer")
    cliffordCmd = rospy.Publisher('/cliffordDrive', Twist, queue_size=100)
    mapOut = rospy.Publisher('/Map', OccupancyGrid, queue_size=100)
    startOut = rospy.Publisher('/SearchStart', Point, queue_size=100)
    goalOut = rospy.Publisher('/SearchGoal', Point, queue_size=100)
    rospy.Subscriber('/CliffordPath',Path,getPath)
    linear  = Vector3(0, 0, 0.0)
    angular = Vector3(0.0, 0.0, 0)
    twist = Twist(linear, angular)
    cliffordCmd.publish(twist)
    numGroundBlocksX = 5 #num ground blocks in +x and -x direction (not including one at origin)
    numGroundBlocksY = 5 #num ground blocks in +y and -y direction (not including one at origin)
    zVar = 0;
    world = worldModifier(numGroundBlocksX,numGroundBlocksY,zVar,0.015)
    world.setUpWorld()
    linear  = Vector3(0, 0, 0.0)
    angular = Vector3(0.0, 0.0, 0)
    twist = Twist(linear, angular)
    cliffordCmd.publish(twist)
    publishMap(world,mapOut)
    publishCliffordSearchStart(world,startOut)
    goal = Point()
    goal.x = -10
    goal.y = 5
    goal.z = 0
    time.sleep(1)
    goalOut.publish(goal)
    rospy.spin()

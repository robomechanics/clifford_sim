import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties, SetPhysicsProperties, SetModelState, GetModelState, GetJointProperties
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import *
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import OccupancyGrid, Path
from std_srvs.srv import Empty
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import rospkg
class simController:
    def __init__(self,numGroundBlocks,zVar,numRobotMap,robotMapResolution,pathSize,GuiWaitTime,terminateParams):
        self.cliffordSDFPath = rospkg.RosPack().get_path('clifford_sim')+'/models/clifford/clifford.sdf'
        self.groundSDFPath = rospkg.RosPack().get_path('clifford_sim')+'/models/ground/ground.sdf'
        self.GuiWaitTime = GuiWaitTime
        self.groundBlockNames = []
        self.groundBlockMatrixXs = []
        self.groundBlockMatrixYs = []
        self.groundBlockXs = np.zeros((2*numGroundBlocks+1,2*numGroundBlocks+1))
        self.groundBlockYs = np.zeros((2*numGroundBlocks+1,2*numGroundBlocks+1))
        self.groundBlockZs = np.zeros((2*numGroundBlocks+1,2*numGroundBlocks+1))
        self.zVar = zVar
        self.pathSize = pathSize
        groundCount = 0
        for i in range(-numGroundBlocks,numGroundBlocks+1):
            for j in range(-numGroundBlocks,numGroundBlocks+1):
                self.groundBlockNames.append("ground"+str(groundCount))
                MatrixX = numGroundBlocks+i
                MatrixY = numGroundBlocks+j
                self.groundBlockMatrixXs.append(MatrixX)
                self.groundBlockMatrixYs.append(MatrixY)
                self.groundBlockXs[MatrixX,MatrixY] = (2*i)
                self.groundBlockYs[MatrixX,MatrixY] = (2*j)
                groundCount = groundCount+1
        rospy.init_node("trainer")
        self.robotMapX = np.arange(-numRobotMap,numRobotMap+1)*robotMapResolution
        self.robotMapY = np.arange(-numRobotMap,numRobotMap+1)*robotMapResolution
        self.robotMapY,self.robotMapX = np.meshgrid(self.robotMapY,self.robotMapX)
        self.cliffordCmd = rospy.Publisher('/cliffordDrive', Twist, queue_size=100)
        self.mapOut = rospy.Publisher('/Map', OccupancyGrid, queue_size=100)
        self.startOut = rospy.Publisher('/SearchStart', Point, queue_size=100)
        self.goalOut = rospy.Publisher('/SearchGoal', Point, queue_size=100)
        rospy.Subscriber('/CliffordPath',Path,self.getPath)
        print("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/reset_simulation")
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service("gazebo/pause_physics")
        rospy.wait_for_service("gazebo/unpause_physics")
        rospy.wait_for_service("gazebo/get_world_properties")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/get_joint_properties")
        print("Got it.")
        self.setModelState = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.getModelStateRelative = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.getJointState = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
        self.reset = rospy.ServiceProxy("gazebo/reset_simulation",Empty)
        self.pause = rospy.ServiceProxy("gazebo/pause_physics",Empty)
        self.unpause = rospy.ServiceProxy("gazebo/unpause_physics",Empty)
        self.getProp = rospy.ServiceProxy("gazebo/get_world_properties",GetWorldProperties)
        self.delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.worldInitialized = False
        self.lastError = np.matrix([[0],[0]])
        self.simTime = 0
        self.lastStuckCount = 0
        self.lastPosStuck = np.zeros(2)
        self.flipLimit = terminateParams[0]
        self.maxStuckCount = terminateParams[1]
        self.stuckThreshold = terminateParams[2]

    def setGuiWaitTime(self,GuiWaitTime):
        self.GuiWaitTime = GuiWaitTime

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

    def getTime(self):
        return rospy.get_time()

    def deleteAllButClifford(self):
        self.worldInitialized = False
        modelsInWorld = self.getProp().model_names
        # delete models not supposed to be in sim
        for modelName in modelsInWorld:
            if (modelName!="clifford"):
                self.delete(modelName)

    def setUpWorld(self,cliffordX,cliffordY,cliffordTheta,moveBlocks=True):
        self.pause()
        self.reset()
        # set random block heights
        if moveBlocks:
            self.groundBlockZs = np.random.normal(0,self.zVar,self.groundBlockZs.shape)
        if not self.worldInitialized:
            # world not initialized yet
            modelsInWorld = self.getProp().model_names
            # delete models not supposed to be in sim
            for modelName in modelsInWorld:
                if (modelName!="clifford") and (not modelName in self.groundBlockNames):
                    self.delete(modelName)
            # import clifford if not in world already
            if not "clifford" in modelsInWorld:
                #c = open('../models/clifford/clifford.sdf','r')
                c = open(self.cliffordSDFPath,'r')
                sdfc = c.read()
                print("import clifford")
                self.spawn("clifford",sdfc,"", Pose(), "world")
            #import ground not in world already
            #g = open('../models/ground/ground.sdf','r')
            g = open(self.groundSDFPath,'r')
            sdfg = g.read()
            pose = Pose()
            for i in range(len(self.groundBlockNames)):
                if not self.groundBlockNames[i] in modelsInWorld:
                    print("import " + self.groundBlockNames[i])
                    pose.position.x = self.groundBlockXs[self.groundBlockMatrixXs[i],self.groundBlockMatrixYs[i]]
                    pose.position.y = self.groundBlockYs[self.groundBlockMatrixXs[i],self.groundBlockMatrixYs[i]]
                    pose.position.z = self.groundBlockZs[self.groundBlockMatrixXs[i],self.groundBlockMatrixYs[i]]
                    self.spawn(self.groundBlockNames[i],sdfg,"",pose,"world")
            self.worldInitialized = True
        # move blocks and clifford to desired position
        r = R.from_rotvec(cliffordTheta*np.array([0, 0, 1]))
        r = r.as_quat()
        self.setModelPose("clifford",cliffordX,cliffordY,self.groundBlockZs.max()+5.5,r[0],r[1],r[2],r[3])
        for i in range(len(self.groundBlockNames)):
            x = self.groundBlockXs[self.groundBlockMatrixXs[i],self.groundBlockMatrixYs[i]]
            y = self.groundBlockYs[self.groundBlockMatrixXs[i],self.groundBlockMatrixYs[i]]
            z = self.groundBlockZs[self.groundBlockMatrixXs[i],self.groundBlockMatrixYs[i]]
            while not self.groundBlockPositionCheck(i):
                self.setModelPose(self.groundBlockNames[i],x,y,z,0,0,0,0)
        self.publishMap()
        self.driveClifford([0,0])
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

    def getCliffordPlanarState(self):
        cliffordState = self.getModelState("clifford").pose
        cliffordOrientation = cliffordState.orientation
        cliffordPosition = cliffordState.position
        r = R.from_quat([cliffordOrientation.x,cliffordOrientation.y,cliffordOrientation.z,cliffordOrientation.w])
        v = [1,0,0]
        v = r.apply(v)
        theta = np.arctan2(v[1],v[0])
        return [cliffordPosition.x, cliffordPosition.y, theta]

    def publishMap(self):
        numGridsPerBlock = 1;
        map2publish = OccupancyGrid()
        map2publish.info.width = self.groundBlockZs.shape[0]
        map2publish.info.height = self.groundBlockZs.shape[1]
        map2publish.info.resolution = 2.0
        map2publish.info.origin.position.x = self.groundBlockXs[0,0]
        map2publish.info.origin.position.y = self.groundBlockYs[0,0]
        map2publish.data = np.ones(map2publish.info.width*map2publish.info.height)
        self.mapOut.publish(map2publish)
    
    def searchPath(self,xGoal,yGoal):
        print("starting search")
        cliffordState = self.getCliffordPlanarState()
        start2publish = Point()
        start2publish.x = 0#cliffordState[0]
        start2publish.y = 0#cliffordState[1]
        start2publish.z = 0#cliffordState[2]
        self.startOut.publish(start2publish)
        goal = Point()
        goal.x = xGoal
        goal.y = yGoal
        goal.z = 0
        self.goalOut.publish(goal)
        self.waitingForPath = True
        while self.waitingForPath:
            rospy.sleep(0.001)
        if self.pathXY.shape[1] == 1:
            return False
        return True

    def getPath(self,data):
        print('receiving path')
        # find total path length
        start = True
        # find total length of path
        pathLength = 0
        for pose in data.poses:
            if start:
                start = False
                nextX = pose.pose.position.x
                nextY = pose.pose.position.y
                nextTheta = pose.pose.position.z
                continue
            lastX = nextX
            lastY = nextY
            lastTheta = nextTheta
            nextX = pose.pose.position.x
            nextY = pose.pose.position.y
            rotationMatrix_rw = np.matrix([[np.cos(lastTheta),np.sin(lastTheta)],[-np.sin(lastTheta),np.cos(lastTheta)]])
            rotationMatrix_wr = np.matrix([[np.cos(lastTheta),-np.sin(lastTheta)],[np.sin(lastTheta),np.cos(lastTheta)]])
            stepX_w = nextX - lastX
            stepY_w = nextY - lastY
            stepRobot = np.matmul(rotationMatrix_rw,np.matrix([[stepX_w],[stepY_w]]))
            stepTheta = 3.14 - 2*np.arctan(abs(stepRobot[0,0]/stepRobot[1,0]))
            if (stepRobot[1,0]*stepRobot[0,0] < 0):
                stepTheta = -stepTheta
            nextTheta += stepTheta
            if stepTheta == 0:
                pathLength += stepRobot[0,0]
            else:
                turnRadius = abs(stepRobot[0,0])/np.sin(abs(stepTheta))
                pathLength += turnRadius*abs(stepTheta)
        # step along path in equal increments to get same path length
        stepSize = pathLength/(self.pathSize-1)
        totalStepsTaken = 0
        pathLengthBeforeSegment = 0
        self.pathX = np.array([])
        self.pathY = np.array([])
        self.pathTheta = np.array([])
        start = True
        for pose in data.poses:
            if start:
                start = False
                nextX = pose.pose.position.x
                nextY = pose.pose.position.y
                nextTheta = pose.pose.position.z
                _lastX = nextX
                _lastY = nextY
                _lastTheta = nextTheta
                self.pathX = np.append(self.pathX,nextX)
                self.pathY = np.append(self.pathY,nextY)
                self.pathTheta = np.append(self.pathTheta,nextTheta)
                continue
            lastX = _lastX
            lastY = _lastY
            lastTheta = _lastTheta
            nextX = pose.pose.position.x
            nextY = pose.pose.position.y
            _lastX = nextX
            _lastY = nextY
            rotationMatrix_rw = np.matrix([[np.cos(lastTheta),np.sin(lastTheta)],[-np.sin(lastTheta),np.cos(lastTheta)]])
            rotationMatrix_wr = np.matrix([[np.cos(lastTheta),-np.sin(lastTheta)],[np.sin(lastTheta),np.cos(lastTheta)]])
            stepX_w = nextX - lastX
            stepY_w = nextY - lastY
            stepRobot = np.matmul(rotationMatrix_rw,np.matrix([[stepX_w],[stepY_w]]))
            stepTheta = 3.14 - 2*np.arctan(abs(stepRobot[0,0]/stepRobot[1,0]))
            if (stepRobot[1,0]*stepRobot[0,0] < 0):
                stepTheta = -stepTheta
            _lastTheta = lastTheta + stepTheta
            nextTheta += stepTheta
            if stepTheta == 0:
                segmentLength = stepRobot[0,0]
                while (pathLengthBeforeSegment + segmentLength - (totalStepsTaken+1)*stepSize) >= 0:
                    stepFraction = (((totalStepsTaken+1)*stepSize) - pathLengthBeforeSegment)/segmentLength
                    nextX = lastX+stepX_w*stepFraction
                    nextY = lastY+stepY_w*stepFraction
                    nextTheta = lastTheta
                    self.pathX = np.append(self.pathX,nextX)
                    self.pathY = np.append(self.pathY,nextY)
                    self.pathTheta = np.append(self.pathTheta,nextTheta)
                    totalStepsTaken+=1
                pathLengthBeforeSegment += segmentLength
            else:
                turnRadius = abs(stepRobot[0,0])/np.sin(abs(stepTheta))
                segmentLength = turnRadius*abs(stepTheta)
                startX = lastX
                startY = lastY
                startTheta = lastTheta
                while (pathLengthBeforeSegment + segmentLength - (totalStepsTaken+1)*stepSize) >= 0:
                    stepFraction = (((totalStepsTaken+1)*stepSize) - pathLengthBeforeSegment)/segmentLength
                    totalStepTheta = stepTheta*stepFraction
                    totalStepX = np.sign(stepRobot[0])*turnRadius*abs(np.sin(totalStepTheta))
                    totalStepY = np.sign(stepRobot[1])*turnRadius*(1-np.cos(totalStepTheta))
                    totalStepRobot = np.matrix([[totalStepX[0,0]],[totalStepY[0,0]]])
                    totalStepWorld = np.matmul(rotationMatrix_wr,totalStepRobot)
                    nextX = startX + totalStepWorld[0,0]
                    nextY = startY + totalStepWorld[1,0]
                    nextTheta = startTheta+totalStepTheta
                    self.pathX = np.append(self.pathX,nextX)
                    self.pathY = np.append(self.pathY,nextY)
                    self.pathTheta = np.append(self.pathTheta,nextTheta)
                    totalStepsTaken+=1
                pathLengthBeforeSegment += segmentLength
        while self.pathX.size < self.pathSize:
            self.pathX = np.append(self.pathX,self.pathX[-1])
            self.pathY = np.append(self.pathY,self.pathY[-1])
        self.pathX = self.pathX[0:self.pathSize]
        self.pathY = self.pathY[0:self.pathSize]
        self.pathXY = np.stack((self.pathX,self.pathY))
        print('got path')
        self.waitingForPath = False
        self.pdIndex = 0

    def setPath(self,pathX,pathY):
        self.pathX = pathX
        self.pathY = pathY
        self.pathXY = np.stack((self.pathX,self.pathY))
        self.waitingForPath = False
        self.pdIndex = 0

    def pdControl(self):
        kp = np.matrix([[10],[5]])
        kd = np.matrix([[0],[0.1]])
        cliffordState = self.getModelState("clifford").pose
        cliffordOrientation = cliffordState.orientation
        cliffordPosition = np.matrix([[cliffordState.position.x],[cliffordState.position.y]])
        r = R.from_quat([cliffordOrientation.x,cliffordOrientation.y,cliffordOrientation.z,cliffordOrientation.w])
        v = [1,0,0]
        v = r.apply(v)
        theta = np.arctan2(v[1],v[0])
        pathDistances = self.pathXY-np.matmul(cliffordPosition,np.ones((1,self.pathXY.shape[1])))
        pathDistances = np.linalg.norm(pathDistances,axis=0)
        self.pdIndex = np.max([self.pdIndex, np.min([pathDistances.argmin()+3,self.pathXY.shape[1]-1])])
        worldError = np.matrix(self.pathXY[:,self.pdIndex]).transpose() - cliffordPosition
        rotationMatrix_rw = np.matrix([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]])
        robotError = np.matmul(rotationMatrix_rw,worldError)
        drive = np.multiply(kp,robotError) - np.multiply(kd,(robotError-self.lastError))
        drive = np.matrix([[np.clip(drive[0,0],-0.5,0.5)],[np.clip(drive[1,0],-1,1)]])
        self.lastError = robotError;
        return [drive[0,0],drive[1,0]]

    def driveClifford(self,action):
        linear  = Vector3(np.clip(action[0],-1,1), 0, 0.0)
        angular = Vector3(0.0, 0.0, np.clip(action[1],-1,1))
        self.cliffordCmd.publish(Twist(linear, angular))
    def getState(self):
        posVel = self.getModelState("clifford")
        frs = self.getJointState("frslower2upper").position[0]
        fls = self.getJointState("flslower2upper").position[0]
        brs = self.getJointState("brslower2upper").position[0]
        bls = self.getJointState("blslower2upper").position[0]
        frw = self.getJointState("frwheel2tire").rate[0]
        flw = self.getJointState("flwheel2tire").rate[0]
        brw = self.getJointState("brwheel2tire").rate[0]
        blw = self.getJointState("blwheel2tire").rate[0]
        steering = self.getJointState("axle2flwheel").position[0]
        frsV = self.getJointState("frslower2upper").rate[0]
        flsV = self.getJointState("flslower2upper").rate[0]
        brsV = self.getJointState("brslower2upper").rate[0]
        blsV = self.getJointState("blslower2upper").rate[0]
        steeringV = self.getJointState("axle2flwheel").rate[0]

        robotState = [posVel.pose.position.x,posVel.pose.position.y,posVel.pose.position.z,\
        posVel.pose.orientation.x,posVel.pose.orientation.y,posVel.pose.orientation.z,posVel.pose.orientation.w,\
        posVel.twist.linear.x,posVel.twist.linear.y,posVel.twist.linear.z,\
        posVel.twist.angular.x,posVel.twist.angular.y,posVel.twist.angular.z,\
        frs,fls,brs,bls,frw,flw,brw,blw,steering,frsV,flsV,brsV,blsV,steeringV]
        r = R.from_quat([posVel.pose.orientation.x,posVel.pose.orientation.y,posVel.pose.orientation.z,posVel.pose.orientation.w])
        x = [1,0,0]
        x = r.apply(x)
        y = [0,1,0]
        y = r.apply(y)
        x = x[0:2]/np.linalg.norm(x[0:2])
        y = y[0:2]/np.linalg.norm(x[0:2])
        rotMatrix_wr = np.stack((x,y)).transpose()
        # robot map points in robot frame
        robotMapPoints = np.stack((self.robotMapX.reshape(-1),self.robotMapY.reshape(-1)))
        # change to world frame
        robotMapPoints = np.matmul(rotMatrix_wr,robotMapPoints)
        # change to relative to first block corner
        robotPosRelativeFirstBlock = np.matrix([[posVel.pose.position.x - self.groundBlockXs[0,0]+1],[posVel.pose.position.y - self.groundBlockYs[0,0]+1]])
        robotMapPoints = robotMapPoints + np.matmul(robotPosRelativeFirstBlock,np.ones((1,robotMapPoints.shape[1])))
        # get index in map
        robotMapPoints = np.floor(robotMapPoints/2)
        outOfRange = np.any([robotMapPoints[0,:]<0,robotMapPoints[1,:]<0,robotMapPoints[0,:]>=self.groundBlockZs.shape[0],robotMapPoints[1,:]>=self.groundBlockZs.shape[1]],axis=0)
        robotMapPointsX = robotMapPoints[0,:].clip(0,self.groundBlockZs.shape[0]-1).astype(int)
        robotMapPointsY = robotMapPoints[1,:].clip(0,self.groundBlockZs.shape[1]-1).astype(int)
        robotMapVals = self.groundBlockZs[robotMapPointsX,robotMapPointsY]
        robotMapVals[outOfRange] = 1000
        robotMap = robotMapVals.reshape(self.robotMapX.shape)
        return(self.groundBlockZs,robotMap,robotState,self.terminateCheck(robotState))
    def getVirtualState(self,robotState):
        r = R.from_quat(robotState[4:8])
        x = [1,0,0]
        x = r.apply(x)
        y = [0,1,0]
        y = r.apply(y)
        x = x[0:2]/np.linalg.norm(x[0:2])
        y = y[0:2]/np.linalg.norm(x[0:2])
        rotMatrix_wr = np.stack((x,y)).transpose()
        # robot map points in robot frame
        robotMapPoints = np.stack((self.robotMapX.reshape(-1),self.robotMapY.reshape(-1)))
        # change to world frame
        robotMapPoints = np.matmul(rotMatrix_wr,robotMapPoints)
        # change to relative to first block corner
        robotPosRelativeFirstBlock = np.matrix([[robotState[0] - self.groundBlockXs[0,0]+1],[robotState[1] - self.groundBlockYs[0,0]+1]])
        robotMapPoints = robotMapPoints + np.matmul(robotPosRelativeFirstBlock,np.ones((1,robotMapPoints.shape[1])))
        # get index in map
        robotMapPoints = np.floor(robotMapPoints/2)
        outOfRange = np.any([robotMapPoints[0,:]<0,robotMapPoints[1,:]<0,robotMapPoints[0,:]>=self.groundBlockZs.shape[0],robotMapPoints[1,:]>=self.groundBlockZs.shape[1]],axis=0)
        robotMapPointsX = robotMapPoints[0,:].clip(0,self.groundBlockZs.shape[0]-1).astype(int)
        robotMapPointsY = robotMapPoints[1,:].clip(0,self.groundBlockZs.shape[1]-1).astype(int)
        robotMapVals = self.groundBlockZs[robotMapPointsX,robotMapPointsY]
        robotMapVals[outOfRange] = 1000
        robotMap = robotMapVals.reshape(self.robotMapX.shape)
        return(self.groundBlockZs,robotMap,robotState,self.terminateCheck(robotState))

    def terminateCheck(self,robotState):
        self.lastStuckCount+=1
        distSinceLastStuck = np.linalg.norm(np.array(robotState[0:2])-self.lastPosStuck)
        if distSinceLastStuck > self.stuckThreshold:
            self.lastPosStuck = np.array(robotState[0:2])
            self.lastStuckCount = 0
        # check if robot flipped over
        r = R.from_quat(robotState[3:7])
        if r.apply([0,0,1])[2] < self.flipLimit\
         or robotState[0] < -10.5 or robotState[1] < -10.5 or robotState[0]>10.5 or robotState[1] > 10.5\
         or self.lastStuckCount>self.maxStuckCount:
            self.lastPosStuck = np.zeros(2)
            self.lastStuckCount = 0
            return True
        return False

if __name__ == '__main__':
    numGroundBlocksX = 5 #num ground blocks in +x and -x direction (not including one at origin)
    numGroundBlocksY = 5 #num ground blocks in +y and -y direction (not including one at origin)
    zVar = 0;
    world = simController(numGroundBlocksX,numGroundBlocksY,zVar,0.015)
    world.setUpWorld()
    world.searchPath(10,10)
    print("done")
    #rospy.spin()

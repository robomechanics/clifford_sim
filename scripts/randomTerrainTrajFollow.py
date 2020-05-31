import rospy
import numpy as np
import matplotlib.pyplot as plt
from simController import simController
import time

if __name__ == '__main__':
    pathsX = np.loadtxt("pathX.csv", delimiter=",")
    pathsY = np.loadtxt("pathY.csv", delimiter=",")
    numGroundBlocks = 5 # number of blocks to right,left,forward, backward of the one center block
    zVar = 0.1 #variance of the height of the random ground blocks
    # below are parameters for the ground height map centered about the robot returned by sim controller in getState
    numRobotMap = 5
    robotMapResolution = 0.25
    pathSize = pathsX.shape[1]
    GuiWaitTime = 0.1 # wait time after moving component in simulation not needed for physics engine, but needed to properly visualize
    flipLimit = 0 # allowable limit for clifford's body to roll over
    maxStuckCount = 25 # if clifford doesn't move for this many control loops, it is considered stuck
    stuckThreshold = 0.05 # amount of movement needed for clifford's body to be considered as moved
    terminateParams = [flipLimit,maxStuckCount,stuckThreshold]
    world = simController(numGroundBlocks,zVar,numRobotMap,robotMapResolution,pathSize,GuiWaitTime,terminateParams)

    moveBlocks = True # whether to randomly move blocks when sim is reset
    simTerminate = True # to keep track of when the simulation needs to be reset
    timeBetweenControls = 0.2 # how often controls will be sent to clifford
    
    #below is for plotting the desired trajectory to track and the actual tracking results
    plt.figure(1)
    plt.draw()
    plt.pause(.001)
    plt.show(block=False)
    robotPathX = []
    robotPathY = []
    lastTrajX = []
    lastTrajY = []

    # Keep running the sim. Reset when clifford is stuck or reached goal
    while True:
        """run sim"""
        world.unpause()
        if simTerminate:
            # Do plotting for last trial
            plt.figure(1)
            plt.clf()
            plt.plot(robotPathX,robotPathY,'g--')
            plt.plot(lastTrajX,lastTrajY,'g-')
            #choose a random trajectory to track
            pathIndex = np.random.randint(0,pathsX.shape[0])
            world.setPath(pathsX[pathIndex,:],pathsY[pathIndex,:])
            #plot next trajectory to track
            plt.plot(world.pathX,world.pathY,'r-')
            lastTrajX = world.pathX
            lastTrajY = world.pathY
            plt.xlim(-11,11)
            plt.ylim(-11,11)
            plt.pause(0.01)
            robotPathX = []
            robotPathY = []
            # reset world: move ground blocks to new random location, move clifford to starting position/orientation
            initX = 0
            initY = 0
            initTheta = 0
            world.setUpWorld(initX,initY,initTheta,moveBlocks=moveBlocks)
            # keep track of how long until next command for clifford
            time.sleep(0.5)
            sleepUntil = world.getTime()
            simTerminate = False
        # calculate action clifford should take based on PD trajectory tracking controller
        actionPD = world.pdControl()
        print("Action Taken: " + str(actionPD))
        # send command to clifford
        world.driveClifford(actionPD)
        # wait for result of action
        sleepUntil += timeBetweenControls
        sleepTime = np.min([timeBetweenControls,sleepUntil-world.getTime()])
        if sleepTime < 0:
            print("running too slow")
            print(sleepTime)
        else:
            rospy.sleep(sleepTime)
        # Get new state of Clifford
        cliffordState = world.getState()
        print("Robot Position: " + str(cliffordState[2][0:2]))
        simTerminate = cliffordState[-1] # the value signals whether clifford is stuck, reached goal, off the map, rolled over...
        # check if close to goal
        if ((cliffordState[2][0]-lastTrajX[-1])**2+(cliffordState[2][1]-lastTrajY[-1])**2) < 0.5:
            simTerminate = True
        # keep track of position for plotting
        robotPathX.append(cliffordState[2][0])
        robotPathY.append(cliffordState[2][1])
    world.pause()

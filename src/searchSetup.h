#include "stlastar.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>

struct coord{
  float x;
  float y;
  float theta;
  int cost;
};

int max(int num1, int num2);
int min(int num1, int num2);

#define max()

class SearchData;
class robotKinematics;
class robotState
{
public:
  coord stateCoord;
  float distanceTol = 0.01;
  float headingTol = 0.5;
  robotKinematics* robot;
  SearchData* searchData;
  
  robotState() {}
  robotState(coord _stateCoord,robotKinematics* _robot, SearchData* _searchData) {stateCoord = _stateCoord;robot = _robot; searchData = _searchData;}
  void setRobot(robotKinematics* _robot){robot = _robot;}
  void setSearchData(SearchData* _searchData){searchData = _searchData;}

  float GoalDistanceEstimate( robotState &nodeGoal );
  bool IsGoal( robotState &nodeGoal );
  bool GetSuccessors( AStarSearch<robotState> *astarsearch, robotState *parent_node );
  float GetCost( robotState &successor );
  bool IsSameState( robotState &rhs );

  void PrintNodeInfo(); 
};




class SearchData
{
public:
  coord startCoord; coord goalCoord;
  float minX; float maxX; float minY; float maxY;
  int mapNumX; int mapNumY; int mapNumTheta;
  float mapDx; float mapDy; float mapDTheta;
  std::vector<signed char, std::allocator<signed char>> mapData;
  bool mapSet = false;
  AStarSearch<robotState>* astarsearch;
  robotKinematics* robot;
  ros::Publisher* path_pub;
  vector<int> gridConnectX;
  vector<int> gridConnectY;
  int neighbors2Connect = 5;

  SearchData(int maxStates,robotKinematics* _robot, ros::Publisher* _path_pub){
    astarsearch = new AStarSearch<robotState>(maxStates);
    path_pub = _path_pub;
    robot = _robot;
    initNeighbors();
  }

  SearchData(robotKinematics* _robot, ros::Publisher* _path_pub){
    astarsearch = new AStarSearch<robotState>;
    path_pub = _path_pub;
    robot = _robot;
    initNeighbors();
  }

  void initNeighbors(){
    for (int i=-neighbors2Connect;i<=neighbors2Connect;i++)
    {
      for (int j=-neighbors2Connect;j<=neighbors2Connect;j++)
      {
        if ((i!=0) && (j!=0))
        {
          gridConnectX.push_back((float)i*mapDx);
          gridConnectY.push_back((float)j*mapDy);
        }
      }
    }
  }

  void setMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO("setting map\n");
    nav_msgs::MapMetaData info = msg->info;
    mapNumX = info.width; mapNumY = info.height;
    minX = info.origin.position.x;
    maxX = info.origin.position.x + info.resolution*(float)(info.width-1); 
    minY = info.origin.position.y;
    maxY = info.origin.position.y + info.resolution*(float)(info.height-1);
    mapDx = mapDy = info.resolution;
    mapNumX = info.width; mapNumY = info.height;
    mapNumTheta = 1000;
    mapDTheta = 2*3.14/100;
    mapData = msg->data;
    mapSet = true;
  }

  void setStart(const geometry_msgs::Point::ConstPtr& msg){
    startCoord.x = msg->x;
    startCoord.y = msg->y;
    startCoord.theta = msg->z;
    ROS_INFO("start x %f, y %f, z %f \n",startCoord.x, startCoord.y, startCoord.theta);
  }

  void setGoal(const geometry_msgs::Point::ConstPtr& msg){
    goalCoord.x = msg->x;
    goalCoord.y = msg->y;
    goalCoord.theta = msg->z;
    ROS_INFO("goal x %f, y %f, z %f \n",goalCoord.x, goalCoord.y, goalCoord.theta);
    findTrajectory();
  }

  void findTrajectory(){
    // set start and goal states
    robotState nodeStart(startCoord, robot, this);
    robotState nodeEnd(goalCoord, robot, this);

    astarsearch->SetStartAndGoalStates( nodeStart, nodeEnd );
    int searchSteps = 0;

    unsigned int SearchState;
    do
    {
      SearchState = astarsearch->SearchStep();
      searchSteps++;
    }
    while( SearchState == AStarSearch<robotState>::SEARCH_STATE_SEARCHING );

    if( SearchState == AStarSearch<robotState>::SEARCH_STATE_SUCCEEDED )
    {
      ROS_INFO("Search found goal state\n");
      nav_msgs::Path path;
      geometry_msgs::PoseStamped pose;
      robotState* node = astarsearch->GetSolutionStart();
      pose.pose.position.x = node->stateCoord.x;
      //printf("nodeX: %f \n",node->stateCoord.x);
      pose.pose.position.y = node->stateCoord.y;
      pose.pose.position.z = node->stateCoord.theta;
      path.poses.push_back(pose);
      for( ;; )
      {
        node = astarsearch->GetSolutionNext();
        if( !node )
        {
          break;
        }
        pose.pose.position.x = node->stateCoord.x;
        //printf("nodeX: %f \n",node->stateCoord.x);
        pose.pose.position.y = node->stateCoord.y;
        pose.pose.position.z = node->stateCoord.theta;
        path.poses.push_back(pose);        
      };
      path_pub->publish(path);
      // Once you're done with the solution you can free the nodes up
      astarsearch->FreeSolutionNodes();  
    }
    else if( SearchState == AStarSearch<robotState>::SEARCH_STATE_FAILED ) 
    {
      ROS_INFO("Search terminated. Did not find goal state. Search Steps: %d\n",searchSteps);
    }
    astarsearch->EnsureMemoryFreed();
  }

  coord findMapPoint(coord inCoord)
  {
    coord outCoord;
    int xIndex = round((inCoord.x-minX)/mapDx);
    xIndex = min(max(xIndex,0),mapNumX-1);
    int yIndex = round((inCoord.y-minY)/mapDy);
    yIndex = min(max(yIndex,0),mapNumY-1);
    outCoord.x = (float)xIndex*mapDx + minX;
    outCoord.y = (float)yIndex*mapDy + minY;
    outCoord.theta = round(inCoord.theta/mapDTheta)*mapDTheta;
    outCoord.theta = fmod(outCoord.theta,2.0*3.14);
    if (outCoord.theta<0)
      outCoord.theta+=2.0*3.14;
    outCoord.cost = mapData[xIndex+mapNumX*yIndex];
    return outCoord;
  }

  bool checkValid(coord inCoord)
  {
    if ((inCoord.x<minX) || (inCoord.x>maxX) || (inCoord.y<minY) || (inCoord.y>maxY))
      return false;
    inCoord = findMapPoint(inCoord);
    if (inCoord.cost == 100)
      return false;
    return true;
  }

};

class robotKinematics
{
public:
  std::vector<float> steerAngs;
  std::vector<coord> changesRelativeRobot;
  float stepSize;
  robotKinematics(float _stepSize,float maxSteeringAng,int steerAngDisc, float robotLength)
  {
    stepSize = _stepSize;
    steerAngs.clear();
    changesRelativeRobot.clear();
    float dAng = maxSteeringAng/(float)(steerAngDisc);
    coord changeRelativeRobot;
    changeRelativeRobot.x = stepSize;
    changeRelativeRobot.y = 0;
    changeRelativeRobot.theta = 0;
    changesRelativeRobot.push_back(changeRelativeRobot);
    for(float i=1;i<steerAngDisc;i+=1.0)
    {
      float steeringAng = i*dAng;
      steerAngs.push_back(steeringAng);
      float turnRadius = robotLength/tan(steeringAng);
      float headingChange = stepSize/turnRadius;
      changeRelativeRobot.y = turnRadius-turnRadius*cos(headingChange);
      changeRelativeRobot.x = turnRadius*sin(headingChange);
      changeRelativeRobot.theta = headingChange;
      changesRelativeRobot.push_back(changeRelativeRobot);
      changeRelativeRobot.y = -changeRelativeRobot.y;
      changeRelativeRobot.theta = -changeRelativeRobot.theta;
      changesRelativeRobot.push_back(changeRelativeRobot);
      changeRelativeRobot.x = -changeRelativeRobot.x;
      changeRelativeRobot.theta = -changeRelativeRobot.theta;
      changesRelativeRobot.push_back(changeRelativeRobot);
      changeRelativeRobot.y = -changeRelativeRobot.y;
      changeRelativeRobot.theta = -changeRelativeRobot.theta;
      changesRelativeRobot.push_back(changeRelativeRobot);
      //printf("change x %f, y %f, theta %f",changeRelativeRobot.x,changeRelativeRobot.y,changeRelativeRobot.theta);
    }
  }
  std::vector<coord> calcNewStates(coord stateCoord)
  {
    std::vector<coord> newStates;
    for (unsigned i=0;i<steerAngs.size();i++)
    {
      coord newState = stateCoord;
      //printf("change x = %f, change y = %f",changesRelativeRobot[i].x*cos(stateCoord.theta) - changesRelativeRobot[i].y*sin(stateCoord.theta),
        //changesRelativeRobot[i].y*cos(stateCoord.theta) + changesRelativeRobot[i].x*sin(stateCoord.theta));
      newState.x += changesRelativeRobot[i].x*cos(stateCoord.theta) - changesRelativeRobot[i].y*sin(stateCoord.theta);
      newState.y += changesRelativeRobot[i].y*cos(stateCoord.theta) + changesRelativeRobot[i].x*sin(stateCoord.theta);
      //printf(" newx = %f, newy = %f\n", newState.x,newState.y);
      newState.theta += changesRelativeRobot[i].theta;
      newState.theta = fmod(newState.theta,2*3.14);
      newStates.push_back(newState);
    }
    return newStates;
  }
};


bool robotState::IsSameState( robotState &rhs )
{

	// same state in a maze search is simply when (x,y) are the same
	if( (stateCoord.x == rhs.stateCoord.x) &&
		(stateCoord.y == rhs.stateCoord.y) && (stateCoord.theta = rhs.stateCoord.theta))
	{
		return true;
	}
	else
	{
		return false;
	}

}

void robotState::PrintNodeInfo()
{
	printf("x = %f, y = %f, theta = %f\n",stateCoord.x,stateCoord.y,stateCoord.theta);
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float robotState::GoalDistanceEstimate( robotState &nodeGoal )
{
	return abs(stateCoord.x - nodeGoal.stateCoord.x) + abs(stateCoord.y - nodeGoal.stateCoord.y);
}

bool robotState::IsGoal( robotState &nodeGoal )
{
	float squareDist = ((nodeGoal.stateCoord.x-stateCoord.x)*(nodeGoal.stateCoord.x-stateCoord.x)+(nodeGoal.stateCoord.y-stateCoord.y)*(nodeGoal.stateCoord.y-stateCoord.y));
	float angleErr = abs(nodeGoal.stateCoord.theta-stateCoord.theta);
	if (abs(nodeGoal.stateCoord.theta+2*3.14-stateCoord.theta) < angleErr)
		angleErr = abs(nodeGoal.stateCoord.theta+2*3.14-stateCoord.theta);
	if (	(squareDist < distanceTol) && 
		(angleErr < headingTol)	)
	{
		return true;
	}

	return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool robotState::GetSuccessors( AStarSearch<robotState> *astarsearch, robotState *parent_node )
{
  robotState newState;
  coord newStateCoord;
  for (int i=0;i<searchData->gridConnectX.size();i++)
  {
    newStateCoord.x = stateCoord.x + searchData->gridConnectX[i];
    newStateCoord.y = stateCoord.y = searchData->gridConnectY[i];
    newStateCoord.theta = robot->calcHeading(searchData->gridConnectX[i],searchData->gridConnectY[i],stateCoord);
    if (searchData->checkValid(newStateCoord))
    {
      newStateCoord = searchData->findMapPoint(newStateCoord);
      newState = robotState(newStateCoord, robot, searchData);
      //printf("..successor: ");
      newState.PrintNodeInfo();
      astarsearch->AddSuccessor(newState);
    }
  }
  /*
  vector<coord> nextCoords = robot->calcNewStates(stateCoord);
	for (unsigned i=0;i<nextCoords.size();i++)
	{
		robotState newState;
		if (searchData->checkValid(nextCoords[i]))
		{
			nextCoords[i] = searchData->findMapPoint(nextCoords[i]);
			newState = robotState(nextCoords[i], robot, searchData);
      printf("..successor: ");
			newState.PrintNodeInfo();
			astarsearch->AddSuccessor(newState);
		}
	}*/
	return true;
}

float robotState::GetCost( robotState &successor )
{
	return robot->stepSize;
}
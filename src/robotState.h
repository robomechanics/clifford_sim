#include "stlastar.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include "SearchData.h"

class robotState
{
public:
  coord stateCoord;
  float distanceTol = 0.1;
  float headingTol = 2*3.14;
  SearchData* searchData;
  
  robotState() {}
  robotState(coord _stateCoord, SearchData* _searchData) {stateCoord = _stateCoord; searchData = _searchData;}
  void setSearchData(SearchData* _searchData){searchData = _searchData;}

  float GoalDistanceEstimate( robotState &nodeGoal );
  bool IsGoal( robotState &nodeGoal );
  bool GetSuccessors( AStarSearch<robotState> *astarsearch, robotState *parent_node );
  float GetCost( robotState &successor );
  bool IsSameState( robotState &rhs );

  void PrintNodeInfo(); 
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
	float distX = stateCoord.x - nodeGoal.stateCoord.x;
	float distY = stateCoord.y - nodeGoal.stateCoord.y;
	return sqrt(distX*distX+distY*distY);
}

bool robotState::IsGoal( robotState &nodeGoal )
{
	coord mapGoal = searchData->findMapPoint(nodeGoal.stateCoord);
	float squareDist = ((mapGoal.x-stateCoord.x)*(mapGoal.x-stateCoord.x)+(mapGoal.y-stateCoord.y)*(mapGoal.y-stateCoord.y));
	float angleErr = abs(mapGoal.theta-stateCoord.theta);
	if (abs(mapGoal.theta+2*3.14-stateCoord.theta) < angleErr)
		angleErr = abs(mapGoal.theta+2*3.14-stateCoord.theta);
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
	std::vector<coord> successorCoords = searchData->getSuccessorCoords(stateCoord);
	robotState newState = *this;
	for (int i=0;i<successorCoords.size();i++)
	{
		if (successorCoords[i].cost < 100)
		{
			newState.stateCoord = successorCoords[i];
			//newState.PrintNodeInfo();
			astarsearch->AddSuccessor(newState);
		}
	}
	return true;
}

float robotState::GetCost( robotState &successor )
{
	float r11 = cos(stateCoord.theta);
    float r12 = sin(stateCoord.theta);
    float r21 = -sin(stateCoord.theta);
    float r22 = cos(stateCoord.theta);
	float worldX = successor.stateCoord.x - stateCoord.x;
	float worldY = successor.stateCoord.y - stateCoord.y;
	float robotX = r11*worldX + r12*worldY;
    float robotY = r21*worldX + r22*worldY;
    float thetaChange = 3.14-2*atan(abs(robotX/robotY));
    float turnRadius = robotX/sin(thetaChange);
	float distance = abs(turnRadius*thetaChange);
	float straightDist = sqrt(worldX*worldX+worldY*worldY);
	float totalCost = (straightDist>distance ? straightDist:distance)*successor.stateCoord.cost;
	return totalCost;
};
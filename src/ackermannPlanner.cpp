#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <stdio.h>
#include <vector>
//#include "searchSetup.h"
#include "robotState.h"
SearchData searchData(100000,1,0.5);
ros::Publisher path_pub;

void setStart(const geometry_msgs::Point::ConstPtr& msg);
void setGoal(const geometry_msgs::Point::ConstPtr& msg);
void findTrajectory();

int main(int argc, char **argv){
  ros::init(argc, argv, "globalPlanner");
  ros::NodeHandle n;
  path_pub = n.advertise<nav_msgs::Path>("/CliffordPath", 1000);
  ros::Subscriber map_sub = n.subscribe("/Map",10,&SearchData::setMap, &searchData);
  ros::Subscriber start_sub = n.subscribe("/SearchStart",10,setStart);
  ros::Subscriber goal_sub = n.subscribe("/SearchGoal",10,setGoal);

  /*
  // set up robot kinematics
  robotKinematics robot(0.5,0.5,10, 0.1);
  // set up search data which stores map, start & goal location
  SearchData searchData(100000,&robot,&path_pub);
  ros::Subscriber map_sub = n.subscribe("/Map",10,&SearchData::setMap, &searchData);
  ros::Subscriber start_sub = n.subscribe("/SearchStart",10,&SearchData::setStart, &searchData);
  ros::Subscriber goal_sub = n.subscribe("/SearchGoal",10,&SearchData::setGoal, &searchData);
  ros::Rate loop_rate(10);
  int count = 0;
  printf("waiting...\n");
  /*while (ros::ok())
  {
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.poses.push_back(pose);
    path.poses.push_back(pose);
    path.poses.push_back(pose);
    path_pub.publish(path);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }*/

  ros::spin();
  return 0;
}

void setStart(const geometry_msgs::Point::ConstPtr& msg){
  searchData.startCoord.x = msg->x;
  searchData.startCoord.y = msg->y;
  searchData.startCoord.theta = msg->z;
  ROS_INFO("start x %f, y %f, z %f \n",searchData.startCoord.x, searchData.startCoord.y, searchData.startCoord.theta);
}

void setGoal(const geometry_msgs::Point::ConstPtr& msg){
  searchData.goalCoord.x = msg->x;
  searchData.goalCoord.y = msg->y;
  searchData.goalCoord.theta = msg->z;
  ROS_INFO("goal x %f, y %f, z %f \n",searchData.goalCoord.x, searchData.goalCoord.y, searchData.goalCoord.theta);
  findTrajectory();
}

void findTrajectory(){
  AStarSearch<robotState> astarsearch(1000000);
  // set start and goal states
  robotState nodeStart(searchData.startCoord, &searchData);
  robotState nodeEnd(searchData.goalCoord, &searchData);
  astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd);
  int searchSteps = 0;
  unsigned int SearchState;
  do
  {
    SearchState = astarsearch.SearchStep();
    searchSteps++;
    ROS_INFO("search steps: %d",searchSteps);
  }
  while( SearchState == AStarSearch<robotState>::SEARCH_STATE_SEARCHING );

  if( SearchState == AStarSearch<robotState>::SEARCH_STATE_SUCCEEDED )
  {
    ROS_INFO("Search found goal state\n");
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    robotState* node = astarsearch.GetSolutionStart();
    pose.pose.position.x = node->stateCoord.x;
    //printf("nodeX: %f \n",node->stateCoord.x);
    pose.pose.position.y = node->stateCoord.y;
    pose.pose.position.z = node->stateCoord.theta;
    path.poses.push_back(pose);
    for( ;; )
    {
      node = astarsearch.GetSolutionNext();
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
    path_pub.publish(path);
    // Once you're done with the solution you can free the nodes up
    astarsearch.FreeSolutionNodes();  
  }
  else if( SearchState == AStarSearch<robotState>::SEARCH_STATE_FAILED ) 
  {
    ROS_INFO("Search terminated. Did not find goal state. Search Steps: %d\n",searchSteps);
  }
  astarsearch.EnsureMemoryFreed();
}
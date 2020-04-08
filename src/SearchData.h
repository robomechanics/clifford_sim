#include "stlastar.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>

#define max(a,b) (a>b ? a:b)
#define min(a,b) (a<b ? a:b)

struct coord{
  float x;
  float y;
  float theta;
  int cost;
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
  vector<float> gridConnectX;
  vector<float> gridConnectY;
  int neighbors2Connect = 10;
  float robotLength;
  float maxSteeringAng;
  float minTurnRadius;
  int splitBlock = 20;

  SearchData(int _mapNumTheta, int _splitBlock, float _robotLength, float _maxSteeringAng){//, ros::Publisher* _path_pub){
    splitBlock = _splitBlock;
    mapNumTheta = _mapNumTheta;
    robotLength = _robotLength;
    maxSteeringAng = _maxSteeringAng;
    minTurnRadius = robotLength/tan(maxSteeringAng);
  }

  void initNeighbors(){
    gridConnectX.clear();
    gridConnectY.clear();
    for (int i=-neighbors2Connect;i<=neighbors2Connect;i++)
    {
      for (int j=-neighbors2Connect;j<=neighbors2Connect;j++)
      {
        if ((i!=0) || (j!=0))
        {
          gridConnectX.push_back((float)i*mapDx);
          gridConnectY.push_back((float)j*mapDy);
          printf("dx = %f, dy = %f \n",(float)i*mapDx,(float)j*mapDy);
        }
      }
    }
  }

  std::vector<coord> getSuccessorCoords(coord robotCoord)
  {
    float dirX_w,dirY_w,dirX_r,dirY_r;
    float headingChange,turnRadius;
    float r11 = cos(robotCoord.theta);
    float r12 = sin(robotCoord.theta);
    float r21 = -sin(robotCoord.theta);
    float r22 = cos(robotCoord.theta);

    std::vector<coord> successorCoords;
    coord successorCoord;
    for (int i=0;i<gridConnectX.size();i++)
    {
      dirX_w = gridConnectX[i];
      dirY_w = gridConnectY[i];
      dirX_r = r11*dirX_w + r12*dirY_w;
      if (dirX_r < 0)
        continue;
      dirY_r = r21*dirX_w + r22*dirY_w;
      headingChange = 3.14-2*atan(abs(dirX_r/dirY_r));
      if (dirX_r*dirY_r < 0)
        headingChange = -headingChange;
      float turnRadius = abs(dirX_r)/sin(abs(headingChange));
      if ((turnRadius > minTurnRadius) || (headingChange==0))
      {
        successorCoord.x = robotCoord.x + dirX_w;
        successorCoord.y = robotCoord.y + dirY_w;
        successorCoord.theta = robotCoord.theta + headingChange;
        successorCoords.push_back(findMapPoint(successorCoord));
      }
    }
    return successorCoords;
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
    outCoord.theta = (float)round(inCoord.theta/mapDTheta)*mapDTheta;
    outCoord.theta = fmod(outCoord.theta,2.0*3.14);
    if (outCoord.theta<0)
      outCoord.theta+=2.0*3.14;
    outCoord.cost = mapData[xIndex+mapNumX*yIndex];
    return outCoord;
  }

  int getMapCost(coord inCoord)
  {
    if ((inCoord.x<minX) || (inCoord.x>maxX) || (inCoord.y<minY) || (inCoord.y>maxY))
      return 100;
    inCoord = findMapPoint(inCoord);
    return inCoord.cost;
  }
/*
  void setMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO("setting map\n");
    nav_msgs::MapMetaData info = msg->info;
    mapNumX = info.width; mapNumY = info.height;
    minX = info.origin.position.x;
    maxX = info.origin.position.x + info.resolution*(float)(info.width-1); 
    minY = info.origin.position.y;
    maxY = info.origin.position.y + info.resolution*(float)(info.height-1);
    mapDx = mapDy = info.resolution;
    mapNumTheta = 1000;
    mapDTheta = 2*3.14/mapNumTheta;
    mapData = msg->data;
    mapSet = true;
    initNeighbors();
  }*/
  void setMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO("setting map\n");
    nav_msgs::MapMetaData info = msg->info;
    mapNumX = info.width*splitBlock; mapNumY = info.height*splitBlock;
    mapDx = mapDy = info.resolution/(float)splitBlock;
    minX = info.origin.position.x - 1.0 + mapDx/2.0;
    minY = info.origin.position.y - 1.0 + mapDx/2.0;
    maxX = minX + mapDx*(float)(mapNumX-1);
    maxY = minY + mapDy*(float)(mapNumY-1);
    mapDTheta = 2*3.14/mapNumTheta;

    mapNumX = info.width*splitBlock;
    mapNumY = info.width*splitBlock;
    mapData.clear();
    for (int i=0;i<info.width;i++)
    {
      for (int k=0;k<splitBlock;k++)
      {
        for (int j=0;j<info.height;j++)
        {
          for (int l=0;l<splitBlock;l++)
            mapData.push_back(msg->data[i+info.width*j]);
        }
      }
    }
    mapSet = true;
    initNeighbors();
  }
};
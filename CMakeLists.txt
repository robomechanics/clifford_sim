cmake_minimum_required(VERSION 2.8.3)

project(clifford_sim)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

#warning: variable CMAKE_CXX_FLAGS is modified
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#warning: use of link_directories() is strongly discouraged
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
  LIBRARIES ${PROJECT_NAME}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


add_library(cliffordPlugin src/cliffordPlugin.cpp)
target_link_libraries(cliffordPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(ackermannPlanner src/ackermannPlanner.cpp)
target_link_libraries(ackermannPlanner ${catkin_LIBRARIES})

install(TARGETS cliffordPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
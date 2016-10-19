cmake_minimum_required(VERSION 2.8.3)
project(dummy_slam)

find_package(catkin REQUIRED COMPONENTS 
		roscpp 
		tf
		nav_msgs
		geometry_msgs
		sensor_msgs
	    )


catkin_package(
    CATKIN_DEPENDS
      roscpp 
      tf
      nav_msgs
      geometry_msgs
      sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES
      dummy_slam_laser
      dummy_slam_map
      dummy_slam
)

include_directories(include
  ${catkin_INCLUDE_DIRS} 
)

set(CMAKE_BUILD_TYPE Release)

############### crsm_laser ###############
add_library(dummy_slam_laser src/dummy_slam/dummy_slam_laser.cpp)
target_link_libraries(dummy_slam_laser ${catkin_LIBRARIES} )

################ crsm_map ################
add_library(dummy_slam_map src/dummy_slam/dummy_slam_map.cpp)
target_link_libraries(dummy_slam_map ${catkin_LIBRARIES} )

################ crsm_slam ################
add_library(dummy_slam src/dummy_slam/dummy_slam.cpp)
target_link_libraries(dummy_slam 
		      ${catkin_LIBRARIES}
		      dummy_slam_laser
		      dummy_slam_map
)

############## crsm_slam_node ##############
add_executable(dummy_slam_node  src/dummy_slam/dummy_slam_node.cpp)
target_link_libraries(dummy_slam_node 
		      ${catkin_LIBRARIES}
		      dummy_slam
)


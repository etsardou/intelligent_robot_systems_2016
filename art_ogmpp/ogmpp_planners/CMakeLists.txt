cmake_minimum_required(VERSION 2.8.3)
project(ogmpp_planners)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  ogmpp_graph
  ogmpp_metrics
  ogmpp_map_loader
  ogmpp_communications
  ogmpp_search_algorithms
)

##################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ogmpp_prms
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    nav_msgs
    ogmpp_graph
    ogmpp_metrics
    ogmpp_map_loader
    ogmpp_communications
    ogmpp_search_algorithms
#  DEPENDS system_lib
)

###########
## Build ##
###########

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/ogmpp_planners
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ogmpp_planners 
  
  src/ogmpp_planners/ogmpp_planners_node.cpp
  src/ogmpp_planners/ogmpp_abstract_planner.cpp
  src/ogmpp_planners/ogmpp_planner_factory.cpp

  # PRMS 
  src/ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ogmpp_planners
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ogmpp_communications_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(ogmpp_planners
  ${catkin_LIBRARIES}
  ogmpp_graph_lib
  ogmpp_metrics_lib
  ogmpp_map_loader_lib
  ogmpp_search_algorithms_lib
)


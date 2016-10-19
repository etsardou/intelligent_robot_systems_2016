cmake_minimum_required(VERSION 2.8.3)
project(ogmpp_search_algorithms)

find_package(catkin REQUIRED COMPONENTS
  ogmpp_graph
  roscpp
)

catkin_package(
  INCLUDE_DIRS 
    include
    include/ogmpp_search_algorithms
  LIBRARIES 
    ogmpp_search_algorithms_lib
  CATKIN_DEPENDS
    ogmpp_graph 
    roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/ogmpp_search_algorithms
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ogmpp_search_algorithms_lib
   src/${PROJECT_NAME}/ogmpp_search_algorithms.cpp
   src/${PROJECT_NAME}/ogmpp_a_star.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(ogmpp_search_algorithms_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ogmpp_graph_lib
  )


cmake_minimum_required(VERSION 2.8.3)
project(ogmpp_map_loader)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  ogmpp_communications
)

catkin_package(
  INCLUDE_DIRS 
    include
    include/ogmpp_map_loader
  LIBRARIES ogmpp_map_loader_lib
  CATKIN_DEPENDS 
    nav_msgs 
    roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  include/ogmpp_map_loader
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ogmpp_map_loader_lib
  src/${PROJECT_NAME}/ogmpp_map_loader.cpp
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(ogmpp_map_loader_lib
  ogmpp_communications_gencpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  )


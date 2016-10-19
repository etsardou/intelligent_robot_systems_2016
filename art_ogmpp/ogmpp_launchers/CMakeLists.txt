cmake_minimum_required(VERSION 2.8.3)
project(ogmpp_launchers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ogmpp_planners
  ogmpp_resources
  map_server
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ogmpp_launchers
  CATKIN_DEPENDS 
    ogmpp_planners
    ogmpp_resources
    map_server
#  DEPENDS system_lib
)


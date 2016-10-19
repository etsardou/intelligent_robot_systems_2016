cmake_minimum_required(VERSION 2.8.3)
project(ogmpp_communications)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  message_generation
)
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   OgmppPathPlanningMsg.msg
 )
 add_service_files(
   FILES
   OgmppPathPlanningSrv.srv
   OgmppSetMapSrv.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
 )
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ogmpp_communications
  CATKIN_DEPENDS 
    geometry_msgs 
    nav_msgs 
    message_generation
#  DEPENDS system_lib
)


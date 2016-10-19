cmake_minimum_required(VERSION 2.8.3)
project(ogmpp_metrics)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  std_msgs
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
  INCLUDE_DIRS 
    include
    include/ogmpp_metrics
  LIBRARIES 
  CATKIN_DEPENDS 
    roscpp
    rostest 
    std_msgs
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
  include/ogmpp_metrics
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ogmpp_metrics_lib
  src/${PROJECT_NAME}/ogmpp_metrics_node.cpp 
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(ogmpp_metrics_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ executable
# add_executable(ogmpp_graph_node src/ogmpp_graph_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(ogmpp_graph_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(ogmpp_metrics_lib
  ${catkin_LIBRARIES}
  )

###########
## Tests ##
###########

#if (CATKIN_ENABLE_TESTING)
  #catkin_add_gtest(ogmpp_cell_unit_test
      #tests/ogmpp_graph/cell_unit_tests.cpp
    #)
  #target_link_libraries(ogmpp_cell_unit_test
    #${catkin_LIBRARIES}
    #ogmpp_graph_lib
    #gtest_main
    #)

  #catkin_add_gtest(ogmpp_node_unit_test
      #tests/ogmpp_graph/node_unit_tests.cpp
    #)
  #target_link_libraries(ogmpp_node_unit_test
    #${catkin_LIBRARIES}
    #ogmpp_graph_lib
    #gtest_main
    #)

  #catkin_add_gtest(ogmpp_graph_unit_test
      #tests/ogmpp_graph/graph_unit_tests.cpp
    #)
  #target_link_libraries(ogmpp_graph_unit_test
    #${catkin_LIBRARIES}
    #ogmpp_graph_lib
    #gtest_main
    #)

#endif()


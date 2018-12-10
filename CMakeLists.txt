cmake_minimum_required(VERSION 2.8.12) # version on Ubuntu Trusty
project(behaviortree_ros)

if(NOT CMAKE_VERSION VERSION_LESS 3.1)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES roscpp std_msgs message_generation behaviortree_cpp)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(GTest)

add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})


######################################################
# LIBRARIES

add_library(behaviortree_ros
    src/loggers/rosout_logger.cpp )

target_link_libraries(behaviortree_ros ${catkin_LIBRARIES})

######################################################
# TESTS


######################################################
# INSTALL


######################################################
# EXAMPLES and TOOLS




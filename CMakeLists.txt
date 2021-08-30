cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)
project(arm)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        tf
        rosserial_arduino
        rosserial_client
        message_generation
        urdf
        xacro
        )

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
        FILES
        MotorState.msg
)

add_service_files(
        FILES
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)


################################################################################
# Build
################################################################################
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(arm_controller src/arm_controller.cpp)
add_dependencies(arm_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_controller ${catkin_LIBRARIES})

add_executable(gamepad_controller src/gamepad_controller.cpp)
add_dependencies(gamepad_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gamepad_controller ${catkin_LIBRARIES})
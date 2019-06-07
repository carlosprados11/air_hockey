cmake_minimum_required(VERSION 2.8.3)
project(air_hockey)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES 
	hockey_pose.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   moveHomeMotors.srv
   todoOK.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES 
	std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  	DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Executables
add_executable(planificador src/planificador.cpp)
target_link_libraries(planificador ${catkin_LIBRARIES})
add_dependencies(planificador air_hockey_generate_messages_cpp)

add_executable(gestor_es src/gestor_es.cpp)
target_link_libraries(gestor_es ${catkin_LIBRARIES})
add_dependencies(gestor_es air_hockey_generate_messages_cpp)

add_executable(gestor src/gestor.cpp)
target_link_libraries(gestor ${catkin_LIBRARIES})
add_dependencies(gestor air_hockey_generate_messages_cpp)

add_executable(prediccion src/prediccion.cpp)
target_link_libraries(prediccion ${catkin_LIBRARIES})
add_dependencies(prediccion air_hockey_generate_messages_cpp)



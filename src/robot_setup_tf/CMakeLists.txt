cmake_minimum_required(VERSION 2.8.3)
project(robot_setup_tf)

## Catkin components required
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES robot_setup_tf
   CATKIN_DEPENDS geometry_msgs roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tf_basetolaser_broadcaster src/tf_basetolaser_broadcaster)

add_executable(tf_basetolaser_listener src/tf_basetolaser_listener)

target_link_libraries(tf_basetolaser_broadcaster
   						${catkin_LIBRARIES}
)

target_link_libraries(tf_basetolaser_listener
   						${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS tf_basetolaser_broadcaster 
				tf_basetolaser_listener
   				ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   				LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   				RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

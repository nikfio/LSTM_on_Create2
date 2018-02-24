cmake_minimum_required(VERSION 2.8.3)
project(neural_network_planner)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

message(STATUS "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  move_base_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  message_filters
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(LevelDB REQUIRED)
list(APPEND catkin_INCLUDE_DIRS PUBLIC ${LevelDB_INCLUDES})
list(APPEND catkin_LINKER_LIBS PUBLIC ${LevelDB_LIBRARIES})
list(APPEND catkin_DEFINITIONS PUBLIC -DUSE_LEVELDB)


find_package(Snappy REQUIRED)
List(APPEND catkin_INCLUDE_DIRS PRIVATE ${Snappy_INCLUDE_DIR})
list(APPEND catkin_LINKER_LIBS PRIVATE ${Snappy_LIBRARIES})

find_package(LMDB REQUIRED)
list(APPEND Caffe_INCLUDE_DIRS PUBLIC ${LMDB_INCLUDE_DIR})
list(APPEND Caffe_LINKER_LIBS PUBLIC ${LMDB_LIBRARIES})
list(APPEND Caffe_DEFINITIONS PUBLIC -DUSE_LMDB)


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
   INCLUDE_DIRS include
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs 
   DEPENDS system_lib
)

###################################
## caffe and dependencies ##
###################################

if(ON)
  if(NOT OpenCV_FOUND)
    set(Caffe_OpenCV_CONFIG_PATH "/usr/share/OpenCV")
    if(Caffe_OpenCV_CONFIG_PATH)
      get_filename_component(Caffe_OpenCV_CONFIG_PATH ${Caffe_OpenCV_CONFIG_PATH} ABSOLUTE)

      if(EXISTS ${Caffe_OpenCV_CONFIG_PATH} AND NOT TARGET opencv_core)
        message(STATUS "Caffe: using OpenCV config from ${Caffe_OpenCV_CONFIG_PATH}")
        include(${Caffe_OpenCV_CONFIG_PATH}/OpenCVModules.cmake) 
      endif()

    else()
      find_package(OpenCV REQUIRED)
    endif()
    unset(Caffe_OpenCV_CONFIG_PATH)
  endif()
endif()

# Compute paths
get_filename_component(Caffe_CMAKE_DIR "/usr/local/caffe/cmake" PATH)
#FIND_PACKAGE(Caffe)
set(Caffe_INCLUDE_DIRS /usr/local/caffe/include)
list(APPEND Caffe_INCLUDE_DIRS "/usr/include")
list(APPEND Caffe_INCLUDE_DIRS "/usr/local/cuda/include")
list(APPEND Caffe_INCLUDE_DIRS "/usr/include/opencv")
list(APPEND Caffe_INCLUDE_DIRS "/usr/include/atlas")
list(APPEND Caffe_INCLUDE_DIRS "/usr/local/caffe/build/src/")


# Definitions
set(Caffe_DEFINITIONS "-DUSE_OPENCV;-DUSE_LMDB;-DUSE_LEVELDB")

set(CAFFE_DIR /usr/local/caffe)

set(CAFFE_LIBRARY  -L${CAFFE_DIR}/build/lib -L/usr/local/Cellar/opencv/2.4.11_1/lib/ -lglog -lprotobuf -lpython2.7 -lcaffe -lm -lpthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lboost_system)


###########
## Build ##
###########


include_directories(
  include
  ${Caffe_INCLUDE_DIRS}
  ${LevelDB_INCLUDES}
  ${catkin_INCLUDE_DIRS}
)


add_library(build_database  src/build_database.cpp )

target_link_libraries(build_database ${catkin_LIBRARIES} ${BOOST_LIBRARIES} ${CAFFE_LIBRARY} ${LevelDB_LIBRARIES})

add_executable(build_database_node src/build_database_node.cpp)

target_link_libraries(build_database_node build_database)

add_executable(TestReadDB src/TestReadDB.cpp)

target_link_libraries(TestReadDB ${catkin_LIBRARIES} ${LevelDB_LIBRARIES} ${LMDB_LIBRARIES})

add_library(train_validate src/train_validate.cpp)

target_link_libraries(train_validate ${catkin_LIBRARIES} ${BOOST_LIBRARIES} ${CAFFE_LIBRARY} ${LevelDB_LIBRARIES})

add_executable(train_validate_node src/train_validate_node.cpp)

target_link_libraries(train_validate_node train_validate)

add_executable(goal_generator src/goal_generator.cpp)

target_link_libraries(goal_generator ${catkin_LIBRARIES} -lglog -lprotobuf)

add_library(online_deploy src/online_deploy.cpp)

target_link_libraries(online_deploy build_database)

add_executable(online_deploy_node src/online_deploy_node.cpp)

target_link_libraries(online_deploy_node online_deploy)

#############
## Install ##
#############


install(TARGETS build_database build_database_node TestReadDB train_validate_node online_deploy_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/neural_network_planner/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############


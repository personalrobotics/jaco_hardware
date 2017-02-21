## TODO: Check Kinova library license and add license blurb

cmake_minimum_required(VERSION 2.8.3)
project(mico_hardware)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    pr_ros_controllers
    sensor_msgs
    )

find_package(Boost REQUIRED COMPONENTS system)
link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${libkindrv_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(mico_hardware
#   src/${PROJECT_NAME}/mico_hardware.cpp
# )

## Declare a cpp executable
add_executable(mico_hardware src/main.cpp src/MicoRobot.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mico_hardware_node mico_hardware_generate_messages_cpp)
set(KINOVA_ARM_LIBS ${catkin_LIBRARIES})
set(KINOVA_ARM_LIBS 
    ${KINOVA_ARM_LIBS}
    -l:Kinova.API.CommLayerUbuntu.so
    -l:Kinova.API.USBCommandLayerUbuntu.so)


## Specify libraries to link a library or executable target against
 target_link_libraries(mico_hardware
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${KINOVA_ARM_LIBS}
 )



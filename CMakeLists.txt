cmake_minimum_required(VERSION 3.0.2)
project(battery_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(battery src/battery.cpp src/fit.cpp)
target_link_libraries(battery ${catkin_LIBRARIES})

#############
## Install ##
#############


install(TARGETS battery
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

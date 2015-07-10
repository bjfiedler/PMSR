cmake_minimum_required(VERSION 2.8.3)
project(youbot_traj_controller)

#if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE Release)
#endif()

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC") 
#set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -D__linux") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

#include_directories(include ${catkin_INCLUDE_DIRS})
#link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
        CATKIN_DEPENDS roscpp rosconsole roslib brics_actuator
)

add_executable(youbot_trajectory_controller src/youbot_trajectory_controller.cpp)

add_dependencies( youbot_trajectory_controller brics_actuator_gencpp )

target_link_libraries(youbot_trajectory_controller
   ${catkin_LIBRARIES}
)

install(TARGETS youbot_trajectory_controller
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



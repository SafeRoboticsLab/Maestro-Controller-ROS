cmake_minimum_required(VERSION 3.0.2)
project(servo_controller_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall    -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(${PROJECT_NAME}_CATKIN_DEPS
    dynamic_reconfigure
    roscpp
    std_msgs
    realtime_tools
    racecar_msgs
)


## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

generate_dynamic_reconfigure_options(cfg/calibration.cfg)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########
#SET(PYTHON_BINDING OFF CACHE BOOL "No python binding for the Pololu-Maestro")
add_subdirectory(include/thirdparty/Pololu-Maestro)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/rc_control/include/thirdparty/Pololu-Maestro/src
)

add_library(servo_controller src/servo_controller.cpp)
add_dependencies(servo_controller ${PROJECT_NAME}_gencfg)
target_link_libraries(servo_controller
  maestro
  ${catkin_LIBRARIES}
)


add_executable(servo_controller_node src/servo_controller_node.cpp)
add_dependencies(servo_controller_node ${PROJECT_NAME}_gencfg)
target_link_libraries(servo_controller_node
  servo_controller
  maestro
  ${catkin_LIBRARIES}
)

# install runtime files
install(TARGETS servo_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT main
)





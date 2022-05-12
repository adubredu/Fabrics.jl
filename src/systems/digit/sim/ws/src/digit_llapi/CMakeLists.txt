cmake_minimum_required(VERSION 3.0.2)
project(digit_llapi)

## Compiler flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  digit_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Shared Object Library - AR lowlevelapi code
add_library(llapi ${CMAKE_CURRENT_SOURCE_DIR}/src/lowlevelapi.c)
target_include_directories(llapi PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/digit_llapi
    ${CMAKE_CURRENT_SOURCE_DIR}/libs)
target_link_libraries(llapi
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/libartl/libartl.a)

#################
## Executables ##
#################
add_executable(main_llapi 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main_llapi.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Digit_Llapi.cpp
)

add_executable(main_srv_llapi
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main_srv_llapi.cpp
)

target_include_directories(main_llapi PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/digit_llapi
  ${CMAKE_CURRENT_SOURCE_DIR}/libs
)

target_include_directories(main_srv_llapi PUBLIC  
  ${CMAKE_CURRENT_SOURCE_DIR}/libs
)

add_dependencies(main_llapi 
  ${catkin_EXPORTED_TARGETS} 
  ${cpp_python_EXPORTED_TARGETS}
)

add_dependencies(main_srv_llapi 
  ${catkin_EXPORTED_TARGETS} 
  ${cpp_python_EXPORTED_TARGETS}
)

target_link_libraries(main_llapi 
  llapi
  ${catkin_LIBRARIES}
)

target_link_libraries(main_srv_llapi 
  llapi
  ${catkin_LIBRARIES}
)

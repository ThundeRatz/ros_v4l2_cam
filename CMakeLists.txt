cmake_minimum_required(VERSION 2.8.3)
project(v4l2_cam)

option(FORTIFY "Fortify GCC compilation" OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG")
set(ROSLINT_CPP_OPTS "--filter=+,-build/c++11")  # https://github.com/ros/roslint/issues/56
add_compile_options(-Wall -Wextra)
if (FORTIFY)
  add_compile_options(-fstack-protector-all -Wstack-protector --param ssp-buffer-size=4 -D_FORTIFY_SOURCE=2 -O2 -g)
endif()

find_package(catkin REQUIRED COMPONENTS
  image_transport message_generation nodelet roscpp roslib roslint sensor_msgs std_msgs
)

roslint_cpp(include/v4l2_cam/camera.h src/v4l2_cam/camera.cpp src/v4l2_cam_nodelet.cpp)
roslint_add_test()

## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

include_directories(
  SYSTEM ${catkin_INCLUDE_DIRS}
  include
)

add_library(v4l2_cam_nodelet
  src/v4l2_cam_nodelet.cpp
  src/v4l2_cam/camera.cpp
)

target_link_libraries(v4l2_cam_nodelet
  ${catkin_LIBRARIES}
)
